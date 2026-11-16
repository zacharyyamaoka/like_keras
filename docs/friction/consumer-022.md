# consumer-022: under `target`, a Python reader of the controller's `state` stalls 60-130 ms every 2.000 s; the private `dora daemon`'s main thread is busy ~50 ms each time, and the raw node's `dora run` shows no such stall (controllers lane, final pass on 41ce82d, 2026-11-16; load-labelled, not judged)

**The ask.**
- Find what the private `dora daemon` does every 2 s that holds its data path for ~50 ms, and remove it from `target`'s path. If it belongs to the coordinator + `dora daemon` setup itself, run a one-machine dataflow as `dora run`, as the raw node does.
- Give a deployer the reader's side of A35's accounting: the worst gap between arrivals per input on a reader (`arrival_gap_max_s.<port>`), beside the box's `send_gap_max_s.<port>`. A35's counters cannot show a stall that happens after the send.

## What I tried

The A35 verification A/B (consumer-018): 5 interleaved pairs of 30 s, `lk/ab_compare.py --seconds 30 --pairs 5` (bam_ws `claude/cpp-controller-nodes-f9a521` @ `a99338289`, against a `git archive` of like_keras `41ce82d`), rack powered, cores 22-23, load 10-23, not the timing tier.

- **Arrival gaps.** No state was lost on either side, but the like_keras subscriber's worst arrival gap was 73-207 ms per run against raw's 19-48 ms. It saw 19-98 gaps over 20 ms per run against raw's 0-25. The A35 lane saw the same at `bda76d9`: 62-71 ms against 14-20 ms, "cause not investigated".
- **After the send.** The box's own worst gap between two sends (`send_gap_max_s.state`, A35) was 11.8-22.6 ms in the same runs. `lk-dora-host` sent on time; the delay is after it.
- **Periodic.** In one like_keras run (`lk/ab/lk/observed.json`, pair 4), 20 gaps were over 40 ms. 15 of them (58-130 ms) start every 2.000 s, at the same phase:
  - intervals of 2.000, 2.001, 1.999, 2.002, 1.998 … s;
  - 0.707-0.712 s past the epoch second;
  - 190 ± 5 ms after every 4th plan.

  The other 5 (42-67 ms) and the smaller ones are scattered.

What it is not:

| Suspect | Experiment | Result |
|---|---|---|
| the A/B's own recording of the 168 KB plans (the gateway) | the same run with `planner.trajectory` not recorded | still 27 gaps over 20 ms, the 2 s ones 70-90 ms, every 2.000 s |
| the subscriber's garbage collector | `gc.callbacks` timing every collection in the subscriber's process for 30 s | 0 gen-2 collections; 15 stalls over 40 ms |
| this host's lane guard (it re-pins every lane thread to cores 22-23 every 2 s) | its loop is `sleep(2)` plus a walk of `/proc`, so its phase drifts against the plans | the stall's offset from the plans held at 190 ± 5 ms for 15 cycles: phase-locked to the run, not to the guard |

What it is, as far as I got: I sampled every thread of the run (the daemon, every node, the main process) every 2 ms for 20 s, reading `/proc/<pid>/task/*/stat`.
- In 3 of the 8 stalls over 40 ms, the daemon's thread named `dora` used ~50 ms of CPU inside a 60-68 ms window, and was runnable in 71-100 % of the samples.
- In the other stalls, my sampler, on the same two cores, dominated the picture.
- Under A35 that `dora` thread runs SCHED_RR 5. So while it is busy it also preempts every SCHED_OTHER thread on its core, the Python subscriber's included.

The raw node's `dora run` has the same thread names at RR 5 (`dora` ×2, `tokio-rt-worker` ×3), and shows no 2 s stall.

## Contract sections

§6.4 (`lk-dora-host` and the daemon), §9.2 (dora backend), §15 A35 (its exit: within noise of raw dora; "paused around a peer's disconnect"), §11 criterion 2 (the A/B reports throughput and loss beside latency).

## Repro

```
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --unshare-net --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1 PYTHONPATH=<like_keras 41ce82d>/src"
PKG=/home/bam/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
$SBX ~/like_keras/.venv/bin/python $PKG/lk/ab_compare.py --seconds 30 --only lk
# then, on $XDG_RUNTIME_DIR/mcn-lk/ab/lk/observed.json ([seq, arrival epoch s] per state): the gaps over 40 ms and their
# intervals; e.g. at 41ce82d, load 20:
#   20 gaps > 40 ms; 15 of them at 0.54, 2.539, 4.54, 6.54, 8.54, 10.542, 12.54, 14.539, 16.53, 18.54, 20.54, 22.543,
#   24.54, 26.544, 28.54 s from the first state, sizes 58-130 ms; 0 states lost inside any
```

The scratch probes (thread sampler, gc timing, no-plan-recording run) are in the controllers lane's session scratch. They are not committed, because each perturbs the run it measures.

## What I did instead

Nothing in the port: the daemon is the backend's. The attached guard of the always-on form (`lk/attached_guard.py`, `lk/twin_demo.sh always_on`, the A36 test) keeps its deadman at 1.0 s. The raw node runs 0.25 s, and a 130 ms stall plus the guard's own path would take half of that.

## Proposal

- **Library investigation, then a fix.** Find the daemon's 2 s work in dora 0.5.0's coordinator + `dora daemon` setup: its coordinator heartbeat, a periodic sweep, its log handling, or something the like_keras hosts send every 2 s. Do this with the daemon not raised (A35 off) as well, to separate "the daemon is busy" from "the busy daemon starves the subscriber at RR 5". If it cannot move off the data path, run a one-machine `target` under `dora run`.
- **Library fix.** Every like_keras reader host counts the worst gap between arrivals per input (`arrival_gap_max_s.<port>` in its box record). Then a deployer sees a stall after the send without an A/B.
- **New ruler test.** In kpi's A35 A/B, the Python subscriber's worst arrival gap per run must be within noise of raw dora's, by the same paired rule as states lost and hand-off p99.

## Resolution (2026-11-16, dispatcher)

CONTRACT.md §15 A46: carried into the next track (with M4 and the timing tier); not a v0 exit.
