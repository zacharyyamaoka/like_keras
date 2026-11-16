# consumer-018: over dora, the controller's boundary runs SCHED_OTHER and nothing counts what it loses. The raw node beside it loses 0-0.1% of its states, like_keras 2-3%, with a 10x worse hand-off tail (controllers lane, M3′ A/B, 2026-11-16; measured on a loaded host, not judged)

**The ask.** Two things the raw-dora node does that like_keras's `target` placement does not:

1. **Scheduling of the boundary.** Give `lk-dora-host`'s RX and TX threads, and the dora threads in its process, a realtime rung below the RT thread's: rtkit SCHED_RR, the same ladder as A19. That is the raw node's `dora_realtime` / `dora_priority` (RR 10 under the control thread's 20). Its `daemon_realtime` (RR 5 for a private daemon) belongs to whoever runs the daemon.
2. **Publish-path accounting.** Count, per exposed `Out`, the steps whose value never went out (superseded in the TX thread's latest slot, or refused by the transport), and the worst gap between two sends. Report them in the box's metrics, as the raw node's exit report and health do (`publish`: published, `ticks_unpublished`, `worst_gap_ms`, and which leg held a gap).

## What I tried

M3′'s A/B. The raw-dora node and the like_keras `target` placement ran back to back, 3 pairs of 30 s each, on the same twin scenario (`lk/ab_compare.py` on bam_ws `claude/cpp-controller-nodes-f9a521`):

- config/twin.ini's seven boards at 100 Hz;
- every 0.5 s, a 7 × 1000 minimum-jerk plan from the arm's command, run from the first tick after it arrives;
- a subscriber in a process of its own that keeps every state it receives (tick or `seq`, and arrival time).

## Contract sections

§6.4 (lk-dora-host's threads: main, RX, TX, RT), §15 A19 (the ladder is for the RT thread only), §8 (metrics: `drops.<port>` for queues), §11 criterion 2 (the A/B against raw dora reports throughput and loss beside latency).

## Repro

`$SBX ~/like_keras/.venv/bin/python $PKG/lk/ab_compare.py --seconds 30`. Rack powered, cores 22-23, load 11-15, not the timing tier. Thread policies were read from `/proc/<pid>/task/*/stat` 3 s into each run.

| per 30 s run | raw dora node | like_keras target |
|---|---|---|
| hand-off (call → first tick that runs the plan), p50 / p99 / max | 3.1-8.4 / 17-21 / 23-29 ms | 7.9-12.3 / 27-167 / 35-296 ms |
| states lost at the subscriber | 0 / 3 / 3 of ~3050 (0-0.1 %) | 94 / 74 / 60 of ~3010 (2.0-3.1 %) |
| worst gap between two arrivals | 82 / 63 / 48 ms | 266 / 223 / 226 ms |
| node's own count of unpublished ticks | 3 / 0 / 3, worst gap 30 / 18 / 28 ms | none: under dora the box record has `drops.log` only |
| node threads | control RR 20, 5 dora threads RR 10, 2 OTHER | RT thread RR 20 (A19), 8 threads OTHER |
| dora daemon threads | RR 5 (private `dora run`, `daemon_realtime`) | OTHER |
| worst real time between two control wakes | 12.6 / 10.2 / 13.3 ms | 10.08 / 10.02 / 10.02 ms |

The RT side matches the raw node or beats it: the worst wake is 10.0-10.1 ms against 10.2-13.3 ms, both on rtkit's SCHED_RR 20, so A19 works. The loss is all at the boundary, and like_keras reports none of it. The box's metrics under dora carry no counter for an `Out` whose value was superseded before the TX thread sent it. In same_box the box record has `drops.state` and `drops.command`, and they stay 0 there because the reader is in the process.

The planner's side over dora shows the same thing on the Python host. `lk/tests/test_target.py::test_A1_handoff_over_dora` measures the planner's return to the next step of its group, a plain call in the same thread. Over fourteen 60 s runs at load 9-39:

- p50 0.05-0.06 ms every time;
- p99 0.11-13.1 ms; in the three final runs 5.59 / 2.00 / 2.94 ms.

A1's bound is 1 ms, judged in the timing tier only. My reading, not measured further: the group thread waits for the GIL behind the host's RX thread (100 Hz commands in) and TX thread (exporting the 7 × 1000 plan), and Python's switch interval is 5 ms.

Plans also reach the node late. Lead is 50 ms, and in most runs one or a few plans of ~119 started more than a tick after their stamp: in the final runs 0 / 16 / 1, the latest 9.9 / 100.3 / 12.5 ms. In one earlier run 2 of 100 were never adopted, and the planner skipped targets while its 0.1 s-stale `command` input was stale.

The RT thread itself overran rarely, under rtkit's RR 20:

- final runs: 0 / 0 / 2 overruns, worst per-round p99 lateness 0.7 / 5.2 / 21.0 ms;
- one earlier run at load 15-16: 52 overruns and a worst round p99 of 100.7 ms, with no SIGXCPU demotion logged. I did not find the cause, and the raw node also met such host-wide stalls (51.4 ms in a memory storm, its README).

So A1's on-time start and sent = received are judged in the timing tier only in this lane's tests.

## What I did instead

Nothing in the library's place: the controller cannot reach `lk-dora-host`'s threads. A component may not name a thread or a transport (§6.1), and the RT-thread ladder is A19's, with no hook. The raw node stays the one that drives the real arm. The twin config keeps the guard deadman off, which matters here: with 266 ms arrival gaps, a guard reading `state` over dora would confirm late, and a 0.25 s `guard_timeout_s` would stop plans. That is round 2 finding 3 of the raw node, which its RR dora thread fixed.

## Proposal

- **Library fix.** `lk-dora-host` asks rtkit for SCHED_RR at a priority below the RT thread's for its RX and TX threads, and for the dora threads its process spawns, through the same ladder as A19, reported in the box record (`io_sched_policy`). The Python dora host's RX and TX threads could follow, though the GIL limits what that buys.
- **Library fix.** Count superseded and refused sends per exposed `Out` in the TX thread (`drops.<port>`, as queues already do), plus the worst gap between two sends, and publish both in the box record, in both backends.
- **New ruler test.** `kpi/bench_overhead` under `kpi/load.py --cpu 24`: the subscriber's loss on a 100 Hz `ServoState` stream over dora must not exceed raw dora's by more than its noise band, and `drops.state` must equal the subscriber's own count of missing steps.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A31-A36); its frozen test comes in ruler M0.5 (A38).

## Status from the consumer (2026-11-16, adapting to A31-A37)

Not verified yet: A35 is not in like_keras `adbac3b`, the sha this adaptation built against (its lane was editing `cpp/hosts` and `cpp/src` in the live checkout during the run). The A/B was not re-run, because without A35 it would only repeat M3′'s numbers. The port needs no change for A35 (the RT thread, the boundary threads and the accounting are the library's), so the next step is `lk/ab_compare.py --seconds 30` on the A35 sha, which ruler M0.5's `consumer::test_A35_boundary_within_noise_of_raw_dora` also runs.

One reading for A35's lane, from the A1 runs of this adaptation (60 s each, the same test as M3′): the planner's hand-off p99 was 0.118, 1.128 and 1.608 ms at load 20-23, 14-18 and 30-37, against M3′'s 2.0-5.6 ms (the RT thread's worst round p99 lateness 1.7, 1.2 and 12.6 ms; 0, 0 and 2 overruns). A late start against the stamp happened in 1 run of 3 (2 plans, worst 51.7 ms, at load 30-37). Load-labelled, not judged.

## Verified by the consumer (2026-11-16, final pass on like_keras 41ce82d)

What A35 asked for is in, and measured from the consumer's side. Its exit is half met in this set: states lost are within noise, hand-off p99 is narrowly not. Built against a `git archive` of like_keras `41ce82d`, with `lk-dora-host` built there by its own `cpp/hosts/build_dora_cxx.sh`. Run in the lane's sandbox; rack powered, cores 22-23, not the timing tier, so nothing here is judged. bam_ws `claude/cpp-controller-nodes-f9a521` @ `a99338289`.

**The A/B.** Five interleaved pairs of 30 s: raw first in even pairs, like_keras first in odd ones. `lk/ab_compare.py` now runs pairs and applies kpi's `within_noise` rule (bootstrap 95 % CI of the paired mean difference against max(floor, raw's spread)):
```
$SBX ~/like_keras/.venv/bin/python $PKG/lk/ab_compare.py --seconds 30 --pairs 5
states lost, like_keras - raw over 5 pairs: the 95 % CI's upper edge -0.007 % vs band 0.100 %: within noise
hand-off p99, like_keras - raw over 5 pairs: the 95 % CI's upper edge 9.200 ms vs band 8.801 ms: NOT within noise
```

| pair | load (lk / raw) | states lost, like_keras | states lost, raw | worst arrival gap, lk / raw (ms) | hand-off p99, lk / raw (ms) | box: `drops.state`, `send_gap_max_s.state` (ms) |
|---|---|---|---|---|---|---|
| 0 | 12 / 10 | 0 of 3005 | 1 of 3041 | 207 / 48 | 12.6 / 10.5 | 0, 14.9 |
| 1 | 15 / 13 | 0 of 3003 | 0 of 3047 | 73 / 19 | 12.2 / 8.7 | 0, 11.8 |
| 2 | 12 / 12 | 0 of 3005 | 0 of 3039 | 82 / 19 | 11.8 / 7.8 | 0, 12.2 |
| 3 | 13 / 21 | 0 of 3002 | 2 of 3061 | 133 / 45 | 28.5 / 16.6 | 0, 22.6 |
| 4 | 20 / 23 | 0 of 3002 | 1 of 3052 | 130 / 45 | 21.6 / 13.4 | 0, 13.5 |

- **States lost:** fixed. like_keras lost 0 in all 5 pairs; M3′ lost 2.0-3.1 %, and the raw node 0-0.07 % here.
- **Hand-off p99:** 11.8-28.5 ms against raw's 7.8-16.6 ms (M3′: 27-167 ms). The miss is 0.4 ms over the band, while the load rose from 10 to 23 across the set. like_keras's p50 was the lower one in 4 of 5 pairs (3.0-10.6 ms against 2.8-9.3 ms). The A35 lane's set at load 8-17 was within noise (4.7 ≤ 5.9 ms). My reading: within noise at moderate load, not yet under heavy load. The timing tier judges.
- **Scheduling, as asked:** every like_keras run reported `io_sched_policy` SCHED_RR 10 (read from a round with the RT thread running; the final drain reads OTHER because `lk_stop` demoted them).
  - `/proc` 3 s in: `lk-dora-host` had 6 threads at RR 10, the RT thread and `lk-sentinel` at RR 20, 3 OTHER.
  - The private daemon had `dora` and 3 `tokio-rt-worker` at RR 5.
  - The raw node, for comparison: 5 dora threads at RR 10, control at RR 20; its `dora run` 2 `dora` and 3 `tokio-rt-worker` at RR 5. The same ladder.
- **Accounting, as asked:** the box record carries `drops.state` and `send_gap_max_s.state` under dora. `drops.state` matched the subscriber's own count of missing states in every run (0 = 0). The worst send gap (11.8-22.6 ms) sits beside the raw node's own worst publish gap (15.5-28.5 ms).
- **RT thread:** 0 overruns in 4 runs and 1 in pair 3; worst wake period 11.1-22.5 ms.
- **CPU (CPU seconds per wall second, pair 4):** like_keras 0.24 in total, raw 0.13.
  - planner 0.062 vs 0.036;
  - subscriber 0.065 vs 0.043;
  - daemon 0.055, plus coordinator 0.0006, vs `dora run` 0.032;
  - node 0.023 vs 0.018;
  - the gateway's 0.018 has no raw counterpart.

**Still in the way, filed as consumer-022.** No state is lost, but the like_keras subscriber's worst arrival gap is 73-207 ms against raw's 19-48 ms. It sees 19-98 gaps over 20 ms per 30 s, against raw's 0-25. The large ones come every 2.000 s, and `send_gap_max_s.state` shows `lk-dora-host` sent on time, so the delay is downstream of it: the private `dora daemon`'s main thread runs about 50 ms at those moments. A Python guard reading `state` over dora meets these stalls, so the port's attached guard keeps its deadman at 1.0 s; the raw node's is 0.25 s.

**Observation:** rtkit's per-user budget is spent faster now that one dora run asks for 13-18 grants. In one suite run, an always-on test's RT thread got "SCHED_FIFO 80 refused, and rtkit refused: the RT thread runs SCHED_OTHER". The log line said so (§4.3 holds), and the test does not judge timing. The A35 lane reported the same budget.
