# consumer-004: SCHED_FIFO or nothing. The RT thread gets no rtkit fallback, no RLIMIT_RTTIME guard and no locked memory, and the component cannot do it itself (controllers lane, M2′, 2026-11-16)

**The ask.** Give the RT thread the raw node's real-time ladder, or a hook that lets the component run its own setup on the RT thread before the first step. The ladder is: SCHED_FIFO, else rtkit's SCHED_RR, else SCHED_OTHER, and when rtkit grants, the RLIMIT_RTTIME guard.

## What I tried

`dep.place(cell.arm, ..., cpus=[23], fifo=80)` for the ported chain in `same_box`.

## Contract sections

§4.4 (`fifo`, `cpus` apply to native groups), §6.3 (`lock_memory` only for a dedicated host process; inproc pre-faults instead), §5.1.

## Repro

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX env LD_PRELOAD=$PKG/lk/build/liblk_alloc_shim.so /home/bam/like_keras/.venv/bin/python $PKG/lk/tests/rt_probe.py $PKG/lk/build/libmcn_arm_controller.so 8 2
```

```
arm_cell.arm: SCHED_FIFO 80 refused (errno 1): the RT thread runs SCHED_OTHER
```

On this host `ulimit -r` is 0, as for every user session and lane. So the RT thread runs SCHED_OTHER at nice 0. That is honest: the warning is logged, so health says so. But it is a step down from the raw node, which sits on the same host with no extra privilege:

- `mcn/rt.hpp` `apply_realtime` asks for SCHED_FIFO, then rtkit's SCHED_RR 20 over D-Bus, then SCHED_OTHER. `health.sched` says which it got.
- The node sets RLIMIT_RTTIME soft to 100 ms and hard to 200 ms (rtkit grants only under one). A sentinel thread on a signalfd demotes a thread that crossed the soft limit instead of letting SIGKILL take the process (raw README, rounds 3, 5 and 6).
- It locks its memory, with `mlockall` after pre-faulting.
- It measured all three as necessary under this host's load. Unlocked, a page fault waited 1.67 s for the disk and the boards timed out. Without SCHED_RR, the loop stalled past the 0.25 s board watchdog at load 36.

## What I did instead

Nothing: the port runs on lk's SCHED_OTHER fallback, and A2's numbers in `lk/tests/test_rt_path.py` are taken that way. The component cannot fill the gap itself:

- `activate()` runs on the host thread, not the RT thread, so a `sched_setscheduler` or rtkit call there lands on the wrong thread.
- `forward()` is the RT thread, but rtkit is a blocking D-Bus call (busctl, 10-18 ms here). It cannot run there, and lk has no "first step" hook.
- RLIMIT_RTTIME is per process. A component in someone's Python process must not set it, and lk sets nothing.
- inproc never calls `mlockall`, by design (§6.3): it would pin the interpreter's heap. So a component in `same_box` cannot avoid a page fault on its first touch of a pool. The pools are touched when reserved, which is the most it can do.

## Proposal

- **Library fix: the scheduling ladder in `Box::run`.** If `place.fifo > 0` and SCHED_FIFO is refused, ask rtkit (`org.freedesktop.RealtimeKit1.MakeThreadRealtimeWithPID`) for SCHED_RR at min(fifo, rtkit's MaxRealtimePriority) **from a host thread before the RT thread starts its loop**, with the RT thread's tid, so the RT thread never blocks on it. Report what was granted in the box's metrics record (`sched_policy`, `sched_priority`) and on health. The raw node's `rt.cpp` and `rt_sentinel.cpp` are the prior art. Its RLIMIT_RTTIME handling has to come with it, or rtkit's own limit kills the process on the first 200 ms spin.
- **Or a contract change: an `on_rt_start()` hook.** A Component member that runs once on the RT thread before its first step, may block, and cannot touch ports. The consumer then brings its own ladder.
- **New ruler test.** In a session without an rtprio limit but with rtkit, `fifo=80` gives the RT thread a realtime policy (SCHED_RR) and says so in metrics. Today it gives SCHED_OTHER.

## Measured since: same_box on lk's SCHED_OTHER thread brakes the twin (fix round 1, 2026-11-16)

**Plainly:** in `same_box` at nice 19, SCHED_OTHER, on cores 22-23 at load about 27-33 (rack powered, not the timing tier), **3 of 7 one-minute real-time runs stopped taking plans**. An independent verifier showed one of them directly to be a **latched TIMEOUT brake**. **`same_box` must not drive the real arm until the RT ladder lands in like_keras** (rtkit SCHED_RR plus the RLIMIT_RTTIME guard, A19; or an on-RT-start hook). Since fix round 1, the port refuses a real bus in `activate()` in every placement, before any device check or bus lock. The raw-dora node drives the arm.

The runs were `lk/tests/rt_probe.py` under A2's load (a 7 × 1000 plan every 0.5 s, the chain at 100 Hz on core 23) on the shipped `config/twin.ini`, where the boards' watchdog is 0.25 s:

| Run (verifier, fix round 1) | Plans adopted (≥ 108 wanted) | What happened |
|---|---|---|
| test_rt_path run 1 | 5 | stopped taking plans after 7 hand-offs in the first half |
| test_rt_path run 2 | 80 | stopped in the second half (59 / 21 hand-offs) |
| test_rt_path run 3 | 117 | passed |
| diagnostic 1 (30 s) | 56 | passed, 415 overruns, load 28.2 / 31.9 / 31.7 |
| diagnostic 2 | 116 | passed, 550 overruns, load 26.0 / 30.8 / 31.3 |
| diagnostic 3 | 118 | passed, 524 overruns, load 29.0 / 30.2 / 31.0 |
| diagnostic 4 | 19 | **braked** about 10 s in, 380 overruns, load 27.5 / 29.6 / 30.7 |

Diagnostic 4's log tap:

```
SCHED_FIFO 80 refused (errno 1): the RT thread runs SCHED_OTHER
holding (reason 0, joint -1)
tracking (reason 0, joint -1)
braked: a human looks first (reason 7, joint 0)
```

Then `node_state` stayed 4 and `refused_reason` stayed 13 (`braked`). Reason 7 is TIMEOUT. The starved RT thread's next tick came more than 0.25 s after the last command, so the twin's boards lapsed into mode 11 while commanded. The chain brakes on that, and the brake latches by design: a human looks first. Every plan after it is refused.

My own runs since, with the chain's new `worst_period_s` metric (the longest time between two ticks on `now()`'s clock, which is what the boards' watchdog sees):

| Run (controllers lane, fix round 1) | Watchdog | Plans adopted | Overruns | Worst period | lk's worst period-error p99 | Load |
|---|---|---|---|---|---|---|
| suite run 1, test_rt_path | 5 s | 117 | 515 | 140 ms | 67.1 ms | 23.0 / 26.5 / 29.3 |
| suite run 2 | 5 s | 117 | 457 | 90 ms | 92.3 ms | 28.1 / 27.3 / 29.3 |
| suite run 3 | 5 s | 116 | 204 | 160 ms | 167.8 ms | 26.0 / 26.4 / 28.8 |
| rt_probe, shipped ini, 1 | 0.25 s | 117 | 117 | 80 ms | 75.5 ms | 22.3 / 25.3 / 28.1 |
| rt_probe, shipped ini, 2 | 0.25 s | 117 | 21 | 50 ms | 50.3 ms | 27.2 / 26.0 / 28.1 |

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX env LD_PRELOAD=$PKG/lk/build/liblk_alloc_shim.so /home/bam/like_keras/.venv/bin/python $PKG/lk/tests/rt_probe.py $PKG/lk/build/libmcn_arm_controller.so 60 2 $PKG/config/twin.ini
```

A starved tick of 160 ms leaves only 90 ms of margin to the 0.25 s watchdog. The raw node on the same host measured a worst period of 17.2 ms in a 10-minute soak at load 25-45, but with rtkit SCHED_RR 20 and locked memory (its README). The difference is the ladder this note asks for, not the chain.

What the port does about it until A19 lands:

- `lk/tests/test_rt_path.py` takes its strict measurements (0 allocations, 0 locks, the RT thread never runs Python, push cost) on a copy of `twin.ini` with `watchdog_s = 5.0`. Host starvation can no longer latch a brake and empty the measurement. The test also asserts `node_state` never reached 4, and prints the overruns and worst period.
- `test_same_box_does_not_brake_under_load` is marked `timing` (skipped unless `KPI_TIER=timing`). It runs the same probe on the shipped 0.25 s watchdog and asserts no brake, plus A2's plan count. It is the test A19 must turn green on this host's load.
- `MoteusChain::activate()` refuses `transport = fdcanusb` until the ladder lands.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).

## Verified by the consumer (2026-11-16, re-port onto A16-A28 and M3′)

Resolved by A19. Everything the port did around SCHED_OTHER is gone: the 5 s "patient twin" watchdog of the RT probe and the timing-only "does not brake" test. The probe now runs 60 s of A2's load on the shipped 0.25 s watchdog.
- Policy granted: the box record reports SCHED_RR 20 (rtkit, the library's ladder) in the device sandbox. Over 3 × 60 s on the final build: 0 brakes, 0 overruns, 115-117 plans adopted each, at load 23-35.
- In `target`: `lk-dora-host`'s RT thread runs RR 20 too. Its log line is "SCHED_FIFO 80 refused: rtkit granted SCHED_RR under RLIMIT_RTTIME".
- The real bus stays refused in `activate()`, now for the interlock gap (`mcn::kInterlockGap`, as the raw node is hold-only), not for the ladder.
- What A19 does not cover, the boundary threads, is consumer-018.
- bam_ws `claude/cpp-controller-nodes-f9a521`, `src/core/mdp/agent/actuators/moteus_controller_node/lk`; `lk/tests` green 3 of 3 back to back in the lane's sandbox (rack powered, cores 22-23, not the timing tier).
