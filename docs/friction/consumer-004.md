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

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).
