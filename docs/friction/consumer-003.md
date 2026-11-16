# consumer-003: SIGTERM ends a same_box process with no deactivate: no hand-back, no exit report (controllers lane, M2′, 2026-11-16)

**The ask.** In a process that runs a Deployment, turn SIGTERM (and SIGHUP) into the same orderly `stop()` that SIGINT reaches through `KeyboardInterrupt`. Then every component's `deactivate()` runs on the ways a process is usually ended. The raw-dora node guarantees that for its hand-back.

## What I tried

The raw node hands the arm back **on every exit after it took the bus lock** (`mcn/handback.hpp`): a dora stop, the planner gone, SIGTERM, SIGINT, SIGHUP, a refused start, an exception. The hand-back is:

- the 60 s hand-off frame;
- `last_targets.json` with the commanded holds, written under the lock;
- the lock released;
- `arm_hold.resume` touched.

It blocks those signals in every thread and takes them on one `sigwait` thread that runs the same `finish()` (`cpp/src/node_main.cpp`). Then it writes `node_exit.json`, the exit report, on every exit.

The port puts the hand-back in `MoteusChain::deactivate()` (`lk/arm_controller.cpp`, which calls `mcn::hand_back`). It runs on `dep.stop()`, which `lk/tests/test_lifecycle.py::test_stop_hands_back_holding` proves.

## Contract sections

§4.4 `stop()`: "deactivates (reverse order)". §5.7: lifecycle; a restarted process comes back through `activate`. §4.6 file 3: `signal.pause()` in `try/finally: dep.stop()`, which SIGTERM never reaches.

## Repro

`lk/friction_repro/sigterm.sh` runs `same_box` on a twin that takes its own bus lock in a temp dir, as `lk/tests/signal_child.py` written the contract's way. It ends the process once with SIGINT and once with SIGTERM.

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX bash $PKG/lk/friction_repro/sigterm.sh $PKG
```

```
SIGINT: exit 130; last_targets.json: /tmp/tmp.gqGLMvvgwo/state/last_targets.json; arm_hold.resume: /tmp/tmp.gqGLMvvgwo/state/arm_hold.resume
SIGTERM: exit 143; last_targets.json: absent; arm_hold.resume: absent
```

`systemctl stop`, `kill` and a dora stop of a Python node all send SIGTERM. On the real arm the result would be:

- the boards fall to their 0.25 s watchdog (mode 11, a bare hold without feed-forward: C1 droops about 6° per hand-off, rack 2026-11-14);
- the bus lock is released only by the process dying;
- arm-hold is never told to resume.

## What I did instead

- The hand-back runs from `deactivate()`, which covers `stop()`, SIGINT, an exception in `forward` (lk's `Box::fail` deactivates the group) and a refused `activate`. That last case hands back in the component's own `catch` before rethrowing, because lk does not call `deactivate` on the child whose `activate` threw.
- SIGTERM is left uncovered, and `test_sigterm_hands_back` is a strict xfail pointing here, so a fix shows up.
- An application could install its own SIGTERM handler, but each one would have to, and the component, which owns the arm, cannot: it is a `.so` in someone else's process.
- The exit report (`node_exit.json`: what ran, how it ended, what it handed back) has no lk equivalent. `deactivate` logs one line, but inproc's `NativeGroup.deactivate` destroys the box right after `lk_stop`, so a line logged in `deactivate` is never taken.

## Proposal

- **Library fix (inproc, the dora Python host, `lk-dora-host`).** `Deployment.start()` installs handlers for SIGTERM and SIGHUP in the main thread, if none is installed, that request `stop()` (SIGINT already reaches the `finally` through `KeyboardInterrupt`). `lk-dora-host` blocks them and takes them on a `sigwait` thread that calls `lk_stop`, as the raw node does. Restore the previous handlers on `stop()`.
- **Library fix (facilities).** Drain the log, health and metrics ports once more after the last `deactivate`, so a component's last words reach the recorder and `lk top`.
- **New ruler test.** A deployment of a component whose `deactivate` writes a file, in a child process ended by SIGTERM: the file exists, in `same_box` and in `target`.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).
