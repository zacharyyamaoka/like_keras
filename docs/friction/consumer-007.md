# consumer-007: a C++ component cannot say it is in error, and its status has no type: a refused or braked chain reads health `warn` for one second, then `ok` (controllers lane, M2′, 2026-11-16)

**The ask.**

- A C++ component sets its own health: level and code (C++ `health(level, code)`, or `log.error` holding health at `error` until cleared).
- The facilities carry a status that changes between 1 Hz rounds.

## What I tried

The raw node publishes `status` on every change and at 10 Hz:

- node state, reason and joint;
- motion gate;
- plans adopted and refused, and why;
- the plan clock;
- the hand-off latencies.

Its `health` window carries the scheduler, the publish path (each joint_state gap and which leg held it) and the guard's view. A braked or refused node is an error a human must look at (`mcn/state.hpp`: braked = "a human looks first").

In the port (`lk/arm_controller.cpp`) the chain logs every state change, at error for braked, refused and bus lost, and publishes the status fields as custom metrics.

## Contract sections

§8 (health: `level` = `error` only for `Exception`, else `warn` for `InputStale`, `Overrun` or a warning logged in the last period, else `ok`; metrics at 1 Hz), §6.1 (`Log`: "fmt: a string literal; two numbers"), §15 A11 (health.code from the component's state, Python attach only).

## Repro

`lk/friction_repro/health_of_a_refused_chain.py` starts the C++ chain on a twin whose claw board is in fault. The held start refuses, so nothing is ever commanded and the chain stays refused.

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX /home/bam/like_keras/.venv/bin/python $PKG/lk/friction_repro/health_of_a_refused_chain.py
```

```
log    cell.arm.chain: level 40: start refused: nothing commanded (reason 10, joint 6)
health t=0.0 cell.arm.chain: level=ok code='' message=''
health t=1.0 cell.arm.chain: level=warn code='WarningLogged' message='start refused: nothing commanded (reason 10, joint 6)'
health t=2.0 cell.arm.chain: level=ok code='' message=''
```

The chain will never command the arm again, yet `lk top` and every health observer see `ok` from the second round on. `cpp/src/rt.cpp` `Box::facilities` sets `error` only for `broken && !child.error.empty()`, that is an exception. A logged error counts as a warning for one period.

## What I did instead

- Status as custom metrics at 1 Hz, numbers only: `node_state`, `reason`, `plans_adopted`, `plans_refused`, `refused_reason`, `plans_dropped`, `motion_blocked`, `transport_errors`, `plan_clock_s`. A state that lasts less than a second, such as a stop and a recovery between two rounds, is visible only in the log.
- State names in the log need one string literal per state and per refusal: the `state_line()` and `plan_refusal()` tables, about 40 lines. `Log` takes a literal and two numbers, so `reason_name(reason)` cannot be passed as an argument; the reason goes out as its number.
- Not ported: the publish-path account. In `same_box` the path is lk's own pump (`lk_wait` → `lk_take` → Python sinks). Its `drops.<port>` and `input_age_s.<port>` are there; each gap and the leg that held it is not.

## Proposal

- **Contract change (§6.1, §8) and library fix.** `void Component::health(const char* level, const char* code)` in C++, held until called again. Python: `self.health(level, code)`, generalising A11's `health.code`. Health then reports the component's level when it is worse than the facility's own. The arguments are string literals, so no allocation.
- **Library fix.** `log.error` sets health `error` with code `ErrorLogged` until the next `ok` round with no error logged. The same for Python (`_LogPort.emit` at ≥ 40).
- **Contract change (§6.1).** `Log` takes `const char*` arguments that are literals (`%s`), so an enum's name (`reason_name()`, static storage) can be logged without a table per format.
- **New ruler test.** A C++ and a Python component that log one error and then nothing: health stays `error` (or carries the component's own level) in every round after it, under both backends.
