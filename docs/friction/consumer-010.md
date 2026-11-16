# consumer-010: a Python component has no take(), so "a new plan arrived" is guessed by identity and value, and the guess can differ from C++ under a swap (controllers lane, M2′, 2026-11-16)

**The ask.** Give a Python component the new-since-last-step signal that C++ `In<T>::take()` has, for latest inputs.

## What I tried

The C++ chain admits a trajectory once, when it arrives: `if (auto* t = trajectory.take()) admit(*t)` (`lk/arm_controller.cpp`). Its Python peer must do the same, or the swap changes behaviour. For example, a plan that comes again after a refusal is retried by one placement and not by the other.

## Contract sections

§4.1 (`forward` sees the newest value of a latest input, "sampled, never waited on"), §6.1 (`take()`: "only what arrived since the last take()"), §14 D4 (`take` is C++-only on purpose), §10 T1.

## Repro

The reference consumer's Python chain needs the same workaround and says why:

```bash
grep -n "_same_plan" -A9 /home/bam/like_keras/kpi/tasks/arm_cell/arm_controller_py.py | head -12
```

```
85:def _same_plan(trajectory, seen) -> bool:
86-    """WHY by value as well as identity: a backend may hand forward a fresh object for the same latest value."""
```

## What I did instead

`lk/arm_controller_py.py` `_same_plan`: a plan is new unless it is the same object, or has the same `header.stamp`, `t` and `positions` as the last one. It differs from C++ `take()` in two places:

- A planner that sends an identical plan again, say to retry one refused for its start, gets a new admission in C++ (a new sequence number) and none in Python (equal by value).
- When the backend hands a fresh object, the guess costs a compare of the whole plan (O(N·J)) on every step.

The swap tests pass because their planners never resend.

## Proposal

- **Contract change (§4.1, §3), with a library fix.** `lk.In(T, fresh=True)`: `forward` gets `None` for that input unless a value arrived since the last step. It is one new option of `In`, not a new public name. The backends already know the answer (each slot's sequence number). D4 keeps `take` C++-only because C++ needs "new-or-null without pointer identity". Python needs the same thing, as this case shows.
- **New ruler test.** A planner that sends the same plan object twice, 0.5 s apart, to a Python and a C++ component that count arrivals: both count 2. Today a Python component is handed the plan on every step and cannot count arrivals at all; the identity guess counts 1.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).

## Verified by the consumer (2026-11-16, re-port onto A16-A28 and M3′)

Resolved by A25. `_same_plan()` identity guessing is deleted: `trajectory = lk.In(JointTrajectory, fresh=True)` and `verdict = lk.In(DiagnosticRecord, fresh=True)`. The swap tests hold same_box and fallback commands to 0.0 rad: 3000 commands and 57 or 11 plans over the cell's targets, and 850 commands over the scripted paths. bam_ws `claude/cpp-controller-nodes-f9a521`, `src/core/mdp/agent/actuators/moteus_controller_node/lk`; `lk/tests` green 3 of 3 back to back in the lane's sandbox (rack powered, cores 22-23, not the timing tier).
