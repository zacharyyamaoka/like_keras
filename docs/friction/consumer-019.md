# consumer-019: the attach write lease covers every exposed input, so a guard in its own process cannot confirm while the planner holds the lease (controllers lane, M3′, 2026-11-16)

**The ask.** In the always-on form (§10, R2), let a guard process write its verdicts (A23: the deadman's confirmation, stop, brake) while a planner process holds the lease on `trajectory`. That means a lease per exposed input (or per group of inputs), or a role that may write a guard input without the lease.

## What I tried

The raw-dora node takes a guard's `{"op": "alive" | "stop" | "brake"}` on its `command` input from any node of the dataflow. Its `guard_example.py` is a process of its own, beside the planner. A23 made the verdict a `DiagnosticRecord` input, and my always-on deployment exposes it (`lk/always_on.py`: `s.expose(arm.trajectory, arm.verdict, arm.state, arm.command)`). A planner attaches and writes `trajectory` (`lk/attached_planner.py`). A guard that attaches beside it, to write `verdict`, is a second writer.

## Contract sections

§9.3 ("one attach client at a time may write to the deployment's exposed inputs ... Any other client's write gets REFUSED naming the holder"), §14 D5 (arbitration is a write lease), §15 A23 (the verdict input), §10 R2 (the always-on form).

## Repro

`src/like_keras/attach.py:302-310`: one `self.holder` for the whole socket.

```python
if self.holder is not None and self.holder is not client and now <= self.expires:
    ... frame(REFUSED, json.dumps({"reason": "lease held", "holder": self.holder.name, "port": name}).encode())
```

So with a planner holding the lease, a separate guard's every `WRITE` on `verdict` is refused. With `guard_timeout_s` set, the chain then stops the plan as "guard silent", although the guard is alive and judging. The always-on form forces planner and guard into one attach client, which means one process. The raw node let them be two.

## What I did instead

Nothing ships that needs both. The twin config keeps the deadman off (`guard_timeout_s` = 0). The deadman and the stop and brake requests are tested in-deployment, with the guard as a sibling component (`lk/tests/test_features.py`, both placements). The always-on test attaches only a planner.

## Proposal

- **Contract change.** Keep the lease per **exposed input** (the holder table keyed by port), not per socket. The planner holds `trajectory`, the guard holds `verdict`, and each lapses and retracts on its own. T8 is unchanged for two planners on `trajectory`.
- **New ruler test.** In the always-on form, an attached planner and an attached guard write at once: both land, and killing the guard retracts only `verdict` (the chain's deadman then stops the plan), not `trajectory`.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A31-A36); its frozen test comes in ruler M0.5 (A38).

## Verified by the consumer (2026-11-16, adapting to A31-A37)

Resolved by A36. The always-on form now runs a guard as its own attach client (`lk/attached_guard.py`: it confirms every state it judged on `verdict`) beside the planner (`lk/attached_planner.py`, on `trajectory`), each in its own process, both recording their proxy's health.

`lk/tests/test_target.py::test_always_on_planner_and_guard_lapse_independently` (dora, `guard_timeout_s` 1.0 s), 3 of 3:

```
  1. planner + guard: 490 moving commands, 0 guard_silent stops; LeaseHeld refusals: {'guard-1': 0, 'planner': 0, 'guard-2': 0}
  2. guard killed: guard_silent stop 0.97 s after the kill; then 0 moving commands; plans_refused 3.0 -> 7.0 (refused_reason [15.0]); guard_age_s 2.51-4.51 s
  3. guard back: the arm moved again 0.56 s after it attached
  4. planner killed: 0 moving commands after the running plan; guard_age_s max 0.030 s; plans_adopted [23.0]; 0 guard_silent stops
```

(run 1; runs 2 and 3: 473 and 490 moving commands; the stop 0.97 s after the kill both times; the arm moved again 0.54 and 0.55 s after the new guard attached; `guard_age_s` max 0.030 and 0.040 s with the planner gone.)

- Killing the guard lapses `verdict` only: the deadman stops the running plan, and the planner's plans keep landing and are refused by the deadman (reason 15, guard_silent), never by the lease.
- Killing the planner lapses `trajectory` only: the arm holds once the running plan ends, and the guard's confirmations keep arriving.
- Neither client's writes were ever refused.
- Why 1.0 s and not the raw node's 0.3 s: this lane's dora boundary still delays states by up to ~0.3 s (consumer-018, A35 not in adbac3b). This is a lease test, not a deadman timing test.
- bam_ws `claude/cpp-controller-nodes-f9a521` @ `ed5b76c96`, `src/core/mdp/agent/actuators/moteus_controller_node/lk`, built and run against a `git archive` of like_keras `adbac3b` (the live checkout was being edited by the A35 lane); `lk/tests` green 3 of 3 back to back in the lane's sandbox, 32 passed and 1 skipped (a timing-tier test) each, rack powered, cores 22-23, load 14-37, not the timing tier.
