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
