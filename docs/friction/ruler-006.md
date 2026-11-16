# ruler-006: two default-tier tests that only a quiet host can pass (lane dora, M3, 2026-11-16)

Lane dora hit two frozen tests that assert wall-clock rates in the default tier. On this host (rack powered, cores 22-23 at nice 19, load average 23-30) they fail for reasons the code cannot change. Both measurements below were taken outside kpi/, with probes kept in the lane's scratch folder.

## 1. `kpi/attach/test_attach.py::test_clients_cannot_hurt_the_node` asserts 280 ticks in 3 s and every gap under 50 ms

- **No client at all.** The always-on Python node (`always_on --language py`, inproc, 100 Hz) ticked 214, 269 and 254 times in three 3 s windows, with maximum gaps of 113, 38 and 36 ms. Probe: `m3/ticks.py`.
- **With a stalled and a killed attached planner.** In the same run, the same node ticked 220 (no client), 213 and 200 times, with maximum gaps of 65, 75 and 82 ms. Probe: `m3/bad_clients.py`.
- So the bad clients cost nothing measurable. The node itself does not reach 280 ticks or a 50 ms gap here.
- In the suite the test once measured 71 ticks during a load spike, and 248 and 201 in other runs.
- **Proposal:** either mark the absolute half `timing`, or compare against a no-client window taken just before it. For example: at least 0.9 × the baseline ticks, and the max gap no more than the baseline's plus 20 ms. The second keeps the test's meaning in every tier.

## 2. `kpi/consumer/test_consumer.py::test_T5_planner_crash` asserts "commands again within 1 s" in the default tier

- The 1 s covers several steps, all of them on the loaded cores:
  - a Python process start and its imports (numpy, robot_msgs, the planner);
  - the lk_hello handshake;
  - activation;
  - the first target tick;
  - `plan_pick`.
- Measured on `target`:
  - one run took 1.25 s (fixed since: the restarted node no longer skips its first target);
  - 1 run of 4 found no command within 5 s;
  - 6 runs in a row then passed.
- On `attached` (with core's two lines from contract-006 applied in a scratch copy): 1 pass and 1 fail. Under load, the restarted planner's `state` input reads stale (`stale=0.1`) target after target, so it never plans. A run once also latched the guard's `runaway` hold (`kpi/tasks/arm_cell/guard.hpp`), after which no plan can move the arm.
- **Proposal:** the 1 s bound belongs to the timing tier. In the default tier, assert that the arm holds and that the restarted planner commands again within the test's existing 5 s.

## Resolution (2026-11-16, dispatcher)

See CONTRACT.md §15, amendment A14. Decided: §1 compares against a no-client baseline; §2 moves the 1 s bound to the timing tier. The guard latch §2 mentions is fixed in the same commit: the reference guard recovers instead of latching.

**Done (ruler lane, the M0.2 commit "kpi: M0.2 — ruler amendment A14").**
- §1: `kpi/attach/test_attach.py::test_clients_cannot_hurt_the_node` measures a 3 s no-client window just before the bad clients start, then the 3 s window with them, and asserts at least 0.9 × the baseline's ticks and a longest gap no more than the baseline's + 20 ms. The absolute 280 ticks and 50 ms gap are `test_clients_cannot_hurt_the_node_unloaded` (`timing`), which `kpi/score.py` requires as well in the timing tier.
- §2: `test_T5_planner_crash` asserts that the arm holds from the end of the plan in flight until the restarted planner's first plan moves it, and that this happens within 5 s. CONTRACT §10's 1 s is `test_T5_planner_crash_within_1s` (`timing`), which `kpi/score.py` requires as well in the timing tier.
- The guard (`kpi/tasks/arm_cell/guard.hpp`, `guard_law.py`): after a runaway hold it accepts the next plan that starts within `start_tolerance_rad` (0.0524 rad, 3°) of the measured position on every joint, and logs the release; a board fault still latches. `kpi/consumer/test_consumer.py::test_guard_recovers_after_a_runaway` pins it, C++ = Python to 1e-9 rad.

**A15 (ruler lane, the M0.3 commit "kpi: M0.3 — ruler amendment A27").** `test_clients_cannot_hurt_the_node` now takes a second 3 s no-client window after the bad clients are gone, and compares against the worse of the two (the fewer ticks, the longer gap), keeping 0.9 × ticks and + 20 ms. On the same 10 runs on the powered rig (cores 22-23, not the timing tier), the M0.2 rule passed 8 and the M0.3 rule 10. `test_clients_cannot_hurt_the_node_unloaded` uses the same baseline beside its absolute bounds.
