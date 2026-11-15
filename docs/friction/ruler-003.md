# ruler-003: `kpi/facilities::test_health[python-inproc]` runs on the wall clock in the default tier (criterion 9)

From lane CORE, fix round 1 of M1, measured against the frozen ruler at `b76d5b2`. `kpi/` was not touched; the ruler lane owns the fix.

## What the test needs

- `run_with_sinks` runs a 50 Hz Python group (`Empty` + `Talker`, `process="subject"`) for 2.6 s of wall time, so about three 1 Hz health records arrive.
- The test then asserts `any(row["mode"] == "operational" and row["level"] == "ok")`: at least one 1 s period with no skipped tick.
- §8 makes `level = warn` with `code = Overrun` when a tick was skipped in the last period. §5.2 forbids catching up with a burst, so a group that wakes late skips ticks and counts them.
- `inproc` counts a tick as skipped when a deadline has passed, or when a step woke more than half a period late (10 ms at 50 Hz). That keeps any two steps at least half a period apart (`backends/inproc.py`, `Thread.loop`).

## What happens on this host

- **Independent verifier, M1 fix round 1, rack powered, cores 22-23, not the timing tier:** 2 of 10 runs failed, at load average about 29.
  - In every run, health correctly reported `warn` / `Overrun` for some periods. Skips per 1 s period: 1, 2, 3, 4, 5, 6 or 8.
  - In the 2 failing runs, all three periods were `warn` / `Overrun` (for example `3`, `8` and `4 ticks skipped`). So no `ok` record arrived, and the assertion failed.
- So the test measures whether a niced Python thread on two shared cores misses a 10 ms wake-up once a second. That is the scheduler, not the health facility. When the run is starved, the facility reports exactly what §8 asks.
- Criterion 9 scores a facility only when all four combinations pass, so one starved run zeroes `health` (1/7 of criterion 9).

## Suggested fix (either one)

1. **Mark the test `timing`.** It runs only where `KPI_TIER=timing` (rack unpowered, or the fixed cloud VM).
2. **Keep it in the default tier, but accept the honest warn:** `any(row["mode"] == "operational" and (row["level"] == "ok" or row["code"] == "Overrun"))`. The facility's own content stays checked (1 Hz, mode, `expected_period_s`, never STALE). The `ok`-when-unloaded half moves to a `timing`-marked twin of the test.

Option 2 keeps criterion 9 measurable on the powered rack. Both leave the facility's behaviour unchanged.

## Related

- ruler-002 item 1 is the same defect in `kpi/twin`: a wall-clock ≥ 10× check in the default tier. It failed in both of the verifier's runs (7.1×), while the CPU time was 32-50× real time and the ten hashes were identical.
