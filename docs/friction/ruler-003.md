# ruler-003: `kpi/facilities::test_health[python-inproc]` runs on the wall clock in the default tier (criterion 9)

From lane CORE, M1. `kpi/` was not touched; the ruler lane owns any fix.

**Status (fix round 2): revised, downgraded to low priority.** Most of the failures this note first blamed on the host were core's: `inproc` counted a tick as skipped when the step before it began over half a period late, though that tick's deadline had not passed. §5.2 counts only missed ticks. Core fixed it in fix round 2 (`acb9495`: `backends/inproc.py`, `Group.loop`; `tests/test_inproc_ticks.py`). What is left is genuine deadline misses when the host is starved, which the facility reports honestly.

## What the test needs

- `run_with_sinks` runs a 50 Hz Python group (`Empty` + `Talker`, `process="subject"`) for 2.6 s of wall time, so three 1 Hz health records arrive.
- The test asserts `any(row["mode"] == "operational" and row["level"] == "ok")`: at least one 1 s period with no skipped tick.
- §8 makes `level = warn` with `code = Overrun` when a tick was skipped in the last period.

## The core half (fixed in fix round 2)

- **Old rule:** skip the next tick, and count it, whenever the forward began more than half a period (10 ms at 50 Hz) late. The deadline did not have to pass.
- **New rule:**
  - skip and count only the deadlines already past when the step ended;
  - keep "never a burst" by waiting until the later of the next deadline and half a period after this forward began.
- **Measured, rack powered, cores 22-23, not the timing tier.** An interleaved A/B, 16 runs each, load average 22-30. The probe timed every subject step and checked each skipped tick against its deadline.

  | | Before (`1fe63be`) | After (`acb9495`) |
  |---|---|---|
  | Ticks skipped | 191 | 15 |
  | ...deadline not yet passed when the step ended | 145 | 0 |
  | test_health would fail | 3/16 | 0/16 |

- **The verifier's experiment** (the same rule as a mutant): warn periods fell from 23/30 to 2/30, and failures from 3/10 to 0/24.

## What is left: real starvation

- **The interleaved health A/B** (12 runs each, load average 30-33): warn periods fell from 30/36 to 19/36, and failures from 7/12 to 4/12.
- **A 30-run probe after the fix**, during a load spike (load average up to 38), had 8 failures. In those runs the 50 Hz thread woke 20-300 ms late: the p99 period error per 1 s period was 20-320 ms, and up to 1.9 s once. Every tick it skipped had its deadline pass.
- So the test still measures whether a niced Python thread on two shared cores misses a 20 ms deadline at least once in every period. Criterion 9 scores a facility only when all four combinations pass, so one starved run still zeroes `health`.

## Suggested fix (unchanged, now low priority)

1. **Mark the test `timing`.** It then runs only where `KPI_TIER=timing`.
2. **Or accept the honest warn in the default tier:** `any(row["mode"] == "operational" and (row["level"] == "ok" or row["code"] == "Overrun"))`. Then put the `ok`-when-unloaded half in a `timing`-marked twin.

## Related

- ruler-002 item 1 is the same class of defect in `kpi/twin`: a wall-clock ≥ 10× check in the default tier.
