# ruler-005: the facilities suite never deploys in lockstep, so a facility broken under lockstep scores green (lane CPP, M2 fix round 1, 2026-11-15)

**The ask.** Give `kpi/facilities/test_facilities.py::test_health` a `realtime=False` half for every combination, and let `run_with_sinks` run the fixture's deployment in lockstep. The change is about 20 lines in `kpi/`, so it belongs to the ruler lane. No builder has touched `kpi/`.

## What the ruler misses

- The fixture's `deployment(realtime=True)` already takes the flag. But `run_with_sinks` always calls it with the default, then sleeps in wall time, so every facility test runs in real time only.
- Under lockstep, `libempty.so` sent **no health record at all** before this fix round:
  - the box took its activation time from the wall clock (epoch s) and the health stamp from the logical step clock;
  - so `uptime_s` came out at about -1.79e9 s, `robot_msgs.DiagnosticRecord` refused every record, and inproc dropped them, logging `sent an invalid std/DiagnosticRecord on health` once.
- `test_health[cpp-inproc]` stayed green the whole time, so criterion 9 could not see it. The independent M2 verifier found it with its own probe (`verify-M2-cpp-r1/probes/lockstep_health.py`).
- The C++ fix landed in lane CPP's fix round 1, in `cpp/src/rt.cpp` (`Box::clock()`). This note asks only that the ruler pin it.

## The proposed change (a diff against M0.1, `61b8a23`)

```diff
 COMBINATIONS = ["python-inproc", "python-dora", "cpp-inproc", "cpp-dora"]
+CLOCKS = [pytest.param(True, id="realtime"), pytest.param(False, id="lockstep")]  # CONTRACT §4.4, §5.5

-def run_with_sinks(facility: Facility, seconds: float = 2.6, record=None) -> None:
+def run_with_sinks(facility: Facility, seconds: float = 2.6, record=None, realtime: bool = True) -> None:
+    """Real time: start, `seconds` of wall time, stop. Lockstep: `seconds` of logical time at full speed (§4.4)."""
 ...
-    dep = facility.deployment()
+    dep = facility.deployment(realtime=realtime)
 ...
-    dep.start()
-    time.sleep(seconds)
+    if realtime:
+        dep.start()
+        time.sleep(seconds)
+    else:
+        dep.start(until=seconds)
     dep.stop()

-def operational_health(facility: Facility) -> list[dict]:
+def operational_health(facility: Facility, realtime: bool = True) -> list[dict]:
 ...
-    run_with_sinks(facility)
+    run_with_sinks(facility, realtime=realtime)

+@pytest.mark.parametrize("realtime", CLOCKS)
 @pytest.mark.parametrize("combination", COMBINATIONS)
-def test_health(facility, combination):
+def test_health(facility, combination, realtime):
 ...
-    operational = operational_health(facility)
+    operational = operational_health(facility, realtime)
+    if not realtime:  # lockstep: no deadline can be missed, so every operational period is ok, in every tier
+        assert operational and all(row["level"] == "ok" for row in operational), operational
+        assert all(row["uptime_s"] >= 0.0 for row in operational), [row["uptime_s"] for row in operational]
+        return
```

- **Every combination, dora included.** §5.5 makes lockstep a rule for every backend. The `*-dora-lockstep` ids then land at M3 with the real-time dora ids. A facility counts only when its test passes in all combinations, so health on `cpp-inproc` would count only if lockstep passes too.
- **Lockstep can assert `ok` in the default tier.** No deadline can be missed in lockstep, so no `Overrun` can be reported. ruler-003's `warn/Overrun` allowance is about real time only.
- **Optional, same fixture.** `test_metrics` and `test_log` could take the same `realtime` parameter.
  - Metrics stamps are on the box's step clock.
  - Log stamps stay wall time under lockstep too (§2.3, and `test_log` already asserts it).
  - With the fix, both pass in lockstep for python-inproc and cpp-inproc (`ruler005/test_facilities_optional.py`: 8 passed).

## Evidence

The full scratch copy is `/tmp/claude-1000/-home-bam-bam-ws--claude-worktrees-cpp-controller-nodes-f9a521/8d63efe5-dad3-48eb-87e4-ce978b961621/scratchpad/lk-build/fix-M2-cpp-r1/ruler005/`, with `test_facilities.py`, `proposal.diff`, and `conftest.py`, which points `native_library` at another build. It ran with `-k "test_health and inproc and not unloaded"`, sandboxed. Rack powered, cores 22-23.

| `libempty.so` | `cpp-inproc-lockstep` | the other three inproc ids |
|---|---|---|
| pre-fix runtime (`build/v0` `c772777`, built in scratch `before/empty/build`) | **FAILED**: `AssertionError: 0 health records` | passed |
| fixed runtime (this round) | passed | passed |

The lane's own regression is `tests/test_native_facilities.py`. Before the fix, `test_lockstep_health_is_delivered[cpp]` and `test_lockstep_uptime_is_on_the_step_clock` failed with the refusal above. After the fix, all 4 tests pass.
