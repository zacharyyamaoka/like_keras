# ruler-002: four kpi readings that score M1 below what it does (lane core, 2026-11-15)

From lane CORE at M1, measured against the frozen ruler at `b76d5b2`. `kpi/` was not touched; the ruler lane owns each fix. Most important first.

## 1. `kpi/twin`: the ≥ 10× real-time assertion runs on the wall clock in the default tier (criterion 10)

- `test_ten_runs_hash_identical_at_10x` times `dep.start(until=4.0)` with `time.perf_counter()`. The test is not marked `timing`, so it runs in the default tier. There, the lane rules pin every process to `nice -n 19 ionice -c3 taskset -c 22-23`, on a host at load average 30.
- **Measured at `84c1e21`, rack powered, cores 22-23, not the timing tier:**
  - all ten runs are hash-identical (that part always passes);
  - in a direct pytest run: wall clock 1.4× real time, so the test failed;
  - in `kpi/score.py`'s run 30 minutes later: it passed, so criterion 10 scored 1.0;
  - on CPU time (`time.process_time()`): 0.11-0.18 s per 4 s of logical time, 22-35× real time.
- **Independent verifier, fix round 1, same commit, same environment:** the wall-clock check failed in both of its runs (7.1× real time), while the CPU-time speed-up was 32-50× and the ten hashes were identical.
- **Lane core's two scoreboard runs in fix round 1** (`6c40de6`, `92cb227`, same environment) failed it too: the slowest run was 8.9× and 7.1× real time, and the ten hashes were identical both times.
- So the test is flaky by environment: it measures the scheduler's starvation of a niced process, not like_keras, and criterion 10 can swing by 1/4 between two runs of the same commit.
- **Suggested fix:** mark the speed half `timing`, or measure `time.process_time()` in the default tier. A second id for the speed half would also stop it hiding the determinism half.

## 2. `kpi/blackbox`: the `arm_cell.ArmControllerPy` step imports the module it blocks (criterion 3)

- The step runs `from kpi.tasks.arm_cell.cell import CONFIG` with `like_keras.backends.dora` blocked. `cell.py` imports `from like_keras.backends import dora` at module level.
- **Corrected in fix round 1 (the first text said the row passed because the blocked module was never executed; that was the wrong reason):**
  - At `84c1e21` the row passed because `like_keras.backends.__getattr__` looked the backend up with `importlib.machinery.PathFinder.find_spec`, which bypasses `sys.meta_path`. The step's Blocker never saw `like_keras.backends.dora`, so the from-import succeeded, and any attribute access (`dora.NAME`) would have run the blocked module. That was a core bug, not the test's.
  - At `1c42c81` core looks the backend up with `importlib.util.find_spec`, through `sys.meta_path`, and keeps the `LazyLoader`. The Blocker's ImportError now propagates from the from-import, so **this row is red**, and criterion 3 scores 0 (it zeroes on any dirty arm_cell component). That is the honest state until `cell.py` stops importing dora at module level.
- **Suggested fix (unchanged):** take `CONFIG` from `kpi.tasks.arm_cell.arm_controller_py`, or use the toml path directly. Or move `from like_keras.backends import dora` inside the function that builds the dora deployment, as `kpi/tasks/relay/solution/relay.py` already does.

## 3. `kpi/churn::test_wire_is_owned_by_like_keras` blocks pyarrow, then imports a module that imports it (criterion 6 part 2)

- The script blocks `pyarrow`, then runs `from kpi.golden.test_golden import emitter`. `test_golden` imports `kpi.golden.arrow_oracle`, which runs `import pyarrow as pa` at the top.
- So the script raises ImportError before like_keras records anything, whatever like_keras does. Found by reading the code at M1; it was not run, because the suite is lane dora's (M3).
- **Suggested fix:** move `emitter` (and `build`) into a module that does not import the oracle, e.g. `kpi/golden/instances.py`.

## 4. `kpi/surface.py --api` counts the contract's own `Component.__init__(**params)` as an option bag (criterion 4)

- `API_PROBE` records a class's `__init__` signature under the bare class name (`"Component"`). The exclusion list (`".add", ".wire", ".expose", ".__init__", ...`) tests `where.endswith(".__init__")`, which therefore never matches.
- So `Component(**params)` counts as a `**kwargs` option bag, though §3 and §4.1 give it that signature on purpose: the kwargs are the fields of `Params`, checked by the frozen dataclass.
- **Measured:** `kwargs_bags = 1`, so the widest-signature part scores `min(1.0, 0.667) = 0.667`, not 1.
- **Suggested fix:** record class signatures as `f"{name}.__init__"`, or except `Component` explicitly.

## Fix round 2 (2026-11-15): the verifier's independent run confirmed items 1, 2 and 4

- **Item 1.** It ran at `1fe63be`, rack powered, cores 22-23, not the timing tier.
  - The wall clock measured 6.7× real time in the scoreboard and 4.25× in a probe.
  - CPU time measured 36.9-60.2× real time.
  - All ten hashes were identical both times.

  The fix is unchanged: mark the speed half `timing`, or use `time.process_time()` in the default tier. Give it its own test id, so a slow run cannot hide the determinism half.
- **Item 2.** `cell.py:15`'s module-level dora import is still the blocker. The fix is unchanged: move it into the function that builds the dora deployment. See contract-004.
- **Item 4.** The exact fix: at `kpi/surface.py:185`, record a class's signature as `f"{name}.__init__"` rather than `name`, so the `.__init__` exclusion at line 200 matches. `widest_signature` then scores 1.0 rather than 0.667.

## Resolution (2026-11-15, dispatcher)

See CONTRACT.md §15 (amendments A1-A7). Decided: A7, the ruler lane fixes it in the M0.1 commit.
