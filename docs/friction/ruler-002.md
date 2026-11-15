# ruler-002: four kpi readings that score M1 below what it does (lane core, 2026-11-15)

From lane CORE at M1, measured against the frozen ruler at `b76d5b2`. `kpi/` was not touched; the ruler lane owns each fix. Most important first.

## 1. `kpi/twin`: the ≥ 10× real-time assertion runs on the wall clock in the default tier (criterion 10)

- `test_ten_runs_hash_identical_at_10x` times `dep.start(until=4.0)` with `time.perf_counter()`. The test is not marked `timing`, so it runs in the default tier. There, the lane rules pin every process to `nice -n 19 ionice -c3 taskset -c 22-23`, on a host at load average 30.
- **Measured, rack powered, cores 22-23, not the timing tier:**
  - all ten runs hash-identical (that part passes);
  - wall clock 1.4× real time, so the test fails;
  - the same runs on CPU time (`time.process_time()`): 0.11-0.18 s per 4 s of logical time, 22-35× real time.
- What the test measures here is the scheduler's starvation of a niced process, not like_keras. Criterion 10 then scores 3/4 for a twin that runs above 20× on CPU.
- **Suggested fix:** mark the speed half `timing`, or measure `time.process_time()` in the default tier. A second id for the speed half would also stop it hiding the determinism half.

## 2. `kpi/blackbox`: the `arm_cell.ArmControllerPy` step imports the module it blocks (criterion 3)

- The step runs `from kpi.tasks.arm_cell.cell import CONFIG` with `like_keras.backends.dora` blocked. `cell.py` imports `from like_keras.backends import dora` at module level.
- It passes today only because `like_keras.backends` resolves its backends lazily (a `LazyLoader`), so the blocked module is never executed. That laziness is real behaviour (the fallback runs where dora-rs is not installed), but the test should not depend on it.
- Criterion 3 zeroes on any dirty arm_cell component, so an eager backend import would score the whole criterion 0.
- **Suggested fix:** take `CONFIG` from `kpi.tasks.arm_cell.arm_controller_py`, or use the toml path directly.

## 3. `kpi/churn::test_wire_is_owned_by_like_keras` blocks pyarrow, then imports a module that imports it (criterion 6 part 2)

- The script blocks `pyarrow`, then runs `from kpi.golden.test_golden import emitter`. `test_golden` imports `kpi.golden.arrow_oracle`, which runs `import pyarrow as pa` at the top.
- So the script raises ImportError before like_keras records anything, whatever like_keras does. Found by reading the code at M1; it was not run, because the suite is lane dora's (M3).
- **Suggested fix:** move `emitter` (and `build`) into a module that does not import the oracle, e.g. `kpi/golden/instances.py`.

## 4. `kpi/surface.py --api` counts the contract's own `Component.__init__(**params)` as an option bag (criterion 4)

- `API_PROBE` records a class's `__init__` signature under the bare class name (`"Component"`). The exclusion list (`".add", ".wire", ".expose", ".__init__", ...`) tests `where.endswith(".__init__")`, which therefore never matches.
- So `Component(**params)` counts as a `**kwargs` option bag, though §3 and §4.1 give it that signature on purpose: the kwargs are the fields of `Params`, checked by the frozen dataclass.
- **Measured:** `kwargs_bags = 1`, so the widest-signature part scores `min(1.0, 0.667) = 0.667`, not 1.
- **Suggested fix:** record class signatures as `f"{name}.__init__"`, or except `Component` explicitly.
