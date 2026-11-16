# kpi/: the frozen ruler

`kpi/` is the ruler the like_keras build is scored against: CONTRACT §10 (the reference consumer and its acceptance tests) and §11 (the scoreboard). Lane RULER wrote it at M0, and it has been read-only for every builder since. A builder diff that touches `kpi/` is rejected.

- **When a ruler test looks wrong:** write `docs/friction/ruler-NNN.md`; the ruler lane makes the change.
- **Where the contract was silent:** the readings `kpi/` pins are in [`docs/friction/contract-002.md`](../docs/friction/contract-002.md).

## Run it

```bash
nice -n 19 ionice -c3 env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/score.py --skip-fallback-suite
```

```bash
nice -n 19 ionice -c3 env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/run.py conformance
```

```bash
nice -n 19 ionice -c3 cmake --build /home/bam/like_keras/build -j 2
```

- **`kpi/run.py <suite>`** runs pytest in a private network and PID namespace (`bwrap --unshare-net --unshare-pid`). The dora suites start their own dora 0.5.0 coordinator and daemon there, never the production one on 127.0.0.1:6012. If a dora fixture finds anything listening on 6012, it refuses and skips. The run writes `build/kpi/junit/<suite>.xml`.
  - Since M0.4 each run also gets a private `XDG_RUNTIME_DIR` under `build/kpi/xdg/`, removed when the run ends. That is where like_keras serves attach sockets (CONTRACT §9.3), and a socket file is not separated by a network namespace: a peer's live deployment named `pick_cell` once failed `test_swap_variant` here.
- **`kpi/score.py`** writes `kpi/scoreboard.json` (gitignored): the four gates, the 12-number vector and the total. It compares them with the committed `kpi/baseline.json` (the empty repo, all 0). It runs `kpi/bench_overhead.py` in the same private namespace as the suites (`--pin 20,21` in the timing tier), so the cross-process A/B gets its private dora.
- **What counts as a pass:**
  - Every denominator is a frozen list of test ids, so a test that skips, errors or is missing counts as failed. A test id counts only if every one of its parametrizations passes.
  - A placement or a backend is a parametrization, so a test passes only if it passes on every placement and every backend it names.
- **Two tiers:**
  - The default tier runs everywhere.
  - Timing assertions (pytest marker `timing`, criterion 2, A1, A2, T2, T3, T4, T9's per-edge cost) run only with `KPI_TIER=timing`: with the rack unpowered, or on the fixed cloud VM. Elsewhere they skip and score 0.
  - Two checks have a default-tier form and a `timing` twin (M0.1). Criterion 10's ≥ 10× real time is measured on CPU time by default and on the wall clock as well in the timing tier. Criterion 9's `health` accepts an honest `warn`/`Overrun` by default, and must also say `ok` when unloaded in the timing tier.
  - M0.2 (CONTRACT §15 A14) adds four more, each a default-tier test plus a `timing` twin the score reads in the timing tier: T5 commands again within 5 s by default and within 1 s in the timing twin; `test_now_is_scheduled_time` and `test_forward_exception_goes_error` keep their ordering and correctness by default, with the absolute wall-clock bounds in `..._wall_clock`; `test_clients_cannot_hurt_the_node` compares the node with bad clients to a no-client window taken just before it, with the absolute tick rate in `..._unloaded`. Criterion 9's `health` also needs its lockstep half (`test_health_lockstep`, both inproc combinations, strictly `ok`) in every tier.
  - M0.3 (CONTRACT §15 A27) corrects two: `test_clients_cannot_hurt_the_node` compares against the worse of two no-client windows, one before the bad clients and one after (A15), and T11's hanging-CLI case also asserts that the refusal says the CLI `did not answer` (ruler-007 §2).
  - `kpi/load.py` refuses more than 2 spinners outside the timing tier, because the rack is powered all day and the live-rig cap applies.
- **The first consumer's friction (M0.3, CONTRACT §15 A16-A27):** one frozen test per decision, each a companion of an id or part its criterion already scores (`kpi/score.py`'s `FRICTION_*`), so weights, gates and formulas are unchanged. Each one fails until its decision lands. Their C++ boxes are CMake targets outside `all`, built by their own test (`kpi/support/lk.py` `feature_library`, `-j 1`): a decision that has not landed fails its one test and never breaks the build of the rest of the ruler.

  | Decision | Test | Counts with |
  |---|---|---|
  | A16 joint names in the C++ JointTrajectory view | `polyglot::test_T10_trajectory_joint_names[inproc,dora]` | criterion 8, the codec part |
  | A17 `In<T, Converted>` | `polyglot::test_converted_input_keeps_a_plan_off_the_rt_thread` | criterion 8, the mixed-chain part |
  | A18 SIGTERM/SIGHUP end a deployment as SIGINT does | `facilities::test_sigterm_runs_deactivate[6 cases]` | criterion 9, lifecycle |
  | A19 the RT ladder is the library's | `blackbox::test_rt_ladder_is_the_librarys` | criterion 3, every C++ row |
  | A20 `period()` | `consumer::test_period_in_activate[4 combinations]` | criterion 1, T7 |
  | A21 a Python System's facility ports | `facilities::test_system_facility_ports` | criterion 9, metrics |
  | A22 `health(level, code)`, `ErrorLogged` | `facilities::test_health_set_by_the_component[python-inproc,cpp-inproc]` | criterion 9, health |
  | A23 an optional C++ In; a verdict as a DiagnosticRecord | `polyglot::test_optional_verdict_input[same_box,target]` | criterion 8, the mixed-chain part |
  | A24 vector params | `facilities::test_vector_params[inproc,dora]` | criterion 9, params |
  | A25 `In(T, fresh=True)` | `consumer::test_fresh_input_counts_arrivals[4 combinations]` | criterion 1, T1 |
  | A26 T2 reports the export and `lk_push` apart | `consumer::test_T2_export_and_push_apart[same_box]` (`timing`) | criterion 1, T2 |

  M0.4 (the library's fix round 1, checked independently from the library verifier's findings) adds four more companions the same way, corrects A16's dora case (A29: its source `GoldenPlan` is module level, so another process can rebuild it), and makes T8 assert that the refused planner's `LeaseHeld` health names the holder (§4.5). Each new test fails on the library at `c8d43b9` and passes on the fixed one.

  | Decision | Test | Counts with |
  |---|---|---|
  | A23 for a queue input: unwired reads `nullptr` and never gates; wired, it gates | `polyglot::test_optional_queue_input[inproc,dora]` (each deployment in a child process, so a crash fails this test only) | criterion 8, the mixed-chain part |
  | A28 / A30: a plain real-time dora `stop()` delivers the final drain to live readers, with no restart | `facilities::test_dora_stop_drains_to_live_readers` | criterion 9, lifecycle |
  | A30 / T1: under lockstep a reader gets a native log line at the step that logged it, as the Python twin's | `consumer::test_log_reaches_readers_at_its_step` (fallback, same_box, target) | criterion 1, T1 |
  | A22 with §4.5: on a tie, the component's code and the facility's message | `facilities::test_health_tie_keeps_the_facility_message[python-inproc,cpp-inproc]`; and T8's holder check | criterion 9, health; criterion 1, T8 |

  A19's test asks the host what it offers first (`kpi/blackbox/rt_ladder_child.py --oracle`). Under `kpi/run.py`'s private PID namespace rtkit cannot name our threads, so there the honest answer is `SCHED_OTHER`, reported; run outside that namespace (plain pytest in the device sandbox) it expects rtkit's `SCHED_RR` on this host.
- **Where kpi writes:** nothing outside the repo. Temporary files go to `build/kpi/tmp`, the private dora's working directory is `build/kpi/dora-<pid>`, and attach sockets go under `build/kpi/xdg/` (M0.4).

## What is here

| Path | Measures |
|---|---|
| `tasks/arm_cell/` | The reference consumer (§10): `planner.py`, `plan.py`, `targets.py`, `arm_controller.cpp` (+ `jtc_core.hpp`, `guard.hpp`, the vendored robot_controllers `joint_trajectory_controller.hpp`), `arm_controller_py.py` (+ `jtc_law.py`, `guard_law.py`), `fake_bus.{hpp,py}`, `cell.py` (target / same_box / fallback), `always_on.py`, `attached_planner.py`, `swaps/` (criterion 5's frozen variants) |
| `tasks/empty`, `tasks/cpp_gain`, `tasks/relay`, `tasks/pipeline` | Criterion 9's empty-step components; criterion 8's C++ boilerplate; criterion 4's workflow tasks, each against its raw-dora or plain-Python baseline |
| `contract/` | The equivalence gate |
| `consumer/` | Criterion 1: A1-A3, T1-T12; and `test_guard_recovers_after_a_runaway` (A14: the reference guard's runaway hold ends at the next plan that starts where the arm is; C++ = Python to 1e-9 rad; not scored); `test_consumer_friction.py` (A20, A25, A26, A30) |
| `conformance/` | §5 on `inproc` and `dora` (criterion 6, part 5) |
| `churn/` | Criterion 6, parts 1-4 (T11) |
| `golden/` | §7 LKF goldens, written by `make_golden.py` with the ruler's own codec (`lkf.py`, numpy + struct). pyarrow validates each golden in full. |
| `polyglot/` | Criterion 8 (T10); `test_polyglot_friction.py` (A16, A17, A23) |
| `facilities/` | Criterion 9; `test_facilities_friction.py` (A18, A21, A22, A24, A28) |
| `twin/` | Criterion 10 |
| `attach/` | Criterion 11 |
| `blackbox/` + `blackbox.py` + `/.importlinter` | Criterion 3; `test_blackbox_friction.py` (A19) |
| `miswire/`, `validgraphs/` | Criterion 12 |
| `deps.py` | The lean-core gate |
| `fallback.py` + `fallback_manifest.json` | The fallback-untouched gate |
| `alloc_probe.cpp` | The real-time hot-path gate: the chain as configured, and with every plan running away and its guard hold released (A14) |
| `bench_overhead.py`, `bench/` | Criterion 2. `bench_chain.cpp` and `chain4.cpp` give the C++ cost per edge. |
| `surface.py` | Criteria 4 and 7, and criterion 8 part 5 |
| `swap_cost.py` | Criterion 5 |
| `load.py` | T3's 24-core load |
