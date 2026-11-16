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

  M0.5 (CONTRACT §15 A38) freezes one test per decision on the second consumer round (A31-A36, consumer-014..019) and on the Python drain under dora (A37), the same way, before their code lands. Each fails on the library at `cede79b` (A31's box does not compile; the others run and fail).

  | Decision | Test | Counts with |
  |---|---|---|
  | A31 `from(const T&, const Owner&)`: two boxes of one `.so` in one process, each converting with its own config | `polyglot::test_conversion_sees_its_owner` | criterion 8, the mixed-chain part |
  | A32 `lk.In(T, optional=True)`: the Python peer of `verdict.cpp`, unwired and wired, equal to the C++ box | `consumer::test_optional_input_in_python[inproc,dora]` | criterion 1, T1 |
  | A33 NaN and ±inf params defaults round-trip through `lk_describe()`, `lk.native()`, `describe()` and the box | `facilities::test_non_finite_params_defaults[inproc,dora]` | criterion 9, params |
  | A34 a refused `activate()` is final under dora: `start()` raises naming it, one start, the dataflow stopped | `consumer::test_refused_activate_is_final_under_dora[realtime,lockstep]` | criterion 1, T7 |
  | A35 `io_sched_policy`, and per exposed Out the steps that never went out and the worst send gap | `facilities::test_dora_boundary_accounting` | criterion 9, metrics |
  | A35's exit: `target`'s states lost and hand-off p99 within noise of raw dora | `consumer::test_A35_boundary_within_noise_of_raw_dora` (`timing`) | criterion 1, T4 |
  | A36 the attach lease per exposed input; T8 unchanged for two writers on one input | `consumer::test_attach_lease_per_input` | criterion 1, T8 |
  | A37 a Python node's deactivate line and offline health reach live readers under a plain dora `stop()` | `facilities::test_dora_stop_drains_python_to_live_readers` | criterion 9, lifecycle |

  The readings M0.5 pins where the contract names no spelling:
  - **A35's counters** sit on the box record (path = the box's own path, A9), as consumer-018 proposed: `io_sched_policy` (Linux's policy numbers, like A19's `sched_policy`), `drops.<exposed name>` (cumulative, like a queue's `drops.<port>`) and `send_gap_max_s.<exposed name>` (seconds). The default-tier test judges the count as **exact** against a subscriber dora drops nothing for (queue 1000): at every box record, the states it missed ≤ `drops.state` ≤ the box's `steps` − the states it received. It does not demand a non-zero count, because no lever makes one on demand: dora 0.5 never makes a sender wait on a subscriber, and it buffers a stopped daemon's sends. On `cede79b` a subscriber stopped for 1.5 s cost the fast one 0-3 states (as quiet windows do), a daemon stopped for 3 s cost 0, and one stopped for 12 s cost a single run of 26. A starved TX thread is what loses states, and on this host some are lost in every run.
  - **A35's exit** is an interleaved A/B (5 pairs of 20 s) against `kpi/consumer/raw_arm_node.py`: the reference chain's own laws on raw dora 0.5.0 Python nodes, the same fake bus (its board watchdog at 5 s on both sides, so a stalled raw Python arm cannot latch it), planner and subscriber. One oracle (`kpi/consumer/boundary_ab.py`) reads both sides off the fake bus's event log. The test passes when the upper edge of a bootstrap 95 % CI of the paired difference is at most the raw side's own spread, with floors of 0.1 % of states and 1 ms of hand-off p99. It writes `build/kpi/a35_ab.json`. To let rtkit grant, run it outside a private PID namespace (`bwrap --unshare-net` only, `KPI_TIER=timing`).
  - **A34's bound:** `start()` must raise within 60 s on these loaded cores, and the node must still have exactly one start 3 s later.

  M0.6 (CONTRACT §15 A40, ruler-009) corrects one M0 case to A34: `conformance::test_activate_raising_sends_nothing[dora]` expects `start()` to raise naming the refusal, as its inproc half does, and still asserts that the reader got no data frame (M0 expected `start()` to return).

  M0.7 (CONTRACT §15 A43) freezes one test each for A41 and A42 (consumer-020, consumer-021), the same way, before their code lands. Both boxes compile on the library at `3816307`, and both tests fail there: the native box lists only the keys its config gave, and no refusal is counted or logged.

  | Decision | Test | Counts with |
  |---|---|---|
  | A41 a native box's `describe()` lists every `LK_PARAMS` field with its default, overlaid by the config given, as its Python twin with the same `Params` does | `facilities::test_native_describe_lists_param_defaults[inproc,dora]` | criterion 9, params |
  | A42 a push the box refuses is counted as `refused.<port>` in its metrics and logged on its log port | `facilities::test_refused_pushes_are_counted_and_logged[inproc,dora,before_activate]` | criterion 9, metrics |

  The readings M0.7 pins where the contract names no spelling:
  - **A41's shape** is the Python twin's. Each given key replaces its default whole (a given row table is shown as given, as Python stores what it is given), and the two datasheets compare as sorted JSON, so an int stays an int and NaN stays NaN.
  - **A42's counter** sits on the box record (path = the box's own path, A9), as consumer-021 proposed: `refused.<exposed name>`, cumulative. Its log line names the input; its text and level are not pinned, and rate limiting may keep it to one line. The host cases run in lockstep, so delivery, and with it the count, is exact on both backends.
  - **A42's `before_activate` case** drives the box through the C ABI: two pushes before `lk_start`, then one admitted and one its conversion refuses. No host pushes before `lk_start` (dora holds a frame until the box runs, and inproc drops a write until then), yet the box refuses such a push, so the box itself must count and log it.
- A19's test asks the host what it offers first (`kpi/blackbox/rt_ladder_child.py --oracle`). Under `kpi/run.py`'s private PID namespace rtkit cannot name our threads, so there the honest answer is `SCHED_OTHER`, reported; run outside that namespace (plain pytest in the device sandbox) it expects rtkit's `SCHED_RR` on this host.
- **Where kpi writes:** nothing outside the repo. Temporary files go to `build/kpi/tmp`, the private dora's working directory is `build/kpi/dora-<pid>`, and attach sockets go under `build/kpi/xdg/` (M0.4).

## What is here

| Path | Measures |
|---|---|
| `tasks/arm_cell/` | The reference consumer (§10): `planner.py`, `plan.py`, `targets.py`, `arm_controller.cpp` (+ `jtc_core.hpp`, `guard.hpp`, the vendored robot_controllers `joint_trajectory_controller.hpp`), `arm_controller_py.py` (+ `jtc_law.py`, `guard_law.py`), `fake_bus.{hpp,py}`, `cell.py` (target / same_box / fallback), `always_on.py`, `attached_planner.py`, `swaps/` (criterion 5's frozen variants) |
| `tasks/empty`, `tasks/cpp_gain`, `tasks/relay`, `tasks/pipeline` | Criterion 9's empty-step components; criterion 8's C++ boilerplate; criterion 4's workflow tasks, each against its raw-dora or plain-Python baseline |
| `contract/` | The equivalence gate |
| `consumer/` | Criterion 1: A1-A3, T1-T12; and `test_guard_recovers_after_a_runaway` (A14: the reference guard's runaway hold ends at the next plan that starts where the arm is; C++ = Python to 1e-9 rad; not scored); `test_consumer_friction.py` (A20, A25, A26, A30, A32, A34, A35's A/B, A36) |
| `conformance/` | §5 on `inproc` and `dora` (criterion 6, part 5) |
| `churn/` | Criterion 6, parts 1-4 (T11) |
| `golden/` | §7 LKF goldens, written by `make_golden.py` with the ruler's own codec (`lkf.py`, numpy + struct). pyarrow validates each golden in full. |
| `polyglot/` | Criterion 8 (T10); `test_polyglot_friction.py` (A16, A17, A23, A31) |
| `facilities/` | Criterion 9; `test_facilities_friction.py` (A18, A21, A22, A24, A28, A33, A35, A37, A41, A42) |
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
