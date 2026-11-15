# like_keras v0: weighted success criteria (2026-11-15)

These criteria were fixed before any option existed, following Zach's rule of criteria before options. The five architecture proposals are scored against them. The build loop then measures the code against the same criteria.

They were built from:
- Zach's own KPIs: compactness, succinctness and speed ("shouldn't add anything on top").
- The consumer's needs ([consumer-spec.md](consumer-spec.md)).
- His principles in the vault ([map-vault.md](map-vault.md)).
- The KPI research ([research-kpis.md](research-kpis.md)).
- The prior art ([research-alternatives.md](research-alternatives.md), [research-ros2-control.md](research-ros2-control.md), [research-api-practice.md](research-api-practice.md)).

## Verdict

- **There are 12 weighted criteria (weights sum to 100) and 4 hard gates.** A gate is pass/fail and is never traded off. The criteria are scored only inside the gates (vault: `C - Hard Gates vs Soft KPIs`).
- **The controller case end to end carries the most weight (15)**, with speed next (13). Speed means overhead added over raw dora and over a direct call. It does not mean loop rate, because the loop rate is set by the bus.
- **Zach's two non-negotiables come next, together worth 22:** black-box ports wired from outside (11) and a succinct API (11). Compactness (8) ranks below succinctness because he said "more important for the interface to be simple than the implementation".
- **The two backend-independence criteria (swap cost 9, dora churn 8) together carry 17.** That is the reason the library exists.
- **Every criterion has a v0 target and a zero point.** The score rises linearly between them and stops at the target, so the build loop never over-pushes one criterion at the expense of another.
- **Every criterion has an automatic measurement once code exists, plus a "paper" proxy** for scoring the five proposals now. The `kpi/` scripts named below do not exist yet. They are the frozen ruler: they are committed before the build lane starts, and the builder may only read them.

## The criteria

| # | Criterion | Weight | What it measures | v0 target (score 1) | Zero point (score 0) |
|---|---|---|---|---|---|
| 1 | Consumer fit: the controller case end to end | 15 | The planner → C++ controller → bus cell runs in all 3 placements and passes the consumer's acceptance tests on a fake bus | 9/9 tests | 4/9 |
| 2 | Speed: adds nothing on top of the transport | 13 | Latency, copies and CPU added over raw dora 0.5.0, the cost per edge over a direct call, and import time | ≤ max(10 µs, 5%) over raw p99. C++ ≤ 100 ns per edge, Python ≤ 1 µs per edge. 0 extra copies. Import ≤ 20 ms | +100% p99, 1 µs / 10 µs per edge, 1 extra copy, 97 ms |
| 3 | Black-box ports, wired from outside | 11 | Components declare only their own ports. Every edge is authored outside them. No hidden remote calls or env lookups | 100% of reference components clean | 80%, or any violation in the controller task |
| 4 | API succinctness | 11 | Number of concepts (Python ∪ C++), widest signature, and the tokens a task takes compared with raw dora and plain Python | ≤ 8 top-level and ≤ 30 total names. ≤ 6 params. ≤ 0.4× raw dora, ≤ 1.2× plain Python | 11 / 43 names (lk_dora today), 10 params, 1.0× / 2.0× |
| 5 | Backend-swap cost | 9 | Lines changed, across every file, to move a task between process, language, backend and host | 0 lines in component or System files, ≤ 1 Deployment line per swap | 20 lines per swap, or any component edit |
| 6 | Insulation from dora churn | 8 | How far a dora change can reach: files naming dora, who owns the wire format, operators, version-skew refusal, a conformance suite | dora named in 1 module only. Skew refused in ≤ 2 s. 2 backends pass the conformance suite | dora named in core or in a component, or a skew hangs |
| 7 | Codebase compactness | 8 | First-party source lines (SLOC) after formatting, per part | Core ≤ 700, each backend ≤ 350, C++ runtime ≤ 900, total ≤ 2,500 | Total 5,000 |
| 8 | Python + C++ reach | 7 | One contract in both languages: mixed chains, one `.so` in 3 placements, a bit-exact codec, little boilerplate | 5/5, with ≤ 2 files and ≤ 10 non-body lines per C++ component | A C++ component needs its own hand-written dora node |
| 9 | Standard component facilities | 6 | Log, metrics, health, params, lifecycle, recording and introspection on every component, with no user code | 7/7 in both languages. Facility code ≤ 30% of SLOC | 3/7, or any facility needs user code |
| 10 | Testability with twins and sim | 5 | Twin swap, deterministic logical-clock stepping, record → replay, a component tested in isolation | 4/4, at ≥ 10× real time | A twin run needs component edits or wall-clock sleeps |
| 11 | Network attach (the IP-camera idea) | 4 | A later process lists and attaches to a running node's exported ports without restarting it | 4/4 on the same host | Attaching needs a restart or a dataflow edit |
| 12 | Wiring checked before run | 3 | Mis-wirings rejected before any process starts, naming `path.port` and the fix | 100% caught, 0 false positives | 50% |
| | **Total** | **100** | | | |

### Why these weights

- **Consumer fit (15) leads** for three reasons:
  - Adoption, not abstraction, is what killed RobMoSys/SmartSoft.
  - Scope creep killed like_keras v1–v4, and a consumer-anchored score keeps the scope honest.
  - Zach wants "the consumer can help set the spec".
- **Speed (13) is overhead, not loop rate.** The 50 Hz full arm comes from the bus: a 4-board exchange has a p95 of 9.3 ms. mjbots' own C++ tool caps 7 boards at roughly 140 Hz on the adapter's legacy firmware (consumer spec §4). What C++ buys is the tail of the loop timing, and the isolation and stall tests under criterion 1 score that tail. Criterion 2 scores only whether the library adds anything.
- **Black-box wiring (11) and succinctness (11)** are Zach's correction ("actually wrong!" about hiding remote calls behind a Python API) and his KPI #2.
- **The backend-independence pair (9 + 8)** is the reason the library exists ("what if dora changes"). It already has: dora 1.0 (2026-09-02) does not interoperate with 0.x and renames `_unstable_deploy`. Today both `lk_dora_rs/src/lk_dora/dora_backend.py:154` and the OBSBOT `dataflow.yml` emit it.
- **Compactness (8)** sits below succinctness, by his own ranking of interface over implementation.
- **Facilities (6) and network attach (4) are low on purpose.**
  - Facilities are where v1–v4 bloated: the mixins and logger alone were about 1.5k LOC.
  - The vault has no attach design.
  - dora 0.5.0 never answers a dynamic node that joins a multi-machine dataflow after it has started.
  - Raise network attach once the controller node works.

## Hard gates (pass/fail; any red means the score is 0 and the change does not land)

| Gate | Pass condition | Command |
|---|---|---|
| Equivalence | Each reference task gives **identical output streams** on recorded inputs in all three forms: in one process, across processes, and with the C++ replacement. Sent = received. | `uv run --directory /home/bam/like_keras pytest -q kpi/contract` |
| Fallback untouched | `arm_hold`, `pick_loop_v1` and `fivebar_run` behave byte for byte as they do today. like_keras stays an optional dependency of bam_rack_twin, and its tests pass with like_keras absent (consumer R15). | `uv run --directory /home/bam/bam_ws/src/dev/bam_rack_twin pytest -q` |
| Real-time hot path | 0 heap allocations per tick over 100k steady-state ticks of the C++ in-process chain. The count comes from overriding `operator new`/`malloc` in a gtest binary (gtest 1.14 is installed). No lock may be shared with a non-real-time thread (consumer R12). | `/home/bam/like_keras/build/kpi/alloc_probe` |
| Lean core | At most 1 direct runtime dependency for the core (`robot_msgs`). Backends are extras (`like_keras[dora]`). After `import like_keras`, `sys.modules` holds no `dora`, `pyarrow` or `yaml`. | `python3 /home/bam/like_keras/kpi/deps.py` |

Process rule: `kpi/` is committed before the build lane starts. A builder diff that touches `kpi/` is rejected. Consumer friction from the controllers lane (`docs/friction/NNN.md`) becomes new frozen tests, never a change to the score.

## Criterion details

### 1. Consumer fit: the controller case end to end (15)

**Definition.** The frozen reference consumer `kpi/tasks/arm_cell/` is consumer spec §2 written in the candidate API. It has three parts:
- A Python `PickPlanner` that declares only its outputs.
- A C++ `arm_controller` System: moteus bus stub → trajectory follower → guard, running in one thread in a fixed order. The bus component ports `arm_hold.py`'s take-over-then-hold.
- Three deployments of the same files:
  - `target`: the controller in its own process, with dora between.
  - `same_box`: the C++ chain on a GIL-free thread inside the Python process.
  - `fallback`: all Python, in one process.

The score is the share of 9 consumer acceptance tests that pass. They run on a fake bus with a 3.5 ms exchange and the board timeout model.

| Test | Pass condition |
|---|---|
| Swap (T1) | All three deployments send the same command on every plan tick, to 1e-9 rad. |
| Send cost (T2) | Over 1000 sends of a 7-joint × 1000-sample `JointTrajectory` while the node ticks, p99 is ≤ 0.1 ms in-process and ≤ 1 ms over dora. |
| Isolation (T3, shortened to 10 min) | The planner spins pure Python with `gc.collect()` every 50 ms, under a 24-core load. At 100 Hz, tick-start jitter is p99 ≤ 0.2 ms and max ≤ 1 ms, in both `same_box` and `target`. |
| Boundary stall (T4) | The dora daemon is held with `kill -STOP` for 4 s mid-trajectory. The trajectory still finishes on time, then holds. 0 missed ticks. `input_stale` is reported. |
| Planner crash (T5) | The planner is killed with `kill -9`. The arm holds, and the restarted planner re-attaches and commands within 1 s. |
| Node crash (T6) | The node is killed with `kill -9`. The fake boards reach timeout mode within 0.25 s. On restart the node runs `take_over`: no mode 0 and no bare stop. |
| Exclusivity (T7) | A second node is refused before it opens the port. It exits non-zero having sent 0 frames. |
| Claim (T8) | Two planners send at once. Only the claim holder runs, and the other sees a refusal on `status`. When the lease lapses, the arm holds. |
| Introspection (T12) | `lk top arm-controller` shows every R14 metric. |

Out of v0:
- Real-arm runs.
- The 1 h soak.
- Rung parity of the C++ follower. That grades the controller, not the library.
- The codec, which is scored under criterion 8.

**Measured by.**
- `uv run --directory /home/bam/like_keras pytest -q kpi/consumer`, which writes pass/fail per test to JSON.
- The 24-core load comes from `kpi/load.py --cpu 24`, a stdlib multiprocessing spin, because `stress-ng` is not installed here.
- Timing tests run only with the rack unpowered (the live-rig CPU cap) or on one fixed cloud VM. Results from different hosts are never compared.

**Paper proxy.** Write consumer spec §2 (a), (b) and (c) in the proposal's API. Score each test: 1 if the design blocks it by construction, 3 if it needs a workaround outside the API, 5 if it follows from the design. The proxy is the average.

### 2. Speed: adds nothing on top of the transport (13)

**Definition.** Four numbers. The score is the mean of their four sub-scores.

1. **Cross-process.** p99 latency through like_keras minus p99 through the raw dora 0.5.0 node API, measured in the same run. Both use the same 2 processes on the same host, pinned to cores 20 and 21.
   - Payloads are 64 B, 4 KiB (dora's `ZERO_COPY_THRESHOLD` is 4096 B), 1 MiB, and a 7×1000 float64 `JointTrajectory`.
   - Sends are open-loop at 1 kHz, and latency is timed from the scheduled send.
   - 100k samples, in at least 5 interleaved A/B pairs, with both Python and C++ senders.
2. **In-process.** Time per edge of a 4-component chain, compared with calling the same functions by hand. C++ is measured with Google Benchmark (1.8.3 is installed), Python with pyperf.
3. **Copies and CPU.** Payload copies per hop for messages ≥ 4 KiB, compared with raw dora's count. Also CPU % of an idle forwarding node at 1 kHz, compared with raw dora.
4. **Import.** Cold `import like_keras`, median of 10 runs with `python -X importtime`.

**Measured by.**
- `uv run --directory /home/bam/like_keras python kpi/bench_overhead.py --pin 20,21`
- `/home/bam/like_keras/build/kpi/bench_chain`

Never pin to core 22: the shared dora daemon has stalled there for up to 4 s. The full tier runs only with the rack unpowered, or on one fixed cloud VM.

**Paper proxy.** Count the work the design adds per message on the hot path: extra encode or decode, dict building, Python frames, locks and copies. None at all scores 5.

**Target.**

| Number | Target (score 1) | Zero point (score 0) |
|---|---|---|
| Cross-process added p99 | ≤ max(10 µs, 5% of raw p99) for every payload | +100% of raw p99 |
| In-process cost per edge | C++ ≤ 100 ns, Python ≤ 1 µs | C++ 1 µs (the consumer's R12 floor), Python 10 µs |
| Copies and CPU | 0 extra copies, CPU ≤ raw + 5% | 1 extra copy, or +50% CPU |
| Import time | ≤ 20 ms | 97 ms (lk_dora today; `yaml` was 86 ms of one trace) |

**Anti-gaming.**
- The equivalence gate requires sent = received, so dropping messages cannot help.
- Payload contents are random.
- Throughput, CPU and time to first message are reported beside latency, so a lazy import cannot hide its cost.

### 3. Black-box ports, wired from outside (11)

**Definition.** The score is the share of frozen reference components and Systems (`kpi/tasks/*`, Python and C++) for which all five rules hold:

1. The component's source names no peer, no topic or address string, no transport or backend, and no `System` or `Deployment`. It declares only its own typed input and output ports, its params and its step.
2. Every edge is authored outside the components, in a System or a Deployment. The number of edges authored in component files is 0.
3. The step runs on recorded inputs with every backend, `dora` and `pyarrow` blocked from import, and matches the recorded outputs. In other words, the framework underneath can be deleted.
4. No step makes a request/response call, such as client/service, a future, or an await on a peer. A query, if needed, is a pair of ports wired outside. This is Zach's Nov 15 correction: no ROS-style wiring inside a component.
5. Nothing outside the backends reads `os.environ`, a global registry or a config file. lk_dora's `LK_DORA_INPUT_PLAN` is the counter-example: a dependency hidden from the code.

**Measured by.**
- `uvx --from import-linter lint-imports --config /home/bam/like_keras/.importlinter`
- `python3 /home/bam/like_keras/kpi/blackbox.py`, an AST scan for forbidden imports, string addresses and env reads. For C++ it checks that component headers include only `like_keras/ports.hpp` and robot_msgs headers.
- `uv run --directory /home/bam/like_keras pytest -q kpi/blackbox`, the plain-call test, which blocks imports with a `sys.meta_path` finder.

**Paper proxy.** As the API is written, can a component name its peer or reach a transport? 5 if it is impossible by construction, 3 if it is possible but the lint catches it, 1 if it is the normal idiom.

**Target.** 100%, with 0 violations. Zero point: 80%. Any violation in the controller task alone scores 0.

### 4. API succinctness (11)

**Definition.** Three measures. The score is their mean.

1. **Concepts.** Count the names in `like_keras.__all__` (top level) and all public names (`griffe dump`). Python and C++ are counted together (the `like_keras::` public symbols in `include/`). A C++ name with no same-named Python counterpart counts again.
2. **Widest signature.** The most parameters on any public callable, plus the number of `**kwargs` and string-enum options.
3. **Workflow size.** Code tokens of the frozen reference solutions (`kpi/tasks/*/solution`), divided by a baseline:
   - for cross-process tasks, the tokens of the frozen raw-dora solutions (`dataflow.yml` plus the node scripts);
   - for in-process tasks, plain Python function composition.

**Measured by.**
- `python3 /home/bam/like_keras/kpi/surface.py --api --tasks`, using stdlib `ast` and `tokenize`. A prototype of this probe measured the lk_dora baseline.
- `uvx griffe dump like_keras -d /home/bam/like_keras/src`

Reported nightly but not scored in v0: cold-agent pass@1. A fresh `claude-sonnet-5-5` sees only the README, the DATASHEET and `help()`, then attempts held-out tasks, with n = 3. A result below 80% is flagged on the scoreboard as a sign the count is being gamed.

**Paper proxy.** Count the names and tokens in the proposal's own version of the reference tasks.

**Target.**

| Measure | Target (score 1) | Zero point (score 0) |
|---|---|---|
| Concepts | ≤ 8 top-level and ≤ 30 total | 11 / 43 (lk_dora today) |
| Widest signature | ≤ 6 params, 0 `**kwargs` or string enums | 10 params, or 3 `**kwargs` or string options |
| Workflow size | ≤ 0.4× raw dora, ≤ 1.2× plain Python | 1.0× raw dora, 2.0× plain Python |

### 5. Backend-swap cost (9)

**Definition.** Lines changed to move each frozen reference task through four swaps:
1. one process ↔ separate processes;
2. Python ↔ C++ implementation of the same box (the C++ body itself excluded);
3. dora backend ↔ in-process backend;
4. one host ↔ two hosts.

Lines are counted across every file in the task tree (`py`, `cpp`, `hpp`, `yml`, `toml`, `json`, env files, `CMakeLists.txt`), after `ruff format` / `clang-format`. The all-Python single-process fallback is swaps 1 and 2 together.

**Measured by.** `python3 /home/bam/like_keras/kpi/swap_cost.py`. It runs the formatters, then `git diff --no-index --numstat` between the frozen `kpi/tasks/<task>/<variant>` trees. Each variant must also pass the equivalence gate.

**Paper proxy.** Count the lines in the proposal's own swap example.

**Target.** 0 lines in any component or System file, ≤ 1 line in the Deployment per swap, and no hand-edited YAML. Zero point: 20 lines per swap, or any edit to a component file.

**Anti-gaming.** Every file is diffed, so placement hidden in env, YAML or auto-detection still counts. Criterion 3 rule 5 bans it outright, and `Deployment.describe()` must print the full placement.

### 6. Insulation from dora churn (8)

**Definition.** How far a dora change can reach. The score is the share of the five parts met.
1. The number of first-party files outside the dora backend directory that import or name dora (`import dora`, `dora_`, `dora-node-api`, `_unstable_deploy`, dataflow YAML).
2. like_keras defines the wire format and schema hash: the Arrow C Data Interface layout of each robot_msgs type. They do not depend on dora's metadata or Arrow major version.
3. dora is used through nodes only, never operators. Operators are outside dora 1.x's stability guarantee.
4. A dora version mismatch is refused at connect within 2 s, and the refusal names both versions. It never hangs (consumer R16 and T11).
5. One backend conformance suite (`kpi/conformance`, covering scheduling and delivery-policy semantics) passes on both the in-process backend and dora 0.5.0. A new backend is then one new module.

**Measured by.**
- `grep -rIl -i dora /home/bam/like_keras/src /home/bam/like_keras/include --exclude-dir=dora | wc -l`
- `uv run --directory /home/bam/like_keras pytest -q kpi/conformance kpi/churn`

A v1 rehearsal is reported but not scored in v0: port the backend to dora 1.1.0 on a cloud VM, then count the diff outside the backend, where the target is 0. A cloud VM is needed because daemons of different versions refuse each other.

**Paper proxy.** List every place the proposal touches dora. If it is one module, score 5.

**Target.** Part 1 is 0 files, part 2 holds, part 3 has 0 operators, part 4 refuses in ≤ 2 s, and part 5 is green on both backends. Zero point: dora is named in the core or in any component, or a version skew hangs.

### 7. Codebase compactness (8)

**Definition.** First-party source lines (SLOC): non-blank, non-comment and non-docstring, counted after `ruff format` / `clang-format`.
- Generated code is included.
- Vendored third-party code is excluded, because it counts against the lean-core gate instead.
- Tests, examples and `kpi/` are excluded.

The count is reported for four parts: the Python API core, each backend, the C++ runtime, and the total. The score is the mean over those four numbers. Each scores 0 at twice its target and 1 at its target.

**Measured by.** `python3 /home/bam/like_keras/kpi/surface.py --size`. It uses stdlib `tokenize` for Python and strips comments from C++, because `cloc` is not installed.

**Paper proxy.** The proposal's own per-module estimate, checked against lk_dora's actual sizes: `graph.py` 343, `deployment.py` 211, `dora_backend.py` 214, `runtime.py` 109 and `codec.py` 28, for 784 core SLOC in total.

**Target.** Python core ≤ 700, each backend ≤ 350, C++ runtime ≤ 900, total ≤ 2,500. Zero point: a total of 5,000. For scale, the old like_keras framework alone was about 3.7k.

**Anti-gaming.**
- `ruff format` defeats line golf.
- Complexity pushed into dependencies trips the lean-core gate.
- Complexity pushed into user code shows up in criterion 4's workflow size.

### 8. Python + C++ reach (7)

**Definition.** One component contract, written in Python or in C++, can be mixed in one System. The score is the share of these five that are met.
1. A Python → C++ → Python chain with the same port names and types runs both in one process and across processes.
2. At runtime the C++ side needs neither Python nor ROS. It builds from one CMake target with this machine's g++ 13 and cmake 3.28.
3. One compiled `.so` runs in all three placements: as a dora node, as a GIL-free thread under a Python host, and on a twin bus (consumer R11).
4. **Codec.** Every robot_msgs type the controller uses round-trips bit-exact between Python and C++, with at most 2 copies and the schema hash checked at connect (consumer R10 and T10). lk_dora's JSON-in-Arrow fails this.
   - The types are `JointTrajectory`, `JointCommandSequence`, `ServoState`, `ServoCommand`, status and health.
   - float64 values and NaN payloads must survive.
5. **Boilerplate.** Adding one C++ component touches few files and needs few lines outside its body. For comparison, ros2_control needs 9 steps and 5 files.

**Measured by.**
- `uv run --directory /home/bam/like_keras pytest -q kpi/polyglot`, which runs the mixed chain and checks the golden bytes in `kpi/golden/*.bin`.
- `python3 /home/bam/like_keras/kpi/surface.py --cpp-boilerplate kpi/tasks/cpp_gain`

Rust is reported but not scored.

**Paper proxy.** Does one component description generate or bind both languages, or are there two copies kept by hand?

**Target.** Parts 1 to 4 pass, and part 5 is ≤ 2 files and ≤ 10 non-body lines. Zero point: a C++ component needs its own hand-written dora node and YAML. That is lk_dora's C ABI today, with a blocking `dora_next_event`, raw bytes and no metadata.

### 9. Standard component facilities (6)

**Definition.** Every component, Python or C++, gets these seven facilities with zero user lines. Each one is a port wired from outside, never a side channel.

1. **log:** structured, with the tick, wall time and component path.
2. **metrics:** step count, step time p50/p99/max, overruns, input age, and drops per input.
3. **health:** OK, WARN, ERROR or STALE. UNKNOWN is the zero value, and the record carries `expected_period_s` (vault `C - Diagnostics Record`). The framework sets STALE after missed periods.
4. **params:** a frozen dataclass (a struct in C++) that round-trips through `get_config` and `from_config`.
5. **lifecycle:** configure, activate, deactivate and cleanup, driven from outside and opt-in. A restarted controller comes back inactive.
6. **recording:** a tap on any wire that replays through the same ports. It writes MCAP with log time and publish time, or npz rows in `board.FIELDS` order for the arm.
7. **introspection:** `describe()` returns JSON listing ports, types, policies, placement and transport. `lk ls` and `lk top` read it.

In C++, log and metrics never block the real-time thread. They go through a lock-free ring that is drained elsewhere, and the allocation gate checks this.

**Measured by.** `uv run --directory /home/bam/like_keras pytest -q kpi/facilities`. It deploys an empty-step component (`kpi/tasks/empty`) in each language on each backend and asserts each facility's fields. `kpi/surface.py --size` reports the facility SLOC.

**Paper proxy.** Which facilities fall out of the design, and which need code in every component?

**Target.** 7/7 in both languages with 0 user lines, and facility code ≤ 30% of first-party SLOC. The 30% cap guards against the scope creep that sank v1–v4. Zero point: 3/7, or any facility that needs user code.

### 10. Testability with twins and sim (5)

**Definition.** Four properties. The score is the share met.
1. **Twin swap.** Swapping the real bus for the twin bus is a Deployment or binding change: 0 component lines and ≤ 1 Deployment line. This is the hardware-station pattern; in ros2_control, the equivalent mock is one `<plugin>` line.
2. **Deterministic stepping.** The in-process backend runs a System on a logical clock, one step at a time, with no wall-clock sleeps. On seeded inputs, 10 runs give identical output-stream hashes. `arm_cell` on the twin runs at ≥ 10× real time.
3. **Replay.** Replaying a recording reproduces the run's outputs bit-exact, through the same ports.
4. **Isolation.** A single component can be tested on its own by feeding its input ports, with no backend and no process.

**Measured by.** `uv run --directory /home/bam/like_keras pytest -q kpi/twin`. It runs `arm_cell` on a `MoteusBusTwin` stub, hashes the output streams with sha256 over 10 runs, and runs the replay test.

**Paper proxy.** Is there a logical clock the user can step?

**Target.** 4/4. Zero point: a twin run needs component edits or real-time sleeps. Why it matters: arm time is earned by passing sim first, and the robot_controllers rung ladder already runs on a twin.

### 11. Network attach: the IP-camera idea (4)

**Definition.** A running deployment exposes the boundary ports its System exports, and only those. A process started later (same host for v0, LAN for v1) can do four things. The score is the share met.
1. List those ports, with their message types, schema hashes and policies, within 1 s (`lk ls <host>`).
2. Attach and receive `state` within 1 s, and have its `trajectory` accepted subject to the claim, with no node restart and no dataflow edit.
3. Die or go slow without affecting the node. Slow clients are dropped, never waited on, and a new client gets the cached latest state, as the OBSBOT gateway does.
4. Do all of this through an explicit wire: a CLI or a manifest naming the host and port. Auto-discovery is never the source of truth; Zach: "I cannot depend on auto discovery for connecting".

**Measured by.** `uv run --directory /home/bam/like_keras pytest -q kpi/attach`. It starts `arm_cell` on the twin, then starts a separate process 1 s later.

**Paper proxy.** Does the design have a gateway or late-join path, and does it use the same port vocabulary?

**Target (v0).** All four, on the same host. Zero point: attaching needs a restart or a dataflow edit, which is what dora 0.5.0 forces today.

### 12. Wiring checked before run (3)

**Definition.** The share of about 20 frozen mis-wirings that are rejected before any process starts. Each rejection must name `path.port`, what was expected, and the fix. The suite covers:
- a wrong port name;
- a type mismatch, and a schema-hash mismatch;
- a feedback cycle with no unit delay;
- a dangling required input, and an unexported boundary port;
- a component placed twice, or never;
- two writers on one command port (one owner per bus, ros2_control's exclusive claim);
- a delivery policy the backend cannot honour, and an unachievable rate;
- a pin to core 22.

A separate frozen suite of valid graphs must all pass, with 0 false positives.

**Measured by.** `uv run --directory /home/bam/like_keras pytest -q kpi/miswire kpi/validgraphs`.

**Target.** 100% caught, each naming `path.port` and the fix, with 0 false positives. Zero point: 50%. Any false positive scores 0.

## Scoring

- **Built code.** Each criterion scores s_i between 0 and 1, rising linearly from its zero point to its target and stopping at 1. The total is `Σ w_i s_i` (the weights sum to 100) when all four gates are green, and 0 otherwise. Always publish the 12-number vector beside the total, because the single number hides trade-offs.
- **The five proposals, on paper.** Score each criterion 1–5 using its paper proxy. The total is `Σ w_i × (score_i − 1) / 4`, which runs from 0 to 100. If a proposal fails a gate by construction, mark it out rather than scoring it.
- **Land rule for the build loop.**
  - All gates must be green.
  - The total must be at or above the committed baseline.
  - No single criterion may fall below its baseline by more than its noise band. The one exception is a commit trailer of the form `KPI-tradeoff: <criterion> <old>→<new> because …`, together with a total that rises by at least 2 points.
  - Timing criteria run as at least 5 interleaved A/B pairs and ratchet on the conservative edge of a bootstrap 95% CI.

## Assumptions these criteria bake in (challenge them here, not after the scoring)

1. **Speed means "adds nothing", not "goes faster".** If Zach actually wants the library to raise the loop rate, that is not a library property. It is the adapter firmware and the Python tail of the loop timing (consumer spec §4). Criterion 1 scores the tail. Moving to 2025 fdcanusb firmware or two buses is hardware work outside like_keras.
2. **v0 is measured on a fake bus.** Real-arm acceptance belongs to the controllers lane, after the sim gates.
3. **The C++ controller lane on raw dora is not scored here.** It is the consumer, and its friction notes feed the ruler. The later port onto like_keras is what `arm_cell` stands in for.
4. **Request/response is not banned outright.** Criterion 3 rule 4 allows a query only as a pair of ports wired outside. This settles the vault's open tension between `*_cmd`/`*_query` and "client/service is wiring within a component" in favour of external wiring. If Zach wants queries banned altogether, tighten rule 4.
5. **Network attach is weighted low** because there is no design for it yet. If the IP camera is the headline goal rather than a later one, move about 5 points to it from facilities and compactness.
