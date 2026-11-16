# contract-003: what lane core read into the contract at M1 (2026-11-15)

From lane CORE while building M1 (`src/like_keras/{__init__,component,system,deployment,codec,record}.py`, `backends/inproc.py`, the generated `cpp/include/like_keras/msgs/*.hpp`). The contract was buildable. Item 1 is the one place core wrote into another lane's path; **the dispatcher should ratify or amend each item.**

## Needs a decision (most important first)

1. **M1's exit cannot be met without `like_keras.backends.dora` importable, so core wrote a stand-in there.**
   - The frozen ruler imports it at module level in `kpi/tasks/arm_cell/cell.py`, all eight `swaps/*/deploy.py`, and the two dora graphs of `kpi/validgraphs`. So the equivalence gate (python/inproc part), A3, criterion 10 and criterion 12 (a failed valid graph is a false positive, which zeroes it) all need the module, though lane dora lands it at M3 (§12).
   - **Done:** `src/like_keras/backends/dora/__init__.py`, 20 lines: `NAME`, `VERSION = "0.5.0"`, `CAPABILITIES` as data, and a `start()` that refuses ("a stand-in until lane dora lands it"). Lane dora owns the file and replaces it whole.
   - **Amend:** §12 M1 lands a capability-only dora stub, or the ruler imports `dora` inside the functions that use it.

2. **The backend protocol (not public, §3) as core built it, for lane dora to implement:**
   - Module attributes: `NAME`, `VERSION`, `CAPABILITIES` (a tuple of strings), `address(system) -> "unix:..."`, and `start(plan, until) -> run` where `run.stop()` is idempotent. `like_keras.deployment.Plan` carries the checked groups (`order`, `hz`, `process`, `host`, `cpus`, `fifo`, `language`), `graph.edges`, `crossing` per edge, `recordings`, `realtime` and the `datasheet`.
   - Capability names core reads: `latest`, `queue`, `lagged`, `lockstep`, `host` (from §4.4's example), plus two new ones:
     - `processes`: each process is its own OS process, so describe() refuses a component it cannot rebuild as `module:qualname(**params)` (a class under `<locals>`, or Params that do not survive JSON). §4.1 says this; it needed a flag because `inproc` never rebuilds.
     - `mixed_process`: native and Python groups may share a process. `inproc` has it (same_box); dora does not, which is §4.3 check 3's "native and Python in one process under dora".
   - The Python runtime (slots `Latest`/`Queue`, `Box`, `Group`, the facilities publisher, the lockstep scheduler) lives in `backends/inproc.py` and is importable, so the dora Python host can reuse it rather than fork it.

3. **`lk.attach` and the module `like_keras/attach.py` (lane dora) share a name.** Importing the submodule rebinds the package attribute `like_keras.attach` to the module.
   - **Done:** `__init__` runs `from like_keras.attach import attach`, so the function wins. Until the module exists, a stand-in raises `NotImplementedError` (M3).
   - **The seam lane dora implements:**
     - `like_keras/attach.py` defines `attach(address, lease_s) -> Component` and `serve(run) -> server` (with `server.close()`).
     - `inproc` calls `serve(run)` when the module is loaded and the System exposes ports.
     - `run.inputs[name]` is the list of slots an attach client writes: `Latest.write(value, sent)` or `Queue.push(value, sent)`. A retraction is `write(None, sent)` or emptying the queue.
     - `run.outputs[name]` is a `Latest` holding the newest value of each exposed Out.

4. **§7.4's "a boundary copy of one is < 1 KiB" does not hold with the member spelling contract-002 pinned, at `kMaxJoints = 16`.** Measured (g++ 13, the generated views):

   | Type | Size |
   |---|---|
   | `ServoState` | 1104 B |
   | `ServoCommand` | 1128 B |
   | `JointState` | 688 B |

   Sixteen 16-byte `std::string_view` names come to 256 B, and every optional array costs 136 B.
   - **Amend:** "about 1 KiB", or names as `const char*` (8 B).
   - **Fix round 2 (verifier confirmed the sizes):** core cannot change the layout alone. The frozen ruler's C++ pins it:
     - `kpi/tasks/arm_cell/jtc_core.hpp:106` copies `command.name[joint] = state.name[joint]`;
     - `kpi/tasks/arm_cell/fake_bus.hpp:155` and `:181` assign a `std::string` into `name[joint]`;
     - contract-002 item 1 spells the member `std::array<std::string_view, lk::kMaxJoints> name`.

     A `const char*` array, or a span over names held outside the value, breaks those lines. No ruler test measures the bound. **Recommendation:** amend §7.4 to "about 1.1 KiB at `kMaxJoints = 16` (`ServoState` 1104 B, `ServoCommand` 1128 B)". If the bound must hold, the ruler changes those three lines, and core then makes the generator emit `std::span<const std::string_view> name`. That is 16 B, so `ServoState` becomes 864 B and `ServoCommand` 888 B.

5. **The process-name rule is generalised so the core never names dora (criterion 6).** §4.3 refuses a process "equal to `dora`". The core instead refuses a process named after *any* module in `like_keras.backends`, found by `pkgutil`, without importing it.

## Readings unlikely to be contested (pinned in the code)

- **Lockstep visibility.** A queue input of a rate-less group sees messages sent at or before its step time (its trigger at t). Its latest inputs see only what was sent strictly before t, which is what `kpi/support/arm_cell_oracle.py` encodes. A lagged same-group input reads the strict view: the writer's previous step.
- **Never a burst (§5.2) is measured at the start of the step's first forward, not at the thread's wake-up** (fix round 1, `92cb227`). The input reads in between (in lane dora's Python host, decoding) count as lateness.
  - **Fix round 2 (`acb9495`):** a tick is skipped, and counted in `overruns`, only when its deadline had passed as the step before it ended. A forward that began more than half a period late *delays* the next step to half a period after it. Round 1 *gave that tick up* and counted it, which over-reported `Overrun` (ruler-003).
- **A queue under lockstep holds a message in `arriving` until a step strictly after its send time.** Otherwise a message sent at t would evict one that the reader stepping at t is still due (`test_queue_drop_oldest` expects 8, 18, 28, ...).
- **Recording topics:**
  - **Explicit ports:** the name the user wired, e.g. `pick_cell.arm.command`.
  - **No ports given:** `<system>.<name>` for each exposed Out, and the leaf's `<path>.<port>` for each edge between groups. There is one channel per leaf Out, and Python-object ports are skipped.
  - **A replay Out** keeps its source channel's topic, so recording a replayed port gives back the original channel (`test_like_keras_decodes_golden` looks it up by `.value`).
  - **Exposed inputs** are not recorded at M1: attach writes them (M3).
- **Replay port names** come from the topic minus the System's own name, with the dots turned to `__`. `pick_cell.targets.target` becomes `targets__target`, and `pick_cell.command` becomes `command`.
- **`describe()` edges use leaf paths** (`pick_cell.arm.bus.state` for the Python arm). §4.4's example shows the boundary name because its arm is a native leaf. `exposed` maps each boundary name to the leaf port behind it.
- **A System may name a child or an exposed port `log`, `health` or `metrics`.** A System has no facility ports of its own, and `kpi/conformance` adds sinks under those names.
- **Health codes:**
  - `Exception` for an error in forward.
  - `InputStale` for a stale input.
  - `Overrun` when ticks were skipped in the last period.
  - `WarningLogged` when a warning was logged in the last period, with its text. §8 names no code for this case, and DiagnosticRecord requires one for `warn`.
  - Level `unknown` before activation.
- **Python receive builds each message through its robot_msgs constructor.** Arrays are `adopt`ed read-only views into the frame, so there is no copy, but validation runs. §7.3's "without re-validation" is an optimisation left open.
- **Facility C++ types use `rm::kMaxNames = 64`** (contract-002 item 8). Per-joint types use `lk::kMaxJoints`.
- **The JointTrajectory view** carries `t`, `positions`, `velocities`, `accelerations`, `effort` and `header`, as contract-002 lists them. `joint_names`, `channels` and `limits` stay Python-side in v0.
- **`backends` load on first use, through the import system.** A deployment file can name `from like_keras.backends import dora` and still run its all-Python fallback where dora-rs is not installed (the lean core): the backend's body runs at its first attribute access. The lookup goes through `sys.meta_path` (fix round 1, `1c42c81`), so an import blocker or hook sees the backend like any other import. `kpi/blackbox`'s ArmControllerPy step is therefore red until `cell.py` stops importing dora at module level; see ruler-002 item 2.

## Not done at M1, on purpose

- **The inproc native data path:** `lk_push`/`lk_take` over Arrow C Data structs, and `lk_step` under lockstep. M2 builds it on the C++ runtime.
  - **Seam:** `backends/inproc.py:NativeGroup` loads the `.so` with `ctypes` (CDLL for create, start, wait and stop; PyDLL for the never-blocking push, take and step, per §6.3), calls `lk_create` so a parameter error surfaces at `start()`, then refuses with "lands at M2".
  - `lk.native()` reads `lk_abi_version()` and `lk_describe()` today, so every native check of §4.3 already runs.
- **`[project.scripts] lk`.** It waits for `cli.py` (lane dora): an `lk` on PATH that cannot import would shadow `python -m like_keras.cli` in `kpi/support/lk.py`.

## Resolution (2026-11-15, dispatcher)

See CONTRACT.md §15 (amendments A1-A7). Decided: A2 (stub ratified), A1 (ruler moves the dora imports), A4 (bound amended), A6 (readings ratified).
