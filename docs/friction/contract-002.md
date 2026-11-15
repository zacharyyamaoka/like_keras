# contract-002: what the ruler had to read into the frozen contract (M0, 2026-11-15)

From lane RULER while writing `kpi/`. The contract was buildable; none of this blocks a lane. These are the places where it was silent or ambiguous. `kpi/` pins one reading of each, and the build lanes have to match it. **The dispatcher should ratify each item or amend the contract.** Each item gives the reading `kpi/` took and the file that pins it.

## Items that need a decision (most important first)

1. **C++ message view spelling (§7.4).** The contract fixes the shape of the generated views but not the member names. kpi's C++ (`tasks/arm_cell/*.cpp|hpp`, `polyglot/echo.cpp`, `bench/*.cpp`) uses the following:
   - **Per-joint types:** `std::size_t size`; `std::array<std::string_view, lk::kMaxJoints> name`; one `std::array<double, kMaxJoints>` per array field (`std::int64_t` for `ServoState.fault`); `std::optional<...>` for every `X | None` field, arrays and scalars alike; `std::int64_t` for `int`; `rm::Header header` holding `double stamp` and `std::string_view frame_id` (`meta` is not exposed).
   - **`JointTrajectory`:** `std::span<const double> t, positions`; `std::optional<std::span<const double>> velocities, accelerations, effort`; `rm::Header header`. The joint count is `positions.size() / t.size()`.
   - **Headers:** one per domain, `like_keras/msgs/{std,sensor,control,trajectory}.hpp`.
   - **Evidence:** a scratch stub of these headers compiles every kpi C++ file.
2. **The CMake seam (§1, §6).** `kpi/CMakeLists.txt` assumes `cmake/like_keras.cmake` defines two things. First, `lk_component(<target> <sources...>)`, which makes `lib<target>.so` in `CMAKE_LIBRARY_OUTPUT_DIRECTORY`. Second, the target `like_keras::rt`, whose include directories hold `lk.hpp`, `abi.h` and `msgs/`.
3. **`lk_describe()` JSON and C ABI port indices (§6.3).** The reading is: `{"like_keras", "abi", "system", "ports": {name: {"dir", "type", "hash", "queue", "stale", "lagged"}}, "params"}`. The port records have the shape of `describe()`'s. `in_port` and `out_port` are a port's position among the exposed ports of its direction, in that `"ports"` order (`kpi/alloc_probe.cpp`, `kpi/miswire/make_fake_native.py`).
4. **`JointTrajectory.channels` is `Mapping[str, Array]`, which is outside §7.1's table.** But `JointTrajectory` is a v0 type. The reading is a map of utf8 to list<float64> over the flat C-order buffer, so a channel's shape and dtype are lost. The schema hash of `trajectory/JointTrajectory` depends on this choice (`kpi/golden/lkf.py`).
5. **`ServoState.fault` is int64, but robot_msgs annotates it as plain `Array`.** It coerces the dtype in `__post_init__`, so a codec that reads only the annotations would make it float64. The golden reading is list<int64>.
6. **Lockstep timing (§5.5):**
   - tick i of an hz group runs at logical time `i / hz` (a float division, not repeated addition);
   - a rate-less group steps at the logical time its triggering message was sent, after every tick at that time, and its `now()` is that time;
   - `start(until=T)` runs the ticks with t < T. The suites avoid `until` values that land on a tick.

   `kpi/support/arm_cell_oracle.py` steps the reference cell by hand on exactly these rules, and A3 and the equivalence gate compare against it bit for bit.
7. **Placing an `lk.attach` proxy and an `lk.replay` source.** Neither fits §5.2's group rules as written:
   - a proxy has only `latest` inputs, so it cannot step in a rate-less group;
   - replay is a source, so it needs `hz`.

   The reading is that both are leaves placed like any other: the proxy in its own `hz=100` group, and replay in an hz group, emitting each message at the first tick ≥ its publish time (`tasks/arm_cell/attached_planner.py`, `kpi/twin`, `kpi/golden`).
8. **Facility types in C++ at `kMaxJoints = 16`.** A `Metrics` record needs about 10 standard names plus one per input port, so 16 slots cannot carry it. The core lane needs a larger capacity for facility types.
9. **Workflow-size targets (criterion 4) look out of reach on small tasks.** The frozen tasks measure as follows (each baseline carries the same typed `JointState` payload):

   | Task | like_keras solution | Baseline | Target |
   |---|---|---|---|
   | `kpi/tasks/relay` (cross-process) | 339 tokens | raw dora: 448 tokens (0.76×) | 0.4× |
   | `kpi/tasks/pipeline` (in-process) | 301 tokens | plain Python: 169 tokens (1.78×) | 1.2× |

   This is the API's fixed cost (System + Deployment + port declarations), not the build's. Either revisit the targets or add a larger task, where the fixed cost amortises.

## Readings unlikely to be contested (pinned in kpi/)

- **LKF (§7.5):**
  - buffers per node follow the Arrow C Data Interface: struct 1, primitive and bool 2, utf8 3, list and map 2; a node's children come after its own buffers, depth first;
  - an empty validity buffer (length 0) means no null slot;
  - `buffer lengths` are unpadded, and each buffer is padded with zeros to 8 bytes;
  - length 0 means no buffers (n = 0);
  - a null slot holds zeros; the children of a null struct slot are null where nullable, zero otherwise.

  pyarrow rebuilds every golden from its raw buffers and validates it in full (`kpi/golden/test_golden.py`).
- **Schema-hash preimage:**
  - nullable is spelled `1`/`0`;
  - a list's child is named `item`; a map's child is `entries` (`+s`, not nullable), with `key` and `value`;
  - map entries keep the mapping's iteration order;
  - the hash prints as `format(u64, '016x')` (`kpi/golden/schema_hashes.json` lists every preimage).
- **Goldens:** the file name is `kpi/golden/<domain>.<Type>.lkf`. `std/Header` has a golden of its own, although v0 only sends it nested.
- **MCAP (§8):**
  - a schema's `name` is the registry name, its `encoding` is `arrow-c-format`, and its `data` is the schema-hash preimage (a bare `+s` names no type);
  - `publish_time` is `header.stamp` in integer ns, with ±1 µs accepted;
  - topics are matched by their `.<port>` suffix;
  - a port that is both exposed and crossing a group may be recorded twice, but only with identical bytes;
  - replay channels are named `<child>__<port>`.
- **The §4.6 snippet:** it uses `cell.targets` and `cell.planner`, but the default names would be `target_file` and `pick_planner`. The reference passes `"targets"` and `"planner"` explicitly.
- **inproc lifecycle (§5.7):** when `activate` raises under `inproc`, `start()` raises in the launching process; it must not exit the launching process.
- **Eager calls:** `component(**inputs)` does not call `activate`. Tests that need it call it themselves.
- **`ArmControllerPy`** is a factory that returns an `lk.System`.
- **New-plan detection in Python:** a Python component can only spot a new `latest` value (C++ `take()`) by identity. `JtcPy` falls back to comparing values, in case a backend hands `forward` a fresh object for the same message.
- **`LK_PARAMS`** handles `std::string`, `bool`, `double`, `int` and nested `LK_PARAMS` aggregates. An empty params struct carries one dummy field.
- **The fake bus:**
  - **Board file:** layout is in `kpi/tasks/arm_cell/fake_bus.{hpp,py}`; the board watchdog is applied lazily when the file is read.
  - **`BUS_LOCK`:** an flock on a per-test path, never the rack's `~/.cache/rack-commissioning/bus.lock`.
  - **`exchange`:** takes `now()` explicitly.
  - **Holding:** before any plan, and while a plan waits for its start, the JTC holds at the measured position, latched once.
- **CLI output:** `lk top` prints plain text when stdout is not a TTY. `lk ls` prints each port's name, type, hash and policy, plus the dora version.
- **Consumer checks:**
  - **T4:** passes if any component's health shows `InputStale` during the stall.
  - **T5 (`target`):** "within 1 s" counts from the moment the restarted node's pid appears, which includes Python start-up. Tight; see the open question below.
  - **T7 under dora:** dora restarts the refused node, so the check is that no other pid ever sent a frame to the board.
- **Black-box rule 3:** blocks `dora`, `pyarrow` and `like_keras.backends.dora`. It does not block `inproc`, which §4.4's default argument makes part of the core.

## Open, for the dispatcher

- **T5 in `target`:** count the 1 s from the restarted process's start (as now) or from its first step? Python start-up alone is 0.3–1 s on this host.
- **Criterion 2 is partly unmeasured at M0.**
  - The C++-sender A/B waits for the dora cxx API, which the dora lane builds into `~/.cache/like_keras/dora-cxx-0.5.0/`.
  - Copies are counted on the receive side only.
  - like_keras's forwarder CPU is not measured yet.

  `kpi/bench_overhead.py` reports each of these as `null`, which scores 0. The ruler lane closes them once the dora lane lands.
- **Dora lane hint (measured):**
  - dora resolves a node's `path` through symlinks, and the venv's `python` is a symlink. A node started as `.venv/bin/python` loses the venv (no pyarrow). The raw baseline runs nodes through `/usr/bin/env`.
  - Nodes inherit the daemon's environment.
  - The daemon fails to come up from a working directory deeper than a unix socket path allows. kpi keeps it under `build/kpi/`.
