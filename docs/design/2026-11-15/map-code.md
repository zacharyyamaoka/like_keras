# Donor-code map for a lean like_keras (2026-11-15)

Verdict: take **lk_dora_rs** as the spine (about 900 LOC of Python that already does graph, placement and compile), take its **black-box lesson** from `robot_controllers`/`substrate`, and treat everything else as reading material. The one thing that must change is the wire: JSON-in-UInt8 is a demo, and `robot_msgs` already has a lossless "JSON tree + named raw arrays" split that maps to Arrow buffers.

## 1. Inventory

| Path | LOC | What it is |
|---|---|---|
| `~/bam_ws/src/core/lk_dora_rs/src/lk_dora/graph.py` | 343 | `Block`, `Component`, `System`, `PortRef`. Eager-or-symbolic `__call__`. `System` evaluates its own graph eagerly. |
| `.../deployment.py` | 211 | `Deployment.process/replace/on_machine/validate`; `PythonNode`/`RustNode`/`CppNode` as frozen dataclasses. |
| `.../dora_backend.py` | 214 | Compiles a `Deployment` to a dora 0.5 descriptor (dict, YAML), `build_native`, `run`. |
| `.../runtime.py` | 109 | Generic Python dora node: it loads the factory from `LK_DORA_FACTORY`, queues inputs, and runs the placed block. |
| `.../codec.py` | 28 | `encode`/`decode`: JSON bytes in an Arrow UInt8 array. |
| `.../cli.py`, `__init__.py` | 67 | `lk-dora compile\|run module:callable`. |
| `lk_dora_rs/native/rust_scale/src/main.rs` | 80 | Rust node on `dora-node-api =0.5.0`. |
| `lk_dora_rs/native/cpp_scale/{main.cpp, dora_node_api.h, dora_bridge/src/lib.rs}` | 79 + 39 + 16 | C++ node over dora's **C** ABI, via a Rust staticlib shim (`liblk_dora_cpp_bridge.a`). |
| `lk_dora_rs/tests/*` (5 files), `examples/*` (6 files) | ~250 + ~250 | 17 passed, 1 opt-in native test (DATASHEET). Examples: components, component+system, two systems, fan-in, polyglot. |
| `lk_dora_rs/README.md`, `DATASHEET.md`, `Log.md` | 281 + 147 + 72 | Docs. |
| `~/bam_ws/src/core/like_keras/lk/archive/common/{component/py/component.py, port.py, system.py, graph.py, node.py, transport.py, transports/dora_backend.py, lifecycle/, logger/}` | 700, 348, 895, 270, 236, 202, 333, 310, ~400 | The old like_keras. `Component[ConfigT]` with a `wiring_mode()` contextvar, `InputPort`/`OutputPort`, a `Backend` ABC (`compile/launch/shutdown`, enum DORA_RS/ROS2/NATIVE), a lifecycle mixin, a `Logger` that dispatches to backends. About 3.7k LOC for the framework alone. |
| `.../like_keras/substrate/lk/{graph,compose,runtime,world,system}.py` | 156, -, -, -, 114 | The later eDSL. Wiring is YAML data. `Port(name="in.joint_cmd", spec={type, rate_hz, qos})`. Impls are `step(inputs, params, state) -> (outputs, state)` and never import `lk`. Black-box lint: wires may only touch exported ports. |
| `.../like_keras/design-notes.md`, `FRs.md` | 17, 27 | Almost empty. They say "scraping away the fancy stuff", and FR 1 is the homing steel thread (a plain Python loop that ticks the sequence controller). |
| `~/bam_ws/src/dev/bam_dora/bam_dora/{wrappers,launcher,clients}.py` | 208, 245, 165 | `DoraNodeWrapper` ABC, a `DoraLauncher` that wraps `subprocess` around `dora start`, and clients. `examples/nodes/ros2_control_bridge_node.py` (193) is **pseudo-code**: the rclpy parts are comments. |
| OBSBOT: `.../obsbot_ptz_camera/src/obsbot_ptz_camera/dora_node.py` | 685 | A dora-free `CameraService` that returns `(output_id, payload, metadata)` tuples. A 40-line dora shell (`run`, `_send`) sits around it. |
| OBSBOT: `src/obsbot_net/{gateway,protocol,client,cli}.py` | 482, 242, 220, 326 | A dora node that relays dora to a local unix socket or LAN TCP as JSON lines. Wire format defined once in `protocol.py`. |
| OBSBOT: `dora/dataflow.yml`, `keep_dataflow.sh`, `dora/bin/*` | 56, 76 | Dataflow with `restart_policy: always`, `_unstable_deploy.machine`, and a supervisor that never stops dora. |
| `/home/bam/robot_msgs/src/robot_msgs/_core.py` + `docs/DESIGN.md` | 380 + 456 | `Msg` base, `to_dict(arrays=...)`, `save_npz`, the type registry. |
| `/home/bam/robot_controllers/src/robot_controllers/{core,registry}.py` | 165, 160 | `Controller` Protocol, `Box(input_type, output_type)`, `ControllerSpec(target="module:Class")`. |

## 2. Keep and drop, with evidence

### Keep

1. **`forward` is plain sync Python, and calling it with `PortRef`s wires instead of running.** `test_graph.py::...wires_when_given_ports` and `...component_is_eager_for_values` prove it. The `graph.py` docstring states the rule. This is the Keras-like bit Zach wants. Keep the interface and shrink the implementation.
2. **Placement is a separate object.** `Deployment` never changes `forward`. `.replace(block, CppNode(...))` swaps the language and keeps the wire names. The Python → Rust → C++ → Python run gave `1→2→6` through `4→8→24` in 4 PIDs (`generated/polyglot.log`). This is the "same wires, different backend" proof.
3. **A `System` is a Component.** It nests behind one callable boundary, and `Deployment.process(system)` puts a whole system in one process. This is the in-process chaining Zach wants for controllers: ros2_control-style chains run in one process, and dora wires only the outside.
4. **List binding means Gather, with no implicit merge** (`runtime.py` queues per input and fires when every queue has an item; test `gather_waits_for_each_fresh_source`). The policy-free rule is correct. Make "latest-wins" an explicit component.
5. **A dora-free core with a thin dora shell** (OBSBOT `CameraService`). It returns `(id, payload, metadata)` tuples, the tests drive it with no dora, and `run()` is about 30 lines. Do the same for a like_keras component runner.
6. **WHY comments sit beside the transport workarounds** (the gateway, the CPU pinning, `restart_policy`). Keep them.
7. **Black boxes by (input type, output type).** `robot_controllers.Box` and `ControllerSpec.target = "module:Class"` give lazy import and swappable implementations. This is exactly what a C++ controller needs: a new `ControllerSpec` or `CppNode` with the same `Box`.
8. **`Backend` ABC with `compile/launch/shutdown` plus a NATIVE backend that runs in-process** (archive `transport.py`). The idea survives; the 3.7k LOC around it does not.

### Drop

| Item | Evidence |
|---|---|
| Pub/sub-style wiring inside the component (`bam_dora` `DoraNodeWrapper.handle_input(input_id, data)` plus the bridge's `send_output` calls) | Zach's correction: that is "wiring within a component". The `ros2_control_bridge_node.py` is also pseudo-code. |
| Dict-in-Arrow payloads (`pa.array([dict])`, `value[0].as_py()`) | Every bam_dora wrapper and the OBSBOT command path use it. It is Python-only, so C++ cannot read it. |
| JSON-in-UInt8 as the wire (`codec.py`) | The DATASHEET calls it "a demonstration contract". The Rust and C++ nodes parse integers out of the JSON by hand. |
| `Component[ConfigT]` with a contextvar `wiring_mode`, port auto-discovery from type hints, `InputPort`/`OutputPort` classes holding `_value` and `_connections` | The archive. 700 + 348 + 895 LOC. |
| Mixins, lifecycle, diagnostics, a `Logger` ABC | Archive, about 1.5k LOC. A lean library should have 3 standard ports and no framework. |
| YAML-as-source-of-truth wiring (`substrate/compose.py`) | Strong linting, but a different product. Python calls plus a compile step give the same graph with less machinery. |
| `LK_DORA_INPUT_PLAN` JSON env var that carries the graph shape into the node | It works but is brittle. Better to have each process rebuild the same factory and read the plan from the shared graph object. |
| Env-var configuration of native nodes (`LK_FACTOR`, `LK_MAX_RUNS`) | Fine for a demo. Pass a node config (JSON or Arrow) as one dora input or one file. |
| Ticks as `dora/timer/millis/N` only | Fine for sensors. A controller loop at 1 kHz needs its own thread and clock, with dora only on the edges (see section 3). |

## 3. dora 0.5.0 APIs as used, and the friction

### Python (`from dora import Node`)

| Call | Used in | Notes |
|---|---|---|
| `Node()` | everywhere | Reads the node id and daemon address from the environment that dora set. `Node("id")` makes a dynamic node. |
| `for event in node:` or `node.next(timeout=0.2)` | `runtime.py` / OBSBOT `run` | `event` is a dict: `type` is `INPUT`, `STOP`, `INPUT_CLOSED` or `ERROR`. For `INPUT` there are `id`, `value` (a pyarrow Array) and `metadata` (a dict). |
| `node.send_output(id, pa.array(...), metadata_dict)` | `_send` in `dora_node.py` | Python signature at v0.5.0: `send_output(self, output_id, data, metadata=None)`. |
| `node.drain()`, `node.is_empty()`, `recv_async(timeout)` | not used here | Exist in `apis/python/node/src/lib.rs` (checked at tag v0.5.0). |
| `node.dataflow_descriptor()`, `node_config()` | not used | Exist. They would let a node read its own YAML. |

Friction, with evidence:

- **`next(timeout)` expiring comes back as an `ERROR` event with "timed out" in the text**, not as `None`. OBSBOT `run()` matches `"timed out" in str(event.get("error"))`. The wrapper must hide this.
- **Event metadata contains dora's own `timestamp` as a `datetime`.** It broke `json.dumps` in the first gateway (`_plain()` in `gateway.py`).
- **A dynamic node that connects after a multi-machine dataflow has started never gets its Subscribe answered** (gateway docstring, `pending.rs @ v0.5.0`). The workaround is a normal node that relays to a socket. Treat the gateway pattern as the answer to "how does an outside program command the robot like an IP camera".
- **The shared daemon on core 22 stalled for 1.4 to 4.0 s under load.** OBSBOT moved its own CPU affinity away from it (`OBSBOT_CPUS`). Heavy work and control loops must stay out of the dora thread.
- **dora stops a node with a console-wide Ctrl+C on Windows**, and all machines must run the same version because daemons of other versions refuse each other. Both are arguments for like_keras owning the API.
- **Dora spawns a venv's `python` as the resolved base interpreter**, so `dora/bin/obsbot-node` is an `exec .../venv/bin/python -m ...` shim. `dora_backend.py` works around it by passing `PYTHONPATH` with `purelib`.
- **Provisioning is manual**: dora copies no environment and no binaries to other machines (README).

### Rust (`dora-node-api =0.5.0`)

`DoraNode::init_from_env() -> (node, events)`, `events.recv()`, `Event::{Input{id,metadata,data}, Stop(_)}`, `Vec::<u8>::try_from(&data)`, and `node.send_output(DataId, metadata.parameters, bytes.into_arrow())`. The `.parameters` clone propagates the metadata. Rust nodes have the full Arrow API.

### C++ — two surfaces; the repo uses the worse one

1. **C ABI (what lk_dora_rs uses).** The functions are `init_dora_context_from_env`, `dora_next_event` (**blocking, no timeout**), `read_dora_event_type`, `read_dora_input_id/data` (a `char*` and length of the raw buffer), `dora_send_output(ctx, id, id_len, data, len)`, `free_dora_event` and `free_dora_context`. It has **no metadata, no Arrow types and no shared-memory sample**. The Rust staticlib `lk_dora_force_link_c_api` shim exists only to link crates.io's `dora-node-api-c` into g++. `native/cpp_scale/build.sh` runs `cargo build --release` and then `g++ ... -ldl -lm -lrt -pthread`.
2. **The official cxx-bridge C++ API (`apis/c++/node/src/lib.rs` at tag v0.5.0, which I read; not built here).** It has `init_dora_node()`, `next_event`, `event_type` (with `AllInputsClosed`), `event_as_input`, `send_output`, `send_output_with_metadata`, **`send_arrow_output` and `event_as_arrow_input_with_info`** (Arrow C Data Interface pointers), a typed `Metadata` (`get_/set_` int, float, string, list, timestamp), and **`allocate_data_sample` plus `send_data_sample`** for zero-copy shared-memory output. It also has `empty_combined_events`, `dora_events_into_combined` and `CombinedEvents` for merging dora with another event source. A controller node needs this surface, because a moteus loop has to wait with a timeout on its own thread.

Lesson: a C++ controller node should consume one `dora input -> queue -> control thread` and never call `dora_next_event` on its 1 kHz thread.

## 4. How `robot_msgs` types cross a process boundary

What `robot_msgs` gives today:

- Storage is struct-of-arrays, one contiguous C-ordered little-endian array per field. `DESIGN.md` says "Arrow record batch, `Eigen::Map`, ndarray view in Rust, or a flatbuffer vector". Arrays are read-only numpy.
- `to_dict(msg, arrays=dict)` **already splits a message into a JSON tree plus named raw arrays** (`{"npz": "a0"}` markers; `save_npz` writes both). `from_dict(tree, arrays=...)` rebuilds it. `"_type": "control/ServoCommand"` names the registered type (`message_types()`).
- Header: `stamp` (float s), `frame_id`, `meta` (str to str). `ServoCommand` carries `state_stamp`/`state_seq`, so latency is measurable from a log.

Proposal for a wire envelope, one dora event per message (nothing here has been run):

| Part | Where | Content |
|---|---|---|
| Schema tree | dora metadata `lk_tree` (a string) or the first buffer | the `to_dict(arrays=...)` JSON (type name, scalars, names, array markers) |
| Buffers | one Arrow `StructArray` or `UnionArray`, one child per array, or one flat `UInt8` buffer plus an `offsets`/`dtype`/`shape` table in the tree | the same raw bytes numpy holds, with no copy on the Python side |
| Wire name | metadata `lk_type = "robot_msgs/1/control/ServoCommand"` | versioned, so C++ can refuse a mismatch |

Then:

- **Python** reads the buffers with `pa.Array.to_numpy(zero_copy_only=True)` and `Msg.from_dict`.
- **C++** reads a `std::span<const double>` from the `ArrowArray` of `event_as_arrow_input_with_info`. A fixed mapping per message type is cheaper than a generic parser, and `robot_msgs` says the field names and units are the semantics.
- **Fixed-schema alternative**: generate an Arrow schema per `Msg` class from `dataclasses.fields` and the type hints (`Array` means `FixedSizeList` or a flat `Float64`, `tuple[str,...]` means `List<String>`, `Header` means a nested struct). This is what a C++ `ServoCommand` consumer should use, because it needs no JSON on the 1 kHz path.
- A ROS 2 export (`robot_msgs.ros`) already exists, but it is for bags, not for the hot path.

Controller-relevant types and what they cost on the wire (from `joint_trajectory.py`, `servo_command.py`): `JointTrajectory` is `(N,J)` float64 arrays (`positions`, optional `velocities`, `accelerations`, `effort`) plus `t` and names. `ServoCommand` is `(J,)` arrays plus optional gains. A 500x6 trajectory is about 24 kB per array, so it is a single non-blocking message at planning rate, matching "send the trajectory via dora while the planner continues".

## 5. Reusable pieces, ranked for the new repo

1. **`graph.py` Block/Component/System/PortRef** (343 LOC). Reuse almost as is. Trim `System._evaluate`, the namedtuple-shaping and the duplicate `_output_map` code to about 220 LOC. Add typed ports (`Port[T]` from the `forward` annotations, which `robot_controllers.Box` already uses). Keep the single-use rule (`already wired`).
2. **`deployment.py` placement** (211 LOC). Reuse. Replace the `Process.every/max_runs` demo knobs with a `Component`-level `rate_hz` and an optional own-thread flag. Keep `.replace(...)` and `.on_machine(...)`.
3. **Backend interface.** Write a ~30-line protocol with `compile(deployment)`, `launch`, `shutdown`, taken from the archive's `Backend` ABC. `NativeBackend` runs a whole system in one process (the fall-back Zach wants); `DoraBackend` follows `dora_backend.py`. Put the dora spawn path (venv shim, `PYTHONPATH`, `restart_policy`) in this one file. That is where "dora changes" gets absorbed.
4. **Dora-free runner core + thin dora shell** (OBSBOT pattern). Same code path for the native and dora backends, so tests need no dora.
5. **Gateway pattern** (`obsbot_net/gateway.py`, 482 LOC + `protocol.py`). Becomes a generic "expose a system's ports on the network" component, which is the "plug in like an IP camera" API. Keep the cached latest-state-on-connect and the dropped slow client; drop JSON base64 images.
6. **Native node template**: Rust `main.rs` and a C++ file written against the **cxx API**, not the C ABI, with the Arrow/metadata calls and a worker thread. The Rust staticlib link trick is still needed in CMake or build.sh (cargo, then g++).
7. **`robot_msgs` envelope** (section 4) plus the `ControllerSpec` lazy `module:Class` target.
8. **Tests to port**: `test_graph.py` (78), `test_backend.py` (63, descriptor-level, no dora), `test_dora_e2e.py` (43, real dora with distinct PIDs), `test_native_e2e.py` (26, opt-in).
9. **Docs only, no code**: archive logger design (backend Protocol with `**context`), lifecycle states. Standard component ports (logs, health, config) come from here, but as 3 reserved ports rather than a mixin framework.

Open questions for the design (not answered by this code): how a component declares its own thread and loop rate (a controller loop is not a callback); how a single message gets a stable type name across Python, C++ and Rust; and whether `Deployment` lives in Python code or in a small file next to the system (the OBSBOT dataflow shows that a hand-written YAML is what operators actually keep running).
