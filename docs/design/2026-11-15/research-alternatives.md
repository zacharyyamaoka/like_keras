# Alternatives to ros2_control and ROS pub/sub: a survey

*2026-11-15 · like_keras re-scope · sources inline · vault priors: `C - External Composition`, `C - 5Cs model`.*

## Verdict

- **Every mature "black-box ports, wiring from outside" system keeps the wiring in a separate artifact that a deployer or system builder owns**, such as the OROCOS deployer script, Drake's `DiagramBuilder`, Holoscan's `compose()` or dora's YAML. Components in these systems never name their peers, while components in ROS and AimRT do.
- **"Chain in-process, network at the boundary" already exists twice.** Holoscan nests Operator → Fragment (one process) → Application (fragments joined over UCX). OROCOS uses lock-free wires inside a process and CORBA/MQueue between processes, with one `connect` call for both. Copy this two-level shape.
- **dora must be a backend, not the API.** dora 1.0 (2026-09-02) "does not interoperate with 0.x at all". We pin 0.5.0, and both our OBSBOT `dataflow.yml` and lk_dora's generator emit `_unstable_deploy`, which 1.0 rejects. dora's in-process unit, the operator, sits outside the 1.0 stability guarantee. So like_keras chains in-process itself and uses dora *nodes* only across processes.
- **Every product that runs at 500 Hz–1 kHz has one C++ thread that owns the bus** (Cheetah, quad A1, libfranka, ros2_control). Python sits above it behind a non-blocking seam and never runs inside the loop.

## Baseline: ros2_control

`controller_manager` runs `read() → update() → write()` in one RT thread at `update_rate`. Controllers and hardware are C++ pluginlib plugins in one process ([docs](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)). Controllers chain in-process through reference interfaces ([chaining](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/controller_chaining.html)), and slow ones can be marked `is_async: true` ([async](https://docs.ros.org/en/jazzy/p/controller_manager/doc/running_controllers_asynchronously.html)). There are no Python controllers. **Steal:** exclusive command-interface claims, so each joint has exactly one writer.

## A. Component frameworks with external wiring

**OROCOS RTT.** A C++ `TaskContext` has `InputPort<T>`/`OutputPort<T>`, and its logic runs in `updateHook()`. The deployer "will form connections between ports using a program script or XML file", which the manual calls "preferred" ([manual](https://docs.orocos.org/rtt/orocos-task-context.html)). Each wire gets a policy, `ConnPolicy::data()` or `buffer(N)`, and in-process wires are lock-free. Between processes, "CORBA transports are not hard real-time, while MQueue transports are". The deployer also assigns each component's `Activity` (thread, period, priority). Mature (since 2001), little recent activity. **Steal:** wire policy and rate are deployment parameters.

**RobMoSys / SmartSoft / SmartMDSD.** A service is a communication *pattern* plus a communication *object*. The pattern set is deliberately small and closed:

- Send, Query and Push (PushNewest/PushTimed) carry data.
- Event carries notifications.
- Parameter and Wiring (rewiring at runtime) handle configuration.
- State handles lifecycle and modes.

Sources: [patterns](https://robmosys.eu/wiki/general_principles:architectural_patterns:communication), [metamodel](https://robmosys.eu/wiki/modeling:metamodels:commpattern). Each role is separate: the service designer picks the pattern, the component supplier builds the box, and the system builder wires the instances and **chooses the middleware**. The tooling is Eclipse MDSD ([SmartMDSD](https://projects.eclipse.org/projects/modeling.smartmdsd)), and the EU project ran 2017–2020. **Steal:** the closed vocabulary, and the system builder choosing the middleware. That second rule is exactly the like_keras API/backend line.

**OpenRTM-aist.** Implements the OMG RTC standard. A component has data ports, IDL service ports and an *execution context* that drives its lifecycle ([guide](https://www.openrtm.org/openrtm/en/book/export/html/381)). It is wired from outside: you drag ports together in RT System Editor and save the system ([RTSE](https://openrtm.org/openrtm/en/node/6494)). It supports C++, Python and Java, runs on CORBA, and is at release 2.1.0. **Steal:** components that share one execution context run serially in one thread, so in-process chaining is a deployment choice.

**Drake systems.** A `LeafSystem` declares typed ports. `DiagramBuilder.Connect(out, in)` wires them, and `ExportInput`/`ExportOutput` make a `Diagram` that is itself a `System`, so diagrams nest ([tutorial](https://drake.mit.edu/tutorials/working_with_diagrams.html)). Python and C++ systems mix in one Diagram. Hardware sits behind `LcmInterfaceSystem` plus LCM subscriber/publisher systems, with the iiwa driver running as its own process. The same diagram runs sim or hardware by swapping driver systems ([LCM](https://drake.mit.edu/pydrake/pydrake.systems.lcm.html), [iiwa](https://drake.mit.edu/pydrake/pydrake.manipulation.html)). It makes no RT claim. **Steal:** the Python ergonomics, `builder.Connect(a.get_output_port(), b.get_input_port())`, almost verbatim.

**NVIDIA Holoscan/GXF.** An `Operator` has typed ports and `compute()`. `Fragment.add_flow(a, b, {("out","in")})` connects operators within one process. `Application.add_flow(frag_a, frag_b, {("op.out","op.in")})` connects fragments over UCX, and which fragment runs where is a launch argument ([distributed](https://docs.nvidia.com/holoscan/sdk-user-guide/using-the-sdk/create-a-distributed-application)). Python and C++ operators share one graph. Scheduler overhead is about 10 µs per iteration, or 17 µs with message passing ([perf](https://docs.nvidia.com/holoscan/archive/4.2.0/performance_considerations.html)). **Steal:** the same verb at both levels, plus per-flow latency tracking (`track_distributed()`).

**AimRT (AgiBot, C++20).** Each channel and RPC is an interface backed by a *backend* chosen in config: local, ros2, zenoh, iceoryx, grpc or mqtt. The local backend calls in-process directly ([concepts](https://en.docs.aimrt.org/tutorials/concepts/concepts.html)). Modules still name their own topics, as in ROS. **Steal:** choosing the backend per channel in config.

## B. Dataflow runtimes

**copper-rs.** Written in Rust. Tasks implement `CuSrcTask`, `CuTask` or `CuSinkTask`. The graph is declared in `copperconfig.ron` (`tasks`, `cnx: [(src, dst, msg)]`), and `#[copper_runtime]` compiles it into a deterministic static schedule with no hot-path allocation. Every message and the clock go into one `.copper` log, so a run replays exactly ([repo](https://github.com/copper-project/copper-rs), [cu29](https://docs.rs/crate/cu29/0.9.1)). A hexapod reported sub-µs task hand-offs ([Codethink](https://www.codethink.co.uk/articles/2026/copper-rs-real-time-robotics.html)). Python tasks exist, but the project's own docs call their performance "abysmal" ([wiki](https://github.com/copper-project/copper-rs/wiki/Python-support)). **Steal:** one replay log, and wiring checked at compile time.

**dora-rs.** Each node is a process, and YAML wires them (`inputs: {x: other/out}`). Each machine runs a daemon, and one coordinator directs the daemons. Messages of 4 KB or more go through Zenoh shared memory. It has C, C++, Rust and Python node APIs, and reports latency 10–17× lower than rclpy ([1.0 post](https://github.com/dora-rs/dora/blob/main/docs/blog/2026-09-02-dora-1.0.md)). Operators are still "experimental". **Steal:** wiring lives in a file the node never reads, and Arrow is the byte layout shared across languages.

**zenoh-flow** ([repo](https://github.com/eclipse-zenoh/zenoh-flow)) is small and still documented at v0.4. Skip it, since dora covers the same ground.

## C. Transports (not component models)

- **zenoh**: about 10 µs peer-to-peer latency and 67 Gbps on one machine ([arXiv 2303.09419](https://arxiv.org/abs/2303.09419)). It is also dora 1.0's data plane.
- **iceoryx2**: zero-copy, lock-free shared memory with pub/sub, request/response and events, plus C, C++ and Python bindings. Latency is about 100 ns in polling mode ([repo](https://github.com/eclipse-iceoryx/iceoryx2), [ROSCon 2024](https://roscon.ros.org/2024/talks/iceoryx2__A_Journey_to_Becoming_a_First-Class_RMW_Alternative.pdf)). It is the best same-host link between the C++ controller process and the Python planner.
- **LCM**: UDP multicast with `lcm-gen` types ([LCM](https://lcm-proj.github.io/lcm/)). Drake and Cheetah use it only at the edges.

## D. Robot real-time loops

- **MIT Cheetah**: one C++ process. `HardwareBridge` runs periodic tasks under `SCHED_FIFO` and `mlockall`: SPI every 2 ms (500 Hz), visualization at 60 Hz and RC at 200 Hz. A `RobotController` plugs into `RobotRunner`, and LCM carries only the gamepad, parameters and visualization ([HardwareBridge.cpp](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/robot/src/HardwareBridge.cpp)).
- **mjbots quad A1**: C++ on a Pi. Commanding and querying 12 moteus plus the IMU takes **740 µs** on one isolated core (`isolcpus`, `chrt 99`, `mlockall`), which leaves room for 1 kHz ([log](https://hackaday.io/project/167845/log/179913), [pi3hat FAQ](https://github.com/mjbots/pi3hat)). It is the closest analogue to our arm. Our 4-board exchange has a p95 of 9.3 ms, roughly 35× slower per servo (our one bus against their four). Measure Python against bus time before crediting C++.
- **libfranka/FCI**: in C++, `robot.control(callback)` runs at 1 kHz on PREEMPT_RT. `pylibfranka` gives Python an `ActiveControl` with `readOnce()`/`writeOnce()` ([docs](https://frankarobotics.github.io/libfranka/pylibfranka/latest/api/robot.html)).
- **ur_rtde**: C++ with pybind11 bindings. It uploads a URScript and streams `servoJ` over RTDE at 500 Hz on e-Series, so the interpolation runs on the robot ([docs](https://sdurobotics.gitlab.io/ur_rtde/)).
- **HEBI**: `Lookup` → `Group`. A feedback thread runs at `feedback_frequency`, and every handler must finish within one period. `send_command` is fire-and-forget with a `command_lifetime` ([API](https://files.hebi.us/docs/python/2.15.0/hebi/py_api.html)). **Steal:** setpoints that expire.

## E–F. Simulink/LabVIEW and 2024–2026 newcomers

- **Simulink/LabVIEW**: typed block ports, subsystems that are blocks, and code generated for RT targets. **Steal:** explicit rate-transition blocks wherever fast meets slow.
- **HORUS** (Rust/PyO3): claims 87–262 ns IPC. Created Oct 2025, about 120★ ([PyPI](https://pypi.org/project/horus-robotics/)). Unproven.
- **PeppyOS** (Rust/Python): preview only. I found no source for "peppygen".
- **rclpy EventsExecutor**: about 10× less CPU, same ROS wiring ([discourse](https://discourse.ros.org/t/faster-rclpy-executor-now-in-rolling/42852)).

## "What if dora changes?" It already did

| Fact | Source |
|---|---|
| Releases: 0.5.0 (2026-03-25), 1.0.0 (2026-09-02), 1.1.0 (2026-10-08) | [releases](https://github.com/dora-rs/dora/releases) |
| No 0.x interop: bincode → postcard, TCP → WebSocket, full restart required | 1.0 post |
| `_unstable_deploy` → `deploy`; `custom:` removed; unknown keys are parse errors | [migration](https://github.com/dora-rs/dora/blob/main/docs/migration-from-0.x.md) |
| C++ API: `init_dora_node_from_id`, `DataSampleHandle` and `node_id()` removed; operators need new callbacks or they fail to link | migration |
| Frozen for 1.x: node APIs, wire, CLI/YAML. Not frozen: operators, bridges, Arrow major, PyO3. Python floor raised to 3.11 | 1.0 post |

Our exposure: `obsbot_ptz_camera/dora/dataflow.yml` and `lk_dora/dora_backend.py:154` emit `_unstable_deploy`, and `native/cpp_scale` pins `dora-node-api-c =0.5.0`. Generated YAML migrates with one generator line; every hand-written YAML migrates on its own.

## Comparison

| System | Wiring authored by | In-process chain | Across processes | RT loop | Python↔C++ |
|---|---|---|---|---|---|
| ros2_control | YAML + topics in node | reference interfaces | ROS topics | one C++ thread | C++ only |
| OROCOS RTT | deployer script | lock-free ports | CORBA/MQueue | Activity per component | C++ |
| SmartSoft | system builder | tasks | pattern middleware | per task | C++ |
| OpenRTM | RT System Editor | shared exec. context | CORBA | exec. context | C++/Py/Java |
| Drake | `DiagramBuilder` | Diagram | LCM systems | none | one mixed graph |
| Holoscan | `compose()` | Fragment | UCX | ~10 µs scheduler | one mixed graph |
| AimRT | module + config | local backend | plugins | executors | C++, Py via protobuf |
| copper-rs | RON, compile time | static schedule | zenoh bridge | deterministic | Rust |
| dora-rs | YAML | operators (unstable) | shm/zenoh | none | C/C++/Rust/Py |
| Cheetah/quad/FCI | hard-coded | one process | LCM or none | 0.5–1 kHz FIFO | C++ (+Py bindings) |

## Closest to "black-box ports, wiring from outside, chain in-process, network at the boundary"

1. **Holoscan: the shape.** Operator → Fragment (in-process) → Application (network), with the same `add_flow` verb at both levels and mixed Python/C++ operators. Copy the API shape; the runtime is too heavy to adopt.
2. **OROCOS RTT: the semantics.** The deployer wires, each wire carries a policy, thread and rate are set at deployment, and the transport switches at the process boundary.
3. **Drake: the Python ergonomics and nesting.** `Connect` and `ExportInput`, a Diagram that is itself a System, mixed blocks, and hardware behind an edge transport.

## Ideas to steal for like_keras, ranked

1. One `connect` verb; deployment decides whether a wire is a direct call or goes through a transport.
2. A policy on every wire, `latest` or `queue(n)`. Controller wires default to `latest`.
3. Threads and rates are set at deployment, never in component code.
4. Setpoint ports carry a lifetime or deadline.
5. One replayable log of every port message plus the clock.
6. A backend per wire: direct call in-process, iceoryx2 on one host, dora or zenoh between hosts.
7. dora nodes only, behind the seam; never dora operators.
