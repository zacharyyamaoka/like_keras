# Stable API over a churning backend: practice for like_keras

*Research lane, 2026-11-15. Primary sources, cited inline.*

**Bottom line**

- **Copy OpenTelemetry's split, not Keras's global switch**: a zero-dependency API package whose default is a working in-process eager backend, plus one package per backend (dora, fused C++), selected by name in the Deployment.
- **Keras leaks exactly at the execution loop** (its `train_step` is written per backend). The loop is what like_keras *is*, so it must specify scheduling semantics normatively and test every backend against them.
- **Delivery policy belongs to the wire.** A backend honours it or refuses at build, never degrading silently as ROS 2 intra-process and dora `backpressure` do.
- **The framework gives every component the same facilities** (logs, metrics, health, params, lifecycle, clock, recording, introspection), shaped after OTel, `diagnostic_msgs`, ROS 2 and Drake's Context.

## 1. Design rules

### A. A stable API over a churning backend

1. **Ship the API and each backend as independent packages; components import only the API**, because a library "cannot know" which implementation the app uses, and the API alone must run with "no-ops which generate minimal overhead" ([OTel library guidelines](https://opentelemetry.io/docs/specs/otel/library-guidelines/)).
2. **Version the API apart from the backends, and freeze it first**, because OTel requires "the API MUST become stable before the other components" and supports an old major for at least 3 years ([OTel versioning](https://opentelemetry.io/docs/specs/otel/versioning-and-stability/)). dora main already routes over a "direct zenoh path" that 0.5.0 lacks ([dora yaml spec](https://raw.githubusercontent.com/dora-rs/dora/main/docs/yaml-spec.md)).
3. **Fail fast at build; never throw from framework code at runtime**, because OTel "MUST NOT throw unhandled exceptions at runtime" ([OTel errors](https://opentelemetry.io/docs/specs/otel/error-handling/)). Drake's `DiagramBuilder::Build` throws "if the graph is not buildable" ([Drake](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_diagram_builder.html)). A control loop should lose a log line, never a tick.
4. **Keep the public surface closed and enumerable, the rest private**, because Keras promises portability only "as long as you only use ops from `keras.ops`" and declares `keras.src` breakable ([Keras 3](https://keras.io/keras_3/)).
5. **Label each backend's escape hatch rather than pretend it is portable**, because Keras's `train_step` is per backend and JAX needs `stateless_call` ([Keras 3](https://keras.io/keras_3/)). In like_keras, components never write the loop, and `backend.native(...)` is the labelled exit.
6. **Have backends declare capabilities as data; normalise syntax in the API**, because DB-API's `threadsafety` (0–3) is checkable but its five `paramstyle`s leaked driver syntax into user code ([PEP 249](https://peps.python.org/pep-0249/)) until SQLAlchemy normalised it with `text(":name")` ([SQLAlchemy](https://docs.sqlalchemy.org/en/20/core/sqlelement.html#sqlalchemy.sql.expression.text)).
7. **Discover backends through an entry-point group and choose them per Deployment, not per process**, because fsspec's `fsspec.specs` entry point plus a small overridable core works ([fsspec](https://filesystem-spec.readthedocs.io/en/latest/developer.html)), while process-global `KERAS_BACKEND`/`RMW_IMPLEMENTATION` cannot say "these two in one thread, that one over dora".
8. **Specify execution semantics normatively, and run one conformance suite against every backend**, because the Array API left "parallelism, scheduling, eager vs. delayed evaluation" out of scope ([Array API](https://data-apis.org/array-api/latest/purpose_and_scope.html)). Keras's contract is measurable: identical "up to `1e-7` precision in float32, per function execution", RNG excepted.
9. **Pin the backend version in the Deployment and check it on connect**, because interop breaks across versions: rmw_zenoh Humble-to-newer "communication ... will fail" ([rmw_zenoh](https://github.com/ros2/rmw_zenoh)), like mismatched dora daemons.

### B. The component boundary

10. **Make each component a black box with ports, wired from outside by a single-use builder**:
    - Hexagonal ports let an app be "driven by users, programs, automated test or batch scripts" and run "in a fully isolated mode" ([Cockburn](https://alistair.cockburn.us/hexagonal-architecture/)).
    - Drake wires from outside with `Connect`, `ExportInput` and `ExportOutput` ([Drake](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_diagram_builder.html)).
11. **Keep state, time and params in a framework-owned Context, so its step is a function**:
    - A Drake Context "never" points to its System, so it can be serialized and restored ([Drake caching](https://drake.mit.edu/doxygen_cxx/group__cache__design__notes.html)).
    - Functional core / imperative shell keeps I/O in the shell ([Bernhardt](https://www.destroyallsoftware.com/screencasts/catalog/functional-core-imperative-shell)).
    - robot_controllers' `tick(state, dt) -> (cmd, diag)` already has this shape.
12. **Declare each output's input dependencies and reject zero-delay cycles at build**, because Drake requires an acyclic graph; feedback needs an explicit unit delay, and the same declarations let the scheduler fuse a chain.

### C. Wiring and scheduling semantics

13. **Set the delivery policy on the wire when connecting**, as OROCOS does with `ConnPolicy::data()` (overwrite) versus `ConnPolicy::buffer(size, lock_policy)` ([OROCOS manual](https://www.orocos.org/book/export/html/1578)). A minimal closed set:

| Policy | Meaning | Prior art |
|---|---|---|
| `latest` | One slot, overwritten. For state. | OROCOS data; ROS `KEEP_LAST 1`; dora `queue_size: 1` (stale-slot bug: dora PR #652) |
| `queue(n)` | Drop oldest; count drops. | dora's default: `queue_size: 10`, `drop_oldest` |
| `reliable(n)` | Block the producer; never drop. | dora `backpressure`: blocks `send_output`, 64 MiB cap, leaves the zero-copy path |
| `gather` | One fresh sample from each input. | lk_dora's Gather list; ROS `message_filters` ExactTime |
| `timer(period, offset)` | Triggered by the clock. | `dora/timer/hz/30`; Lingua Franca `timer t(offset, period)` |

14. **Return freshness with every read**, because OROCOS `read()` returns `NewData`, `OldData` or `NoData`, and writing to an unconnected port is not an error, so a controller sees a stale trajectory without a side channel.
15. **Report the transport chosen for each wire**, because in dora, `backpressure` silently moves every consumer of that output off zero-copy ([dora spec](https://raw.githubusercontent.com/dora-rs/dora/main/docs/yaml-spec.md)).

### D. In-process behaviour must equal cross-process behaviour

16. **Run the same tests on the fused path and the IPC path**, because ROS 2 intra-process quietly diverged from inter-process ([ROS 2 IPC](https://design.ros2.org/articles/intraprocess_communications.html)):
    - no `transient_local`;
    - overwritten entries are lost where inter-process would have delivered them;
    - `keep_all` with reliable blocking "can't be exactly emulated";
    - a depth of 10 can hold 20 messages when both paths are live.

    The win is real: Mont Blanc latency fell from 1050 µs to 160 µs.
17. **Loan, write, then commit**: iceoryx2 does `loan_uninit()` → `write_payload(...)` → `send()` ([iceoryx2](https://raw.githubusercontent.com/eclipse-iceoryx/iceoryx2/main/examples/rust/publish_subscribe/publisher.rs)). **Convert to the wire type only when a message crosses a boundary**, as ROS 2 type adaptation keeps in-process data "without ever converting it" ([REP 2007](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2007.rst)).
18. **Fuse a controller chain into one thread, ordered read → chain → write**, as ros2_control does:
    - One loop "reads states from hardware, updates controllers and writes commands" at `update_rate`, under `SCHED_FIFO` 50. An `ERROR` from a controller deactivates "the entire controller chain" and switches to fallbacks ([controller_manager](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html)).
    - Chained controllers export reference interfaces and are activated following-first ([chaining](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/controller_chaining.html)).
    - Copper goes further: the graph is "wired at compile time" and the hot path is zero-allocation ([copper-rs](https://github.com/copper-project/copper-rs)).
19. **Use the Arrow C Data Interface as the payload ABI, with nanoarrow on the C++ side**:
    - The interface is "ABI-stable", zero-copy within a process, and needs no dependency on the Arrow project ([Arrow C Data](https://arrow.apache.org/docs/format/CDataInterface.html)).
    - nanoarrow is "two files" that compile to "a few hundred kilobytes" ([nanoarrow](https://arrow.apache.org/nanoarrow/latest/index.html)).
    - dora payloads are already Arrow; robot_msgs' struct-of-arrays maps onto a StructArray exposed via `__arrow_c_array__` ([PyCapsule](https://arrow.apache.org/docs/format/CDataInterface/PyCapsuleInterface.html)). Across processes, use Arrow IPC.

### E. Lifecycle and supervision

20. **Give every component one state machine, driven from outside**:
    - ROS 2 has `Unconfigured`, `Inactive`, `Active` and `Finalized`. `on_error` returns a node to `Unconfigured`, self-management is "discouraged", and a latched `lifecycle_state` topic announces each transition ([ROS 2 lifecycle](https://design.ros2.org/articles/node_lifecycle.html)).
    - OROCOS: `configureHook`/`startHook`/`updateHook`/`stopHook`/`cleanupHook`. Copper: `start`/`preprocess`/`process`/`postprocess`/`stop`, with non-time-critical work in `postprocess` ([cu29 CuTask](https://docs.rs/cu29-runtime/0.10.0/cu29_runtime/cutask/trait.CuTask.html)).
21. **Declare supervision as data on the System**:
    - Erlang offers `one_for_one`, `one_for_all` and `rest_for_one`. Restart intensity defaults to 1 restart in 5 s, children are `permanent`, `transient` or `temporary`, and they start in order and stop in reverse ([OTP](https://www.erlang.org/doc/system/sup_princ.html)).
    - dora's `restart_policy` (`max_restarts`, a doubling `restart_delay`, `restart_window`) is the same idea in YAML.
    - Robot-specific rule: a restarted controller comes back `Inactive`, so energising the hardware stays an explicit decision.

### F. Time, recording and replay

22. **Let components read time only from the context, and let the Deployment own the clock source**:
    - ROS 2 separates `SystemTime`, `SteadyTime` (for hardware timeouts; not comparable with the others) and `ROSTime` (which can follow simulation via `/clock`). A time of zero "should be considered an error", and backward jumps fire callbacks ([ROS 2 clock](https://design.ros2.org/articles/clock_and_time.html)).
    - Lingua Franca holds logical time fixed during a reaction; lag = physical − logical ([LF](https://www.lf-lang.org/docs/writing-reactors/time-and-timers)). Copper threads one `RobotClock` through hardware, sim and replay.
23. **Record at the wire in MCAP, keeping both timestamps**:
    - MCAP messages carry `log_time` and `publish_time` plus a schema per channel ([MCAP](https://mcap.dev/spec)).
    - `dora replay` turns on `backpressure` for every input so that replay never drops a message.
    - Exact replay needs deterministic steps. Copper replays "the same code with the same data in the same order".

## 2. Standard component facilities

| Facility | Best-known API shape | like_keras shape |
|---|---|---|
| Logs | stdlib `logging.getLogger(name)`; a library adds only a `NullHandler` ([howto](https://docs.python.org/3/howto/logging.html#configuring-logging-for-a-library)); export through the OTel Logs API ([spec](https://opentelemetry.io/docs/specs/otel/logs/api/)) | `ctx.log`, named by component path. In C++, a lock-free ring drained off the real-time thread. |
| Metrics | OTel `Meter` → `Counter`, `UpDownCounter`, `Histogram`, `Gauge`, `Observable*`; safe for concurrent use; an `Enabled` check ([metrics API](https://opentelemetry.io/docs/specs/otel/metrics/api/)) | Instruments created in `configure`. Free for every component: step count, step-time histogram, input age, drops per input, overruns. |
| Health | `DiagnosticStatus` `OK=0 WARN=1 ERROR=2 STALE=3` with `name`, `message`, `hardware_id`, `values` ([msg](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/diagnostic_msgs/msg/DiagnosticStatus.msg)); k8s liveness, readiness and startup probes ([k8s](https://kubernetes.io/docs/concepts/configuration/liveness-readiness-startup-probes/)) | `ctx.health.set(level, message, **values)`. The framework sets STALE after missed periods; ready means `Active`. |
| Params | ROS 2 `declare_parameter` with `ParameterDescriptor(read_only, ranges)`; pre-set, on-set (validate atomically, no side effects) and post-set callbacks ([rclcpp](https://docs.ros.org/en/iron/p/rclcpp/generated/program_listing_file_include_rclcpp_node_interfaces_node_parameters.hpp.html)); a parameter-events topic ([design](https://design.ros2.org/articles/ros_parameters.html)) | A frozen dataclass on the class, validated atomically. Changeable only while `Inactive` unless marked `live`. Each change is emitted on a port. |
| Lifecycle | ROS 2 managed node; OROCOS hooks; cu29 `start`/`process`/`stop` | `configure`, `activate`, `step`, `deactivate`, `cleanup`, `on_error`. State is published on a latched port. |
| Clock | ROS 2 `ROSTime` and `SteadyTime`; LF logical time; cu29 `RobotClock` | `ctx.now` is the logical time of the trigger; `ctx.steady` is for hardware timeouts only. |
| Recording | MCAP; Copper's single `.copper` log | Any wire can be tapped with `Deployment.record(...)`. Replay is a source with the same ports. |
| Introspection | Drake `GetGraphvizString`; the ROS 2 graph plus `lifecycle_state`; `dora list` | `system.describe()` returns JSON: ports, types, policies, placement and transport. The same JSON feeds Clank and the backends. |
| Supervision | OTP child spec; dora `restart_policy` | `System.supervise(strategy, intensity, period)`, compiled to each backend. |

## 3. The API shape these rules imply

```python
class PD(Component):                         # a black box: ports + params, no wiring inside
    class In(Ports):
        state: JointState
        ref: Trajectory                      # delivery policy is set on the wire, not here
    class Out(Ports):
        cmd: ServoCommand
    class Params(ParamSet):
        kp: float = 40.0
    def configure(self, ctx):
        self.err = ctx.metrics.histogram("track_err", unit="rad")
    def step(self, ctx, i: In) -> Out:       # functional core: no I/O, no wall clock
        if i.ref.status is OLD: ctx.health.set(WARN, "ref stale")
        ...

app = System("arm")                          # wiring happens outside the components
app.connect(planner.out.traj, pd.in_.ref, policy=reliable(4))
dep = (Deployment(app)
       .fuse([driver, pd], thread="rt", rate_hz=1000, backend="cpp")
       .process(planner, backend="dora"))
dep.check()   # capabilities, types, cycles, versions: every failure happens here
```
