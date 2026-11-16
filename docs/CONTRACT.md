# like_keras — build contract (v0, frozen 2026-11-15)

Zach, 2026-11-15: *"That is wiring within a component. that is ros pub/sub client/service architecture. we are moving more to robmosys black box ports and wiring from outside. So now its about the planning actually just deifning what its outputs are ... like keras kinda gives us that fleixbility to do the nice stable API we want and then underneath can churn more."*

This file is the seam the build lanes share. Change it only as a barrier (alone, before dependent lanes run). The pick and its reasons are in [design/2026-11-15/judging.md](design/2026-11-15/judging.md): base **P1 (thin facade)** with grafts G1–G14 from the runners-up. The ruler it is measured with is [design/2026-11-15/criteria.md](design/2026-11-15/criteria.md); §11 below is its build-loop form.

**What v0 is, in five lines.**
1. Components are black boxes: typed `In`/`Out` ports carrying `robot_msgs` types, a `forward`, params. They never name a peer, a transport, a thread or a host.
2. A `System` wires components from outside, in the subscribe direction (`s.add(planner, state=arm.state)`). A System is itself a component.
3. A `Deployment` places components on threads, processes and hosts. Inside a thread, edges are plain calls in a derived order. Across a thread, they are lock-free slots. Across a process, they are the backend's transport.
4. Backends v0: `inproc` (one process, many threads) and `dora` (dora-rs 0.5.0, nodes only), behind one conformance suite.
5. C++ components are written in C++ against one header; one `.so` runs as a dora node, as a GIL-free thread inside a Python process, or on a twin bus.

## 1. Layout and ownership

| Path | Owner | What |
|---|---|---|
| `docs/CONTRACT.md` (this file), `docs/design/` | contract (frozen) | the API, the semantics, the ruler's targets |
| `kpi/` (everything: `tasks/arm_cell/`, `conformance/`, `contract/`, `consumer/`, `golden/`, `miswire/`, `validgraphs/`, `polyglot/`, `facilities/`, `twin/`, `attach/`, `blackbox/`, `churn/`, scripts) | lane **ruler**; committed before any build lane starts; read-only to builders | the frozen ruler (§11) and the reference consumer (§10) |
| `/home/bam/robot_msgs`: `std/log_record.py`, `std/diagnostic_record.py`, `std/metrics.py`, public `robot_msgs.adopt` | lane **robot_msgs** (a barrier in another repo; lands alone, before M1) | the three facility message types and the zero-copy constructor (§2.3) |
| `src/like_keras/{__init__,component,system,deployment,codec,record}.py` | lane **core** | the Python API (§4), the semantics (§5), the codec (§7), recording (§8) |
| `src/like_keras/backends/inproc.py` | lane **core** | threads, slots, logical clock, `ctypes` native host (§9.1) |
| `cpp/include/like_keras/msgs/*.hpp` | lane **core** generates (`python -m like_keras.codec --cpp`); committed; CI checks the hash | C++ message views (§7.4). CMake never runs Python. |
| `cpp/include/like_keras/{lk.hpp,abi.h}`, `cpp/src/{rt,abi}.cpp`, `cmake/like_keras.cmake` | lane **cpp** | the C++ API (§6), the RT thread, the C ABI |
| `src/like_keras/backends/dora/{__init__,emit,host,gateway}.py`, `cpp/hosts/dora_host.cpp`, `cpp/hosts/build_dora_cxx.sh` | lane **dora** | the dora backend (§9.2) |
| `src/like_keras/{attach,cli}.py` | lane **dora** | attach protocol, `lk ls`, `lk top` (§9.3) |
| `docs/friction/NNN.md` | the **controllers** lane (the consumer) | friction found while porting; the ruler lane turns each into a new frozen test, never a change to a score |

The consumer's code lives outside this repo and outside `bam_rack_twin`: `~/bam_ws/src/core/mdp/agent/actuators/moteus_controller_node/` (the C++ chain and its Python peer `ArmControllerPy`). `arm_hold`, `pick_loop_v1` and `fivebar_run` are never edited (gate *fallback untouched*).

Commands (every lane, from any cwd; the rack is powered all day, so builds and benches run under `nice -n 19 ionice -c3`, never on core 22):

```bash
env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q kpi/<suite>
cmake -S /home/bam/like_keras -B /home/bam/like_keras/build -DCMAKE_BUILD_TYPE=Release
nice -n 19 cmake --build /home/bam/like_keras/build -j 8
```

## 2. Dependencies

### 2.1 Runtime (the lean-core gate)

- **Python ≥ 3.11.** The core imports only the standard library at import time. `robot_msgs` (path dependency `/home/bam/robot_msgs`, which brings numpy) is the one runtime dependency, imported lazily when a plan is built. After `import like_keras`, `sys.modules` holds no `robot_msgs`, `numpy`, `dora`, `pyarrow` or `yaml`; import takes ≤ 20 ms.
- **C++20**, g++ 13, CMake 3.28. `lk.hpp` and `librt` use the standard library only: no Arrow library (the Arrow C Data Interface is two plain C structs, declared in `abi.h`), no TOML or JSON library (params arrive as JSON and `LK_PARAMS` parses the subset it needs).

### 2.2 Extras and build-only

| Extra / tool | Pin | Used by | Never used by |
|---|---|---|---|
| `like_keras[dora]` | `dora-rs==0.5.0`, `pyarrow` (as resolved in `uv.lock`) | `backends/dora/` only | core, components, `inproc` |
| dora CLI | `dora-cli 0.5.0` on `PATH` | `backends/dora/emit.py` (`dora start/stop`), the skew check | everything else |
| dora cxx node API | git tag `v0.5.0`, crate path `apis/c++/node`, built once by `cpp/hosts/build_dora_cxx.sh` into `~/.cache/like_keras/dora-cxx-0.5.0/` (~0.7 GB, shared by all lanes; the root disk is 96 % full) | `cpp/hosts/dora_host.cpp` only | component `.so` files (they never link dora) |
| `like_keras[test]` | pytest, pyperf | `kpi/` | runtime |
| gtest 1.14, Google Benchmark 1.8.3 | system packages (installed) | `kpi/alloc_probe`, `kpi/bench_chain` | runtime |

`like_keras.backends.dora.VERSION = "0.5.0"` is the one place the dora version is written. User code never names it; `describe()` and `lk ls` print it.

### 2.3 Cross-repo barrier: what `robot_msgs` gains (lands before M1)

like_keras defines no message types. It needs three facility types and one constructor, added to `robot_msgs` by its own lane with golden tests there:

```python
# robot_msgs/std/log_record.py
@dataclass(frozen=True, eq=False, repr=False, kw_only=True)
class LogRecord(Msg):
    header: Header        # stamp = wall time of the record, epoch s
    path: str             # component path, e.g. "pick_cell.arm.jtc"
    tick: int             # the component's step count when it logged
    level: int            # stdlib logging level: 10 debug, 20 info, 30 warning, 40 error
    message: str

# robot_msgs/std/diagnostic_record.py — the field names of io-diagnostics' DiagnosticStatus v0 (contract/record.py), flat subset
@dataclass(frozen=True, eq=False, repr=False, kw_only=True)
class DiagnosticRecord(Msg):
    header: Header        # stamp = source time of the evaluation
    path: str
    level: str = "unknown"            # unknown | ok | warn | error. UNKNOWN is the zero value; STALE is never sent (§8)
    code: str = ""                    # PascalCase machine reason, required when level != "ok"
    message: str = ""
    mode: str = "initialization"      # initialization | operational | offline  (the lifecycle, §5.7)
    expected_period_s: float = 1.0
    uptime_s: float = 0.0
    stale_inputs: tuple[str, ...] = ()

# robot_msgs/std/metrics.py
@dataclass(frozen=True, eq=False, repr=False, kw_only=True)
class Metrics(Msg):
    header: Header
    path: str
    names: tuple[str, ...]
    values: Array         # float64, one per name
```

`robot_msgs.adopt(array: np.ndarray) -> np.ndarray` becomes public (today `_core.adopt`, "Internal use only"): it marks a read-only array as library-owned so `frozen_array` keeps it without a copy. like_keras calls it only on numpy views over received buffers whose schema hash matched (§7.3). Without it every Python receive copies.

## 3. The public surface (criterion 4)

| Scope | Names | Count |
|---|---|---|
| Top level (`like_keras.__all__`) | `Component`, `In`, `Out`, `System`, `Deployment`, `native`, `attach`, `replay` | 8 |
| Component | `forward`, `Params`, `params`, `activate`, `deactivate`, `now`, `log`, `metric`; implicit output ports `health`, `metrics` (`log` is both the logger and its port) | 10 |
| System | `add`, `wire`, `expose` | 3 |
| Deployment | `place`, `record`, `start`, `stop`, `describe` | 5 |
| Backends | `like_keras.backends.inproc`, `like_keras.backends.dora` | 2 |
| C++ only (no same-named Python symbol) | `get`, `take`, `write`, `LK_PARAMS`, `LK_EXPORT` | 5 |
| **Total** | | **33** |

The target is ≤ 30 total; v0 starts at 33 on purpose (§14, D4) and the ratchet may only lower it. Widest signature: `Deployment.place` with 6 parameters. No `**kwargs` option bags (the `**inputs` of `add`/`wire` are port names, checked against the component's declared inputs), no string enums.

Not public: the backend protocol, the C ABI (`abi.h`, for hosts), the attach wire format, the CLI.

## 4. Python API

```python
import like_keras as lk
```

### 4.1 Component, In, Out

```python
class Component:
    class Params: ...                      # default: an empty frozen dataclass. Override with @dataclass(frozen=True) class Params
    params: Params                         # set by __init__
    log: logging.Logger                    # named after the component path; every record also goes to the `log` port
    # implicit ports on every component, never declared: log (LogRecord), health (DiagnosticRecord), metrics (Metrics)

    def __init__(self, **params) -> None   # final: self.params = type(self).Params(**params). Subclasses never override it.
    def forward(self, **inputs) -> Any     # required on leaves. Argument names must equal the In names (checked at class creation).
    def activate(self) -> None             # optional. Create state, open devices. Raise to refuse (the process exits non-zero, §5.7).
    def deactivate(self) -> None           # optional. Release devices. Must not raise.
    def now(self) -> float                 # this step's scheduled time, seconds (epoch in real time; logical under lockstep)
    def metric(self, name: str, value: float) -> None   # a custom metric, published on `metrics` at 1 Hz
    def __call__(self, **inputs) -> Any    # eager: runs forward on plain values, now() = 0.0, no backend, no thread

class In(Generic[T]):
    def __init__(self, type: type[T], *, queue: int | None = None, stale: float | None = None, lagged: bool = False) -> None

class Out(Generic[T]):
    def __init__(self, type: type[T]) -> None
```

- Ports are class attributes: `state = lk.In(ServoState, stale=0.1)`. On an instance, `planner.state` is a port reference used only for wiring. In and Out names share one namespace per component; a clash is refused at class creation, as is any name that collides with a Component member.
- `In` options (the semantics are §5):
  - `queue=None` is **latest**: forward sees the newest value, sampled, never waited on. `queue=n` (n ≥ 1) is a **queue** of n, drop-oldest, drops counted; queue inputs trigger the component (§5.2).
  - `stale=s`: a latest value that arrived more than `s` seconds ago (on the receiver's clock) is delivered as `None` and the input is listed in `health.stale_inputs` with level `warn`, code `InputStale`.
  - `lagged=True`: the input reads what its writer produced in the previous step of the same group. A lagged input is a fact about the box (the moteus bus sends this tick's command in the next exchange), and it is what makes a same-thread cycle legal.
- `forward` returns: one `Out` → the bare value; several → a tuple in declaration order. `None` (or a `None` element) means not sent.
- A component placed outside the launching process is rebuilt there as `module:qualname(**params)`, so params must round-trip through JSON (`dataclasses.asdict` and back). Closures and lambdas cannot cross a process; `describe()` refuses them.

### 4.2 System

```python
class System(Component):
    def __init__(self, name: str) -> None
    def add(self, component: C, name: str | None = None, **inputs: PortRef) -> C
    def wire(self, component: Component, **inputs: PortRef) -> None
    def expose(self, *ports: PortRef, **renamed: PortRef) -> None
```

- **Wiring is subscribe-direction kwargs** (Zach's settled form, vault Apple Note *A start*: "only define the inputs for components ... like subscribe not publish"). `s.add(PickPlanner(), target=targets.target, state=arm.state)` adds the child and binds its inputs. `s.wire(arm, trajectory=planner.trajectory)` binds inputs of a child added earlier; it exists for cycles and forward references. Fan-out is several children naming the same output.
- Each input has exactly one writer (an `Out` of a sibling, or an input this System exposes). Fan-in onto one input is refused; a merge is an explicit component.
- `name` defaults to the class name in snake_case; a duplicate needs an explicit name. Children and exposed ports are attributes of the System (`cell.arm`, `cell.state`); a clash is refused.
- `expose(arm.state, command=guard.safe)` makes boundary ports: an exposed `Out` is readable from outside; an exposed `In` is writable from outside (a parent System, a Deployment wire, an attach client). The exposed ports are the System's ports when it is added to a parent ([closure under composition](design/2026-11-15/map-vault.md)). Only exposed ports appear in `describe()`'s datasheet and on the attach socket.
- A System has no `forward`. Its children run in the order the build derives (§5.3).

### 4.3 Checks before run (criterion 12)

`Deployment.describe()` and `Deployment.start()` run every check and raise one `ValueError` listing all failures. Each failure names `system.child.port`, what was expected, the fix, and the `file:line` of the `add`/`wire`/`expose`/`place` call that caused it (captured with `inspect` at call time). The checks:

1. Unknown port name; an `In` bound twice; an input left unbound (unless exposed); an `Out`→`In` type mismatch; a schema-hash mismatch against a native `.so`'s `lk_describe()`.
2. A same-group cycle with no lagged input; `lagged=True` on an edge that crosses a group (already delayed); a cycle in a rate-less group.
3. Placement: when any `place()` call exists, every leaf placed exactly once; a group mixing Python and native components; a native component and a Python component in one process under `dora`; a source (no inputs) in a group without `hz`; a component in a rate-less group that can never step (§5.2); a process name outside `[a-zA-Z0-9_.-]+` or equal to `dora`.
4. Realtime: `fifo` outside 1–99; `cpus` containing a reserved core (`RESERVED_CPUS = {22}`, where the shared dora daemon has stalled for up to 4 s); `cpus` beyond the machine.
5. Backend capability: a policy, `host=`, or rate the backend cannot honour (a backend declares its capabilities as data and refuses; it never degrades silently); a non-`robot_msgs` type on an edge that crosses a process or a language (any Python object may travel an in-process Python edge).
6. Attach: an input the deployment's top-level System exposes that is also wired inside the deployment (attach clients would make it two writers). Inputs exposed by a nested System are its boundary, wired by its parent, and are not affected.

`kpi/miswire` holds ~20 frozen mis-wirings that must each be caught; `kpi/validgraphs` must all pass (any false positive scores criterion 12 at 0).

### 4.4 Deployment

```python
class Deployment:
    def __init__(self, system: System, backend: ModuleType = like_keras.backends.inproc, realtime: bool = True) -> None
    def place(self, *components: Component, process: str = "main", host: str | None = None,
              hz: float | None = None, cpus: Sequence[int] | None = None, fifo: int | None = None) -> Deployment
    def record(self, path: str | os.PathLike, *ports: PortRef) -> Deployment
    def start(self, until: float | None = None) -> Deployment
    def stop(self) -> None
    def describe(self) -> dict[str, Any]
```

- `place(...)`: each call is one **group**: one thread running its components in the derived order (§5.1). Each distinct `process` is one OS process; `host` names a dora machine id (refused by `inproc`). With no `place()` call at all, every leaf runs in one group in process `main`. A System passed to `place` places all its leaves; a `native` component is one leaf.
- Under `inproc`, every process collapses into the launching process, and every group stays its own thread with the same slot semantics. So moving a component between processes is one edit of `process=` or of `backend=`, never both.
- `fifo` and `cpus` apply to native groups (the C++ RT thread). On a Python group they are not applied, and `describe()` lists them under `ignored` (a GIL-holding thread at SCHED_FIFO inverts priorities). This keeps the Python↔C++ swap one line.
- `record(path, *ports)`: no ports means every edge that crosses a group plus every exposed port. Writes MCAP (§8).
- `start()`: with `realtime=True`, launches and returns at once (non-blocking); `until=t` runs t seconds and stops (blocking). With `realtime=False` (lockstep, §5.5), `until` is required and the call returns when logical time reaches it, at full speed, with no sleeps.
- `stop()`: deactivates (reverse order), joins threads, stops processes (`dora stop` under `dora`), closes the recorder. Idempotent.
- `describe()`: runs §4.3, then returns the plan, the device datasheet (§8), as JSON-serialisable data:

```json
{"like_keras": "0.1.0", "abi": 1, "hash": "sha256 of this document without the hash",
 "backend": {"name": "dora", "version": "0.5.0", "capabilities": ["latest", "queue", "lagged", "host", "lockstep"]},
 "system": "pick_cell",
 "components": {"pick_cell.planner": {"impl": "bam_pick.planner:PickPlanner", "language": "python", "params": {"lead_s": 0.05},
                "ports": {"state": {"dir": "in", "type": "control/ServoState", "hash": "c41d9e07a2f3b815", "queue": null, "stale": 0.1, "lagged": false}}}},
 "edges": [{"src": "pick_cell.arm.state", "dst": "pick_cell.planner.state", "crosses": "process", "transport": "dora", "at": "cell.py:14"}],
 "groups": [{"process": "arm-controller", "host": null, "language": "native", "hz": 100, "cpus": [20], "fifo": 80, "order": ["pick_cell.arm"]}],
 "exposed": {"state": "pick_cell.arm.state"},
 "attach": {"socket": "unix:/run/user/1000/like_keras/pick_cell.sock"},
 "ignored": []}
```

### 4.5 `native`, `attach`, `replay`

```python
def native(path: str | os.PathLike, config: str | os.PathLike | Mapping[str, Any] | None = None) -> Component
def attach(address: str, lease_s: float = 0.25) -> Component
def replay(path: str | os.PathLike) -> Component
```

- `native`: reads the `.so`'s `lk_describe()` (no construction, no I/O) and returns a component whose ports are the exposed ports of the C++ System. `config` is a TOML path (stdlib `tomllib`) or a mapping; it is passed as JSON to `lk_create` at start. A wrong ABI version or a schema hash that differs from the Python side is refused with both values.
- `attach`: a component whose ports mirror a running deployment's exposed ports (an exposed `In` becomes an `In` you wire into, an exposed `Out` an `Out` you subscribe from). `address` is explicit (`unix:<path>` in v0), never discovered. Writing takes the deployment's write lease (§9.3); a refused write is logged at error level on the proxy's `log` port, and its `health` goes `error` with code `LeaseHeld` naming the holder.
- `replay`: a source whose `Out`s are the recorded channels: the exposed name when the channel was an exposed port, else `<child>__<port>`. Under lockstep it emits each message at its recorded publish time.

### 4.6 The reference consumer, in this API

The full frozen version is `kpi/tasks/arm_cell/`. File 1, the planner, declares its outputs and nothing else:

```python
import dataclasses
import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.trajectory import JointTrajectory
from bam_pick.plan import PickTarget, plan_pick

class PickPlanner(lk.Component):
    target = lk.In(PickTarget, queue=8)        # forward runs once per target
    state = lk.In(ServoState, stale=0.1)       # latest; None if older than 0.1 s
    trajectory = lk.Out(JointTrajectory)

    @dataclasses.dataclass(frozen=True)
    class Params:
        lead_s: float = 0.05

    def forward(self, target, state):
        if state is None:
            self.log.warning("no fresh arm state; target skipped")
            return None
        plan = plan_pick(target, start=state.joint_state())          # seconds of numpy
        return plan.starting_at(self.now() + self.params.lead_s)     # absolute start; returning is sending
```

File 3, wiring and placement, owned by the system builder:

```python
import signal
import like_keras as lk
from like_keras.backends import dora
from bam_pick.planner import PickPlanner
from bam_pick.targets import TargetFile

CONFIG = "records/bam-fb-arm/arm_controller.toml"
arm = lk.native("build/libarm_controller.so", config=CONFIG)
# arm = ArmControllerPy(config=CONFIG)                  # all-Python fallback: this one line

def pick_cell(arm: lk.Component) -> lk.System:
    s = lk.System("pick_cell")
    targets = s.add(TargetFile(path="targets.jsonl"))
    s.add(arm, "arm")
    planner = s.add(PickPlanner(), target=targets.target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    s.expose(arm.state, arm.command)
    return s

cell = pick_cell(arm)
dep = lk.Deployment(cell, backend=dora)                  # same_box: delete ", backend=dora"
dep.place(cell.arm, process="arm-controller", hz=100, cpus=[20], fifo=80)
dep.place(cell.targets, hz=2)
dep.place(cell.planner)                                  # rate-less: steps once per target
dep.start()
try:
    signal.pause()
finally:
    dep.stop()
```

File 2, the C++ controller, is §6.2.

## 5. Execution semantics (normative; `kpi/conformance` runs every rule on every backend)

### 5.1 Groups, threads, languages

- A group is one thread. Components in a group run as plain calls in the derived order; an edge inside a group is a variable (Python) or a pointer (C++), with no copy and no message.
- A group holds one language. A native group is the C++ RT thread of one `.so` (its whole `lk::System`). Python never runs on a native group's thread, and the native thread never calls Python or takes the GIL.
- An edge between groups is a slot: latest → a triple buffer (one atomic exchange, no lock, no wait on either side); queue → a bounded SPSC ring per edge. Retired messages go back on a graveyard ring and are released on a non-RT thread.

### 5.2 When a component steps

- **Group with `hz`**: every tick, in derived order, a component steps if it has no queue inputs, or if every one of its queue inputs holds a message (one is popped from each: *gather*). Sources (no inputs) live only here.
- **Group without `hz`** (rate-less): a step starts when a message arrives on a queue input of the group. In derived order, a component steps if every queue input holds a message, or, having no queue inputs, if a same-group upstream stepped earlier in this step. A component that can never step is refused (§4.3).
- `latest` never triggers. An overrun skips the missed ticks (counted in `metrics.overruns`) and never catches up with a burst.
- Pacing is absolute (`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` in C++; a deadline loop in Python), from the group's start.

### 5.3 Order

The build topologically sorts each group over its same-group edges, ignoring lagged inputs. A same-group cycle without a lagged input is an error; nobody writes an order by hand. For the controller chain, `bus.command` being lagged derives `bus → jtc → guard`.

### 5.4 Delivery across groups and processes

| Policy | Producer side | Consumer side | Drops |
|---|---|---|---|
| latest | overwrites the slot; never blocks | reads the newest at the start of its step; `stale=` turns old into `None` | not counted (superseded) |
| queue(n) | appends; never blocks | pops per §5.2 | drop-oldest beyond n, `metrics.drops.<port>` |
| lagged | same group only | previous step's value | — |

- A value written at time t on an edge between groups is visible to the receiver at its first step strictly after t.
- Producers never block on a consumer, a transport or a daemon. Every process sends through a per-process TX thread with bounded queues (latest outputs keep only the newest unsent value); no group thread ever calls into a transport.
- A **retracted** input (§9.3, a write lease that lapsed) reads as `None` / `nullptr` until its next value; a retracted queue is emptied.

### 5.5 Lockstep (`realtime=False`; the equivalence gate runs here)

- One logical clock; time starts at 0.0 and every hz group ticks at its period on it. Ticks at equal times run in `place()` order.
- The visibility rule of §5.4 holds exactly: a value sent at logical time t is seen at the receiver's first step strictly after t, never earlier and never later. Every backend implements it:
  - `inproc`: one thread steps every group in time order.
  - `dora`: each boundary output sends exactly one frame per sender step, a value or an empty marker, tagged with the sender's step time; a process starts its step at t only once it holds, on every boundary input, the frame of the upstream's last step before t (a conservative, deadlock-free schedule because every crossing edge is delayed). Per-input FIFO delivery is asserted by `kpi/conformance`.
- Ten seeded runs give identical output-stream hashes (criterion 10); `arm_cell` on the twin runs at ≥ 10× real time.

### 5.6 Clocks

- `now()` (Python, C++) is the step's **scheduled** time in seconds: epoch time in real time (the monotonic deadline mapped to `CLOCK_REALTIME` by an offset sampled at start), logical time under lockstep. Trajectories carry their absolute start in `header.stamp` (epoch s, the `robot_msgs` convention), so a late delivery never shifts a plan.
- Ticks are logged as int64 counts; `LogRecord.tick`.

### 5.7 Lifecycle and errors

- States: `initialization` (constructed) → `operational` (after `activate`) → `offline` (after `deactivate` or an error). Published as `health.mode`.
- `start()` activates each group's components in derived order before its first step; `stop()` deactivates in reverse. A restarted process comes back through `activate` (for the bus: `take_over`, never a bare stop).
- `activate` raising: the group never starts, the process exits non-zero having sent no data frame (T7). An exception in `forward`: that component's group deactivates, `health` goes `error` with code `Exception` and the message, and the process keeps serving `health`, `metrics` and attach so `lk top` can see why.
- Lifecycle commands from outside go through the Deployment and the attach socket (`activate`, `deactivate`); there is no lifecycle data port in v0.

## 6. C++ API

### 6.1 `like_keras/lk.hpp`

```cpp
namespace lk {
inline constexpr int kMaxJoints = 16;

struct InOpts { int queue = 0; double stale = 0.0; bool lagged = false; };   // same meaning as Python In(...)

class Component {
 public:
  virtual ~Component() = default;
  virtual void activate() {}      // non-RT thread; may block and open devices; throw to refuse
  virtual void forward() = 0;     // RT thread: no allocation, no lock, no blocking call except device I/O this component owns
  virtual void deactivate() {}    // non-RT thread; must not throw
  double now() const;             // this step's scheduled time, seconds (§5.6)
  void metric(const char* name, double value);   // name: a string literal; ≤ 32 per component; no allocation
  struct Log {                    // fmt: a string literal; two numbers; copied into a lock-free ring, formatted off the RT thread
    void info(const char* fmt, double a = 0, double b = 0);
    void warning(const char* fmt, double a = 0, double b = 0);
    void error(const char* fmt, double a = 0, double b = 0);
  } log;
};

template <class T> class In {
 public:
  In(Component* owner, const char* name, InOpts opts = {});
  const T* get() const;   // the current value: latest → newest (nullptr if none, stale or retracted); queue → this step's message
  const T* take();        // only what arrived since the last take(), else nullptr (latest: the newest if new; queue: pops one)
};

template <class T> class Out {
 public:
  Out(Component* owner, const char* name);
  T& write();             // this step's preallocated value; calling write() marks it sent this step
};

class System : public Component {
 public:
  explicit System(std::string_view name);
  template <class T, class... Args> T& add(std::string_view name, Args&&... args);   // allocation happens here, never later
  template <class T> void wire(In<T>& dst, Out<T>& src);    // dst ← src (subscribe direction); a swapped or mistyped wire is a compile error
  template <class... Ports> void expose(Ports&... ports);   // boundary name = port name
  template <class Port> void expose(std::string_view as, Port& port);
  void forward() final;   // the children, in the derived order (§5.3)
};
}  // namespace lk

#define LK_PARAMS(Type, ...)   // reflects an aggregate's fields: JSON → Type; an unknown or mistyped key throws naming it (at lk_create, never on the RT thread)
#define LK_EXPORT(factory)     // factory: lk::System factory(const Params&) with LK_PARAMS(Params, ...); emits the C ABI of §6.3
```

Rules a component author follows, checked by `kpi/alloc_probe` and `kpi/blackbox`:
- A component header includes only `like_keras/lk.hpp`, `like_keras/msgs/*.hpp` and its own headers. It never includes dora, never reads the environment, never names a peer.
- All allocation happens in the constructor or `activate`. `forward` touches only preallocated storage.
- A C++ component is ≤ 2 files and its non-body lines are the ports, `LK_PARAMS` and one `lk_component(...)` CMake line (criterion 8 part 5: ≤ 10).

### 6.2 The reference controller (file 2)

```cpp
#include <like_keras/lk.hpp>
#include <like_keras/msgs/control.hpp>      // generated, committed: rm::ServoState, rm::ServoCommand (fixed capacity)
#include <like_keras/msgs/trajectory.hpp>   // rm::JointTrajectory (read-only view)
#include "moteus_bus.hpp"                   // mjbots lib/cpp over fdcanusb; the twin bus by params
#include "jtc_core.hpp"
#include "guard.hpp"

struct MoteusBus : lk::Component {
  struct Params { std::string adapter = "0607623A"; bool twin = false; double watchdog_s = 0.25; };
  lk::In<rm::ServoCommand> command{this, "command", {.lagged = true}};   // goes out in the next exchange
  lk::Out<rm::ServoState>  state{this, "state"};
  explicit MoteusBus(const Params& p) : p_(p) {}
  void activate() override { bus_.open_exclusive(p_); bus_.take_over(); }   // R1, R2: BUS_LOCK, then a hold in the same tick
  void forward() override { bus_.exchange(command.get(), state.write()); } // one query+command frame, ~3.5 ms
  void deactivate() override { bus_.stop_feeding(); }                      // boards fall into their timeout mode
  Params p_; MoteusBusCore bus_;
};
LK_PARAMS(MoteusBus::Params, adapter, twin, watchdog_s)

struct Jtc : lk::Component {
  lk::In<rm::JointTrajectory> trajectory{this, "trajectory"};   // latest
  lk::In<rm::ServoState>      state{this, "state"};
  lk::Out<rm::ServoCommand>   command{this, "command"};
  void activate() override { core_.reserve(lk::kMaxJoints, 4096); }
  void forward() override {
    if (auto* t = trajectory.take()) core_.replace(*t, *state.get());   // switches at t->header.stamp
    if (!trajectory.get()) core_.hold_here(*state.get());               // retracted (lease lapsed): hold where it is (R5)
    core_.sample(now(), *state.get(), command.write());                 // past the end: hold the last sample, qd = 0
  }
  JtcCore core_;
};

struct ArmConfig { MoteusBus::Params bus; Guard::Params guard; };
LK_PARAMS(ArmConfig, bus, guard)

lk::System arm_controller(const ArmConfig& c) {
  lk::System s{"arm_controller"};
  auto& bus   = s.add<MoteusBus>("bus", c.bus);
  auto& jtc   = s.add<Jtc>("jtc");
  auto& guard = s.add<Guard>("guard", c.guard);   // fault, runaway → hold; overspeed logged only; PD + FF only
  s.wire(jtc.state, bus.state);
  s.wire(guard.state, bus.state);
  s.wire(guard.proposed, jtc.command);
  s.wire(bus.command, guard.safe);                 // a legal cycle: bus.command is lagged
  s.expose(jtc.trajectory, bus.state);
  s.expose("command", guard.safe);
  return s;                                        // derived order: bus → jtc → guard
}
LK_EXPORT(arm_controller)
```

### 6.3 The C ABI (`like_keras/abi.h`, `LK_ABI_VERSION 1`; for hosts, not for component authors)

```c
struct ArrowArray;                         /* Arrow C Data Interface, declared in abi.h */
typedef struct lk_box lk_box;
typedef struct { double hz; const int* cpus; int ncpus; int fifo; int lock_memory; int stepped; } lk_place;

uint32_t    lk_abi_version(void);
const char* lk_describe(void);                                               /* static JSON: system, exposed ports (name, dir, type, schema hash, opts), params schema, like_keras version */
lk_box*     lk_create(const char* params_json, char* err, size_t err_len);   /* construct; no I/O; NULL + err on failure */
int         lk_start(lk_box*, const lk_place*, char* err, size_t err_len);   /* activate (may block), then spawn the RT thread unless stepped */
int         lk_step(lk_box*, double now);                                    /* stepped (lockstep) only: one step on the caller's thread */
int         lk_push(lk_box*, uint32_t in_port, struct ArrowArray* msg);      /* never blocks; takes ownership; msg->release runs later on a non-RT thread */
int         lk_take(lk_box*, uint32_t out_port, struct ArrowArray* out);     /* never blocks; 1 = a new value moved into *out (caller releases), 0 = none */
int         lk_wait(lk_box*, int timeout_ms);                                /* I/O threads only: block until an exposed Out has news, or timeout */
void        lk_stop(lk_box*);                                                /* stop the RT thread, deactivate in reverse order */
void        lk_destroy(lk_box*);
```

- `lk_push`, `lk_take` and `lk_step` also release retired inputs on the calling thread before they return. In-process (`inproc`), the Python side calls them through **`ctypes.PyDLL`**, holding the GIL: they never block, and `CDLL`'s release-and-re-take of the GIL could wait a full 5 ms switch interval behind a spinning planner. `lk_create`, `lk_start`, `lk_wait` and `lk_stop` go through `ctypes.CDLL` (GIL released).
- In-process pushes are zero-copy: the `ArrowArray` points at the frozen numpy buffers of the `robot_msgs` message and its `release` drops the reference, which happens on the Python thread at the next call.
- `lock_memory=1` (`mlockall`) is set only for a dedicated controller process (`lk-dora-host`). Inside a Python process (`inproc`), `lk_start` pre-faults the RT thread's stack and pools instead.
- The RT thread signals new exposed outputs with one non-blocking `eventfd` write per step that published; that wake is counted in every overhead number (§11).

### 6.4 The dora host (`lk-dora-host <plan.json> <process>`)

Built once against the dora **cxx** node API 0.5.0. It is the only C++ that links dora. Component `.so` files never do.

- **Allowed dora calls, and no others:** `init_dora_node`, `next_event`, `event_type`, `event_as_arrow_input`, `send_arrow_output`. **Never** the C node API (`dora_next_event` and `dora_send_output` alias one `&mut DoraContext` with no lock, and it reads only `UInt8` inputs), `allocate_data_sample`/`DataSampleHandle`/`send_data_sample*` (removed in dora 1.0), `init_dora_node_from_id`/`init_dora_node_flexible`/`node_id`/`dataflow_id` (removed in 1.0), or operators. This list goes into the source as a WHY comment.
- Threads:
  1. **main**: a 2 s watchdog around `init_dora_node()` (exit 4 naming the pinned version if it hangs); `dlopen` the process's one native `.so`; `lk_create`; send `lk_hello`; hold data until every peer's hello verifies (2 s, else exit 3 naming both sides' versions); `lk_start`.
  2. **RX** (main after start): `next_event` → hello inputs verified; data inputs → `event_as_arrow_input` → schema format compared once per port → `lk_push`. `Stop` / `AllInputsClosed` → `lk_stop`, exit 0.
  3. **TX**: `lk_wait` → for each exposed `Out`, `lk_take` → `send_arrow_output` (dora's own one copy into its sample, as raw dora) → release.
  4. **RT**: inside the box, from `lk_start`. It never touches dora, so a 4 s daemon stall shows only as input age (T4).
- `lk_hello` is a reserved output, wired by the generated dataflow to every peer node: `{"like_keras": "...", "abi": 1, "backend": "dora", "dora": "0.5.0", "ports": {"<port>": "<schema hash>"}}`.

## 7. Messages across processes and languages

### 7.1 The wire is the Arrow C Data layout of each `robot_msgs` type, owned by like_keras

`like_keras/codec.py` derives the layout from the dataclass fields (never from dora, never from pyarrow):

| `robot_msgs` field | Arrow (C Data format string) |
|---|---|
| the message | a struct array of length 1, children in dataclass field order (`+s`) |
| `float` / `int` / `bool` / `str` | float64 `g` / int64 `l` / bool `b` / utf8 `u` |
| `tuple[str, ...]` | list of utf8 (`+l` → `u`) |
| `Array` (per-joint or `sample()` field) | list of float64 (`+l` → `g`) over the C-order flat buffer; the shape is `(len(names),)` or `(len(t), len(names))` as `robot_msgs` declares the field; int64 arrays where `robot_msgs` annotates them |
| nested `Msg` (`Header`, `JointLimits`) | nested struct |
| `Mapping[str, str]` (`Header.meta`) | map of utf8 → utf8 (`+m`) |
| `X | None` | the child with a validity bitmap |
| `ClassVar` | skipped |

A field type outside this table (tuples of messages, unions) is refused by `describe()` naming the field. That is why `JointCommandSequence` is out of v0 (§13).

- **Schema hash**: the first 8 bytes, little-endian, of SHA-256 over `"<registry name>|"` followed by `"<child name>:<format>:<nullable>;"` for every child, depth first. It travels **once per connection** (`lk_hello`, the attach `HELLO`, `lk_describe()`), never per message, so like_keras does not depend on dora's metadata format (which 1.0 changed).
- **Zero copies on the Python send path**: the struct is built with `pa.Array.from_buffers` over the frozen numpy buffers (dora backend), or as raw `ArrowArray` structs over the same buffers through `ctypes` (`inproc`, no pyarrow). dora's own copy into its sample is the only copy, as with raw dora.

### 7.2 The v0 codec types

`control/ServoState`, `control/ServoCommand`, `trajectory/JointTrajectory`, `sensor/JointState`, `std/Header` (nested), `std/LogRecord`, `std/DiagnosticRecord`, `std/Metrics`, plus whatever type the reference planner's `target` uses if it crosses a process. Golden bytes: `kpi/golden/<type>.lkf`, one fixed instance each (with a NaN payload `0x7ff8000000000001` and a non-ASCII joint name), written by the ruler lane's independent encoder `kpi/golden/make_golden.py` (numpy + `struct`, not `codec.py`). Python and C++ must reproduce them bit for bit, both ways (T10).

### 7.3 Python receive

numpy views over the received Arrow buffers (`to_numpy(zero_copy_only=True)` under dora; raw pointers under `inproc`), passed through `robot_msgs.adopt`, then the message is built without re-validation when the schema hash matched at connect. The views keep the transport buffer alive through their `base` chain.

### 7.4 C++ message views (generated, committed)

`python -m like_keras.codec --cpp` writes `cpp/include/like_keras/msgs/<package>.hpp` (namespace `rm`), committed with a header comment carrying the generator hash; CI fails if regenerating changes a byte. CMake never runs Python, so the controller `.so` builds on a host with no Python.

- **Per-joint types** (`ServoState`, `ServoCommand`, `JointState`, and the facility types) are fixed-capacity values: `std::array<double, lk::kMaxJoints>` per array field plus a joint count; joint names are `std::string_view`s into storage the component owns (set in `activate`). Usable as `In` and `Out`. A boundary copy of one is < 1 KiB.
- **Sample-major types** (`JointTrajectory`) are read-only views: `std::span<const double>` per field, row-major `(N, J)`, over the received buffers; the buffers stay alive until the RT thread retires the message. `In` only in v0.

### 7.5 LKF: one message at rest or on a socket

Attach frames and recordings carry one message as **LKF**, the Arrow C Data buffers flattened:

```
0   u32  magic "LKF1"           4   u32  buffer count n
8   u64  schema hash            16  i64  array length (1; 0 = no value)
24  u64  buffer lengths[n]      ..  the buffers, each 8-byte aligned, depth first (validity, offsets, data; then children)
```

A decoder rebuilds an `ArrowArray` whose pointers point into the frame; no Arrow library is needed on either side.

## 8. Standard facilities (criterion 9; zero user lines, both languages)

| Facility | Author writes | Framework does | Shape |
|---|---|---|---|
| log | `self.log.info(...)` (stdlib) / `log.info("fmt", a, b)` (C++) | `log` port; C++ via a lock-free ring drained by the process's facilities thread | `robot_msgs.std.LogRecord` |
| metrics | optional `metric(name, value)` | 1 Hz: `steps`, `step_p50_s`, `step_p99_s`, `step_max_s`, `period_error_p99_s`, `overruns`, `input_age_s.<port>`, `drops.<port>`, `refused_writes.<port>`, plus custom names (a custom name equal to a standard one is refused) | `robot_msgs.std.Metrics` |
| health | nothing | 1 Hz: `mode` from the lifecycle; `level` = `error` (`Exception`), else `warn` (`InputStale`, `Overrun` in the last period, or a warning logged in the last period, its text as `message`), else `ok`; `expected_period_s` = 1.0. **STALE is never sent**: an observer (`lk top`, the attach gateway) derives it when no record arrived for max(3 × `expected_period_s`, 5 s), as in io-diagnostics' record v0 | `robot_msgs.std.DiagnosticRecord` |
| params | a nested frozen `Params` (Python) or an `LK_PARAMS` aggregate (C++) | JSON round trip; shown in `describe()`; static per activation in v0 (a change is deactivate → rebuild → activate) | JSON |
| lifecycle | optional `activate` / `deactivate` | driven by the Deployment and by attach `CONTROL` frames; restarted components come back through `activate` | `health.mode` |
| clock | `now()` | §5.6 | float s |
| recording | nothing | `Deployment.record(path, *ports)` writes **MCAP** (unchunked, no summary in v0): one channel per recorded port (`topic = <path>.<port>`, `message_encoding = "lkf"`, schema = the Arrow format string with `schema_encoding = "arrow-c-format"`), `log_time` = when recorded, `publish_time` = `header.stamp`. Replayed by `lk.replay` through the same ports | MCAP |
| introspection | nothing | `describe()` (§4.4) is the device datasheet; served on the attach socket; read by `lk ls` and `lk top` | JSON |

The C++ RT thread only writes counters and fixed-bin histograms (relaxed atomics); formatting, publishing and percentiles happen on the facilities thread. Facility code stays ≤ 30 % of first-party SLOC.

## 9. Backends v0

### 9.1 `inproc`

- One process: the launching one. Every group is a thread; native groups are the `.so`'s own RT thread started through `lk_start`. Python groups are `threading.Thread`s.
- Serves the attach socket itself (§9.3) when the System exposes ports.
- Logical clock (`realtime=False`): one thread steps every group in time order, natives through `lk_step`.

### 9.2 `dora` (dora-rs 0.5.0)

- `start()` writes `build/like_keras/<system>/{dataflow.yml,plan.json}`, checks `dora --version` against `VERSION` (2 s timeout; refuse naming both), then `dora start dataflow.yml --name <system> --detach`. The launching process runs no components; every process, `main` included, is a dora-spawned node, so a crashed one is restarted by dora. `stop()` runs `dora stop --name <system>`.
- **Dataflow emission rules** (`emit.py`): nodes only, never operators; `path`, `args` and `env` at node level, never `custom:` (rejected by 1.x); `restart_policy: always` on every node; `queue_size` = the input's queue (1 for latest; the receiver keeps its own latest slot either way); ids are the dotted component path and port, validated against `[a-zA-Z0-9_.-]`; `_unstable_deploy: {machine: <host>}` only when `host=` is set (the 0.5 spelling; the 1.x rename to `deploy` happens in this file only); `lk_hello` outputs wired to every peer; a `gateway` node (below) when the System exposes ports.
- **Python processes**: `python -m like_keras.backends.dora.host <plan.json> <process>` (the plan path is an argument, never an environment variable). It rebuilds its components from `module:qualname(**params)`, runs one thread per group, receives on its own RX thread with `node.next(timeout=0.1)` (dora 0.5.0 releases the GIL inside `next`, so groups keep running), and sends from its TX thread.
- **Native processes**: `lk-dora-host` (§6.4). A process holds either Python groups or one native `.so`, never both.
- **Gateway**: `python -m like_keras.backends.dora.gateway <plan.json>`, a node wired to every exposed port, serving the attach socket for the whole dataflow. This is the OBSBOT gateway pattern, and it sidesteps dora 0.5's unanswered late-joining dynamic nodes.
- **Lockstep** under dora: §5.5; the step tag rides in the frame's dora metadata (`lk_t`, int64 ns), in lockstep only. Real-time frames carry no like_keras metadata.
- **Skew**: CLI checked at `start()`; node-linked versions and per-port hashes checked by `lk_hello` within 2 s; a mismatch exits non-zero naming both versions and never hangs (T11).
- **What still leaks, on purpose**: supervision is dora's `restart_policy`; dora sends below 4 KiB over TCP and above it through shared memory (`describe()` reports the transport per edge, it does not hide it); every host must run dora 0.5.0.

### 9.3 Attach: the IP-camera idea, same host in v0

- **Address**: `unix:$XDG_RUNTIME_DIR/like_keras/<system>.sock`, printed by `describe()` and `lk ls`. A client always passes an explicit address; `lk ls` with no argument lists that directory as a convenience, never as the source of truth.
- **Frames**: `u32 length | u8 kind | payload`. Kinds: `HELLO` (JSON: versions, describe hash), `DESCRIBE` (the datasheet), `SUB` (port), `DATA` (u16 port + LKF), `WRITE` (u16 port + LKF), `LEASE` (heartbeat), `REFUSED` (JSON: reason, holder), `CONTROL` (`activate` | `deactivate`).
- **Readers**: a new subscriber gets the cached latest value of each subscribed `Out` at once. Each client has a bounded send queue (16 frames); a full queue disconnects that client. The deployment never waits on a client.
- **Writers, the write lease**: one attach client at a time may write to the deployment's exposed inputs. The lease is taken by the first `WRITE` or `LEASE` and renewed by every `WRITE` and by the proxy's heartbeat (every `lease_s / 3`). Any other client's write gets `REFUSED` naming the holder, counted in `metrics.refused_writes.<port>`. When the holder is silent for `lease_s` (crashed, killed, slow), the lease lapses and the gateway **retracts** every input the holder wrote (§5.4): the controller then holds where it is (R5). The next client may take the lease at once (T5, T8).
- **CLI**: `lk ls [ADDRESS]` lists exposed ports with type, schema hash, policy and the dora version within 1 s; `lk top ADDRESS` shows every component's health and metrics, live, deriving STALE.
- TCP and other hosts are v1; until then `ssh -L` forwards the socket.

## 10. The reference consumer and the acceptance tests

`kpi/tasks/arm_cell/` (frozen, written by the ruler lane in this contract's API): `planner.py` (§4.6), `targets.py`, `arm_controller.cpp` (§6.2), `arm_controller_py.py` (`ArmControllerPy`, the same exposed ports over a Python twin), `fake_bus.{hpp,py}` (a 3.5 ms exchange, the board watchdog at 0.25 s into timeout mode, `BUS_LOCK`), and `cell.py` with three deployments of the same files:

| Deployment | What | The one line that differs |
|---|---|---|
| `target` | the controller in its own process (`lk-dora-host`), the planner in `main`, dora between | `backend=dora` |
| `same_box` | the C++ chain on its own SCHED_FIFO thread inside the Python process | (default `inproc`) |
| `fallback` | all Python, one process | `arm = ArmControllerPy(...)` |

Every test runs on the fake bus, with the rack unpowered or on one fixed cloud VM; the 24-core load is `kpi/load.py --cpu 24`. Each test counts only if it passes in every placement it names.

| Test | Placements | Setup | Pass |
|---|---|---|---|
| **A1 trajectory hand-off over dora, non-blocking** | `target` | The planner returns a 7-joint × 1000-sample `JointTrajectory` every 0.5 s for 60 s while the node ticks at 100 Hz | `forward` returns to the planner's group with no transport call on its thread: from `forward` returning to the next step's start p99 ≤ 1 ms; the controller starts each plan at the first tick ≥ its `header.stamp` (error ≤ 1 tick); sent = received; the node's tick-start jitter is unchanged (T3 limits) |
| **A2 the same planner in one process** | `same_box` | A1's load, `inproc` | Same, with the push p99 ≤ 0.1 ms; the RT thread never takes the GIL (asserted by a GIL-held-by probe) |
| **A3 the same planner, all Python** | `fallback` | A1's load | Same outputs as A2 (T1) |
| T1 Swap | all three, lockstep | Recorded targets | The same command on every plan tick, to 1e-9 rad |
| T2 Send cost | `target`, `same_box` | 1000 sends of 7 × 1000 while the node ticks | p99 ≤ 0.1 ms in-process, ≤ 1 ms over dora |
| T3 Isolation (10 min) | `target`, `same_box` | The planner spins pure Python with `gc.collect()` every 50 ms, under `kpi/load.py --cpu 24`; 100 Hz | Tick-start jitter p99 ≤ 0.2 ms, max ≤ 1 ms |
| T4 Boundary stall | `target` | `kill -STOP` the dora daemon for 4 s mid-trajectory | The trajectory finishes on time, then holds; 0 missed ticks; `InputStale` visible on `lk top` during the stall (attach does not go through the daemon) |
| T5 Planner crash | `target`; and the always-on form with an attached planner | `kill -9` the planner, restart it | The arm holds; the restarted planner commands again within 1 s |
| T6 Node crash | `target` | `kill -9` the node | Fake boards in timeout mode within 0.25 s; on restart `take_over`, no mode 0, no bare stop |
| T7 Exclusivity | `target`, `same_box` | Start a second node | Refused in `activate` before the port opens; exits non-zero having sent 0 frames |
| T8 Claim | always-on form, two attached planners | Both write at once | Only the lease holder's trajectories run; the other gets `REFUSED` (its proxy's health `error`, code `LeaseHeld`); when the holder stops, the inputs retract and the arm holds |
| T9 Overhead | C++ chain | Google Benchmark vs hand-written calls, plus the malloc hook | ≤ 100 ns per edge (criterion target; R12's floor is 1 µs); 0 allocations per tick |
| T10 Codec | Python ↔ C++ | `kpi/golden/*.lkf` | Bit-exact both ways; a hash mismatch refused at connect |
| T11 Version skew | `target` | A node or CLI on another dora version | A refusal naming both versions within 2 s; never hangs |
| T12 Introspection | `target`, `same_box` | `lk top <address>` | Shows every R14 metric |

The **always-on form** (R2) is a Deployment of the controller alone (`dep.place(arm, process="arm-controller", hz=100, ...)`, exposing `trajectory`, `state`, `command`), started under a systemd user unit; planners reach it with `lk.attach`. T5's attached variant and T8 use it.

Out of v0's acceptance (stay with the controllers lane): real-arm runs, the 1 h soak, rung parity of the C++ JTC (T13).

## 11. The KPI scoreboard (the ruler the build loop ratchets on)

`kpi/score.py` runs every measurement, writes `kpi/scoreboard.json` (the 12-number vector, the gates, the total), and compares it with the committed `kpi/baseline.json`. Each criterion scores `s = clamp((zero − x) / (zero − target), 0, 1)` (mirrored when higher is better); the total is `Σ wᵢ sᵢ` when every gate is green, else 0. The vector is always published beside the total.

### Gates (any red = 0, the change does not land)

| Gate | Pass | Measured by |
|---|---|---|
| Equivalence | identical output streams in one process, across processes and with the C++ replacement, on recorded inputs, in lockstep; sent = received | `pytest -q kpi/contract` |
| Fallback untouched | `arm_hold`, `pick_loop_v1`, `fivebar_run` byte-for-byte as today; `bam_rack_twin` tests pass with like_keras absent | `uv run --directory /home/bam/bam_ws/src/dev/bam_rack_twin pytest -q` |
| Real-time hot path | 0 heap allocations per tick over 100k ticks of the C++ chain; no lock shared with a non-RT thread | `build/kpi/alloc_probe` (gtest, `operator new`/`malloc` counter) |
| Lean core | ≤ 1 runtime dependency (`robot_msgs`); `import like_keras` loads no `robot_msgs`, numpy, dora, pyarrow or yaml | `python3 kpi/deps.py` |

### Criteria

| # | KPI (w) | Definition | Measured by | v0 target | Counter-metric (must not regress) |
|---|---|---|---|---|---|
| 1 | Consumer fit (15) | share of T1–T8, T12 passing in every named placement | `pytest -q kpi/consumer` | 9/9 | A1–A3 pass; the fallback gate |
| 2 | Speed (13) | p99 added over raw dora 0.5.0 (64 B / 4 KiB / 1 MiB / 7×1000, open loop at 1 kHz, Python and C++ senders, ≥ 5 interleaved A/B pairs, pinned to cores 20–21); per-edge cost vs a direct call; copies per hop ≥ 4 KiB and idle CPU; cold import | `kpi/bench_overhead.py --pin 20,21`, `build/kpi/bench_chain`, `python -X importtime` | ≤ max(10 µs, 5 %); C++ ≤ 100 ns, Python ≤ 1 µs per edge; 0 extra copies, CPU ≤ raw + 5 %; ≤ 20 ms | sent = received; throughput, CPU and time to first message reported beside latency; the RT → TX wake and the PyDLL path included |
| 3 | Black box (11) | share of reference components and Systems that name no peer, transport, System or Deployment; 0 edges in component files; the step runs with dora and pyarrow blocked; no request/response; no env or registry reads outside backends | `uvx --from import-linter lint-imports`, `kpi/blackbox.py`, `pytest -q kpi/blackbox` | 100 % | workflow tokens (criterion 4) may not drop by moving wiring into components |
| 4 | Succinctness (11) | public names (Python ∪ C++-only); widest signature; tokens of `kpi/tasks/*/solution` vs raw dora and plain Python | `kpi/surface.py --api --tasks`, `uvx griffe dump` | ≤ 8 / ≤ 30 names (v0 starts at 8 / 33); ≤ 6 params; ≤ 0.4× raw dora, ≤ 1.2× plain Python | cold-agent pass@1 ≥ 80 % (Sonnet 5.5, README + `help()` only, n = 3; reported) |
| 5 | Swap cost (9) | lines changed across every file to swap process, language, backend, host | `kpi/swap_cost.py` | 0 component/System lines, ≤ 1 Deployment line per swap | every variant passes equivalence; `describe()` prints the full placement (no hidden env or YAML) |
| 6 | dora insulation (8) | files naming dora outside `backends/dora` and `cpp/hosts`; wire owned by like_keras; nodes only; skew refused ≤ 2 s; conformance green on both backends | `grep -rIl -i dora src include --exclude-dir=dora`, `pytest -q kpi/conformance kpi/churn` | 0 files; 5/5 | v1 rehearsal on a cloud VM: diff outside the backend = 0 (reported) |
| 7 | Compactness (8) | first-party SLOC after `ruff format` / `clang-format`, generated code counted | `kpi/surface.py --size` | core ≤ 650, each backend ≤ 300, attach + CLI ≤ 250, C++ runtime incl. `dora_host` ≤ 850, generated views ≤ 100, total ≤ 2,450 (the criterion's limits are 700 / 350 / 900 / 2,500) | workflow size (criterion 4) and the lean-core gate catch complexity pushed into users or dependencies |
| 8 | Python + C++ reach (7) | mixed chain in and across processes; C++ needs no Python or ROS at runtime or build; one `.so` in 3 placements; bit-exact codec; ≤ 2 files and ≤ 10 non-body lines per C++ component | `pytest -q kpi/polyglot`, `kpi/surface.py --cpp-boilerplate kpi/tasks/cpp_gain` | 5/5 | the generated-header hash check stays green |
| 9 | Facilities (6) | 7 facilities on an empty-step component, each language, each backend, 0 user lines | `pytest -q kpi/facilities` | 7/7; facility SLOC ≤ 30 % | the alloc gate (no facility work on the RT path) |
| 10 | Testability (5) | twin swap in ≤ 1 line; 10 seeded lockstep runs hash-identical; replay bit-exact; a component tested alone | `pytest -q kpi/twin` | 4/4 at ≥ 10× real time | no wall-clock sleep in any test (`kpi/blackbox` scans) |
| 11 | Network attach (4) | a process started 1 s later lists ports ≤ 1 s, receives `state` ≤ 1 s, writes under the lease, dies without hurting the node, via an explicit address | `pytest -q kpi/attach` | 4/4, same host | T3 jitter unchanged with 3 slow clients attached |
| 12 | Wiring checks (3) | the ~20 frozen mis-wirings rejected before any process starts, each naming `path.port`, the fix and `file:line` | `pytest -q kpi/miswire kpi/validgraphs` | 100 %, 0 false positives | any false positive scores 0 |

### Land rule

- All gates green and the total ≥ the committed baseline.
- No criterion falls by more than its noise band, unless the commit carries `KPI-tradeoff: <criterion> <old>→<new> because …` and the total rises by ≥ 2.
- Timing criteria run as ≥ 5 interleaved A/B pairs and ratchet on the conservative edge of a bootstrap 95 % CI. Results from different hosts are never compared.
- A builder diff that touches `kpi/` is rejected. Consumer friction (`docs/friction/NNN.md`) becomes new frozen tests through the ruler lane.

## 12. Milestones and lane order

| Step | Who | Lands | Exit (green on the scoreboard) |
|---|---|---|---|
| M0 (barrier, alone) | ruler; robot_msgs | `kpi/` complete and committed; `robot_msgs` facility types and public `adopt` | the scripts run and report the empty repo as 0; golden bytes generated by the independent encoder |
| M1 (barrier: the API) | core | Python core, `inproc`, codec, record/replay, lockstep, generated C++ views | gates lean core and equivalence (Python, inproc); criteria 3, 4, 10, 12; facilities (Python); A3 |
| M2 | cpp | `lk.hpp`, `abi.h`, RT thread, C ABI, `inproc` native via `PyDLL` | gate RT hot path; `bench_chain`; T1, T2, T3, T7, T9, T10 in `same_box`; A2 |
| M2′ | controllers lane | ports its chain (today on raw dora: `mcn/chain.hpp`, `handoff.hpp`) onto `lk.hpp` in `same_box`; files friction notes | its own twin tests green |
| M3 | dora | `emit.py`, Python host, `lk-dora-host`, gateway, attach, `lk ls`/`lk top`, skew | conformance on both backends; T1–T8, T11, T12 in `target`; `bench_overhead` A/B; A1 |
| M3′ | controllers lane | `target` on the real controller `.so` (fake bus first, then its own sim gates) | the consumer's acceptance on its fake bus |

The controller lane's current raw-dora node (C node API, a timer-driven single I/O thread, the `RMSG` byte frame in `mcn/wire.hpp`) is the consumer baseline for criterion 2's A/B. When it ports, `wire.hpp`'s frame gives way to §7 and its dora `main` to `lk-dora-host`. Its `handoff.hpp` (triple buffer, SPSC ring, retired-plan ring) is the shape of §5.1's slots and may be lifted into `lk.hpp` with credit.

## 13. Out of scope for v0

- **Network**: TCP/LAN attach, authentication, discovery of any kind; zenoh, iceoryx2 and ROS 2 backends; dora 1.x (rehearsed on a cloud VM, not shipped).
- **Messages**: `JointCommandSequence` and any type with tuples of messages or unions (v0.1); Rust components; message types defined by like_keras (never).
- **Runtime**: live parameter changes (a change is a rebuild); a lifecycle data port; a supervision API (dora's `restart_policy`, systemd outside it); hot-swapping a `.so` without a restart; Python in a SCHED_FIFO group; more than one native `.so` per process; a live two-host run (the swap is compiled and line-counted; the run is v0.1).
- **Tooling**: MCAP chunking, compression and indexing; a graph viewer (Clank's job); Windows.
- **Acceptance owned elsewhere**: real-arm runs, the 1 h soak, rung parity of the C++ JTC.

## 14. Decisions taken in this contract (each reversible)

| # | Decision | Why | Undo |
|---|---|---|---|
| D1 | Base P1, C++ authored in C++ (not P3's Python-declared, generated C++) | P3 puts Python on every C++ build host and inverts consumer spec (b) | Take `design/2026-11-15/proposal-P3-spec-compiler.md` as the base; §5 and §7 carry over |
| D2 | Attach is same-host only (unix socket) in v0 | No vault design for LAN attach yet; criterion 11 weighs 4 | Move ~5 weight to criterion 11 and add TCP to §9.3 |
| D3 | Python wiring is subscribe-direction kwargs only (`add(..., state=arm.state)`, `wire(c, **inputs)`) | Zach's settled form; every proposal had dropped it | Add `System.wire(out, *ins)` beside it |
| D4 | 33 public names, 3 over target, for `take()` (new-or-null without pointer identity), `replay` and `metric` | Each one serves a consumer test; the ratchet lowers the count | Drop any of the three and accept the lost test path |
| D5 | Arbitration is a write lease at the attach gateway, not a `Claim` message type | A claim message only works if every trajectory names its sender (stringly `header.meta`) | Add `robot_msgs.control.Claim` and an exposed `claim` input |
| D6 | dora's cxx API for C++, dora's own Python package for Python | The C API is unsound with two threads and reads only `UInt8`; one compiled edge under every Python process (P4) taxes the vanilla-Python goal | Route Python's dora I/O through `lk-dora-host` (P4's edge) |
| D7 | `fifo`/`cpus` ignored (and reported) on Python groups rather than refused | Keeps the Python↔C++ swap one line; nothing unsafe is applied | Refuse them in §4.3 and accept 2-line swaps |
| D8 | C++ message views live in like_keras, not `robot_msgs` | They are spans into like_keras's wire layout | Move the generator to `robot_msgs.cpp_gen` when a second C++ consumer appears |

## 15. Amendments (2026-11-15, dispatcher, after the M1 and M2 friction)

The frozen text above stays as written; where an amendment below contradicts it, the amendment wins. Each one answers a friction note in `docs/friction/` and is reversible.

| # | Amends | Decision | Why | Undo |
|---|---|---|---|---|
| A1 | §12 M1/M2 exits, criterion 3 | Criterion 3 is judged in full at **M2's exit**. At M1 it was the Python blocked-step rows, the static scan and import-linter. The ruler moves every module-level `from like_keras.backends import dora` in `kpi/` into the function that builds the dora deployment (`kpi/tasks/arm_cell/cell.py`, `kpi/tasks/arm_cell/swaps/*/*/deploy.py`), as `kpi/tasks/relay/solution/relay.py` already does. | The C++ rows need M2's `.so`. The `ArmControllerPy` row was red only because the ruler imported the module its own blocker refuses (contract-004 §1, ruler-002 §2). | Restore the module-level imports and score criterion 3 at M3 |
| A2 | §1 ownership, §12 M1 | Ratified: the capability-only stand-in `src/like_keras/backends/dora/__init__.py` that core landed at M1. Lane dora owns that file and replaces it whole at M3. | `kpi/validgraphs` calls functions that import the backend, so the module must import before M3 (contract-003 §1, contract-004 §3). | Delete the stub and move the two validgraph cases to M3 |
| A3 | §12 M1/M2 exits, criterion 9 | Facilities `introspection`'s `lk ls` step moves to **M3's exit**. M1 and M2 must pass the other six facilities on python-inproc and cpp-inproc. | `like_keras.cli` and the attach socket are lane dora's (contract-004 §2). | Put the step back in M1 |
| A4 | §7.4 | The bound reads "about 1.1 KiB at `kMaxJoints = 16`" (ServoState 1104 B, ServoCommand 1128 B). The layout is unchanged. | Sixteen `std::string_view` names and the optional arrays cost what they cost, and the ruler's C++ pins the member spelling. A 1.1 KiB copy at 100 Hz is nothing (contract-003 §4). | Ruler edits `jtc_core.hpp:106`, `fake_bus.hpp:155,181`; core emits `std::span<const std::string_view> name` (864 / 888 B) |
| A5 | §11 criterion 7 | The targets **stay** (total ≤ 2,500 scores 1, total 5,000 scores 0). M1 and M2 came in over them: core 1,190 SLOC, inproc about 670, the C++ runtime 1,500. A compaction milestone, **M4**, follows M3 (below). It may generate the per-type codec tables from `codec.py` into the committed views. It may also replace the `LK_PARAMS` JSON parser with a flat `key=value` string, provided `LK_PARAMS(Type, ...)` and the Python call site stay as §6.1 and §4.5 show them. | Compactness is one of Zach's three named KPIs; moving the target to fit the code would grade the build on its own output (contract-005 §1). | Raise the targets in §11 and drop M4 |
| A6 | §5, §6.3, §8, §9 | Ratified: the readings in contract-003 (the backend protocol; `attach`'s name; the process-name rule over every `like_keras.backends` module; lockstep visibility; recording topics; health codes incl. `WarningLogged`; adopt-on-receive) and contract-005 §2-5 (facilities as the extra out ports `log`, `health`, `metrics`; `lk_push(box, port, NULL)` retracts; `lk_describe()` builds from default `Params` with no activation and no I/O; the queue-ring, string-arena, FIFO-refused and `kMaxJoints` readings). | Each fills a gap the contract left without contradicting it. | Amend the specific reading |
| A7 | `kpi/` (the ruler) | The ruler lane makes one amendment commit ("kpi: M0.1"), and from then on that commit is the frozen ruler. It contains: ruler-001 (`score.py` runs `bench_overhead.py` through `kpi/run.py`'s private-namespace wrapper and passes `--pin 20,21` in the timing tier; a valid graph that does not pass counts as a false positive); ruler-002 §1 (the twin's ≥ 10× speed check gets its own test id, marked `timing`; the default tier keeps the ten-hash determinism check and asserts ≥ 10× on CPU time); §2 (= A1); §3 (`emitter`/`build` move out of the module that imports the pyarrow oracle); §4 (`surface.py` records class signatures as `<name>.__init__`); ruler-003 (`test_health` accepts `warn/Overrun` beside `ok` in the default tier; the `ok`-when-unloaded half moves to a `timing`-marked twin); ruler-004 §1 (T10 exposes the echo box's unwired inputs) and §2 (A2 lists native threads while the deployment runs). Every change adds or corrects; none weakens what a test asserts in the tier where it can be judged. | Each defect is a ruler bug a builder may not fix; all were reproduced by an independent verifier. | `git revert` the M0.1 commit |
| A8 | §12 exits, criterion 4 | Before M3, criterion 4 is judged as **measured and not regressed** (0.6866 at M1 and M2), not against its v0 target. | Its workflow part cannot rise before M3: relay's reference solution runs only on dora, and the ruler froze pipeline's solution tokens (M1 verifier, round 4). | Judge it against the target at every milestone |
| A9 | §6.3, §8 (contract-005 §2 as ratified in A6) | Ratified: each native box's metrics round starts with one box-level record, path `""` (the host names it the box's own path), carrying the whole step's standard names plus `drops.log` and `unreleased`. That path has metrics and never health, so `lk top` and every observer must not mark it STALE. | It is where counters that belong to no one component get published (M2 verifier, round 1). | Drop the record and attach the two counters to the first child |
| A10 | §5.5, §8 | Under lockstep, every facility record (health, metrics) is stamped at the facility tick, its publish time on the logical clock, in **both** languages: `inproc` re-stamps a native box's records, and `uptime_s` shifts by the same amount. A Python ↔ C++ swap then changes no facility stamp. Owner: lane core/cpp, landed with the consumer-friction fixes; `tests/test_native_facilities.py` follows. | Today C++ stamps the last step's time (0.98, 1.98 s) and Python the tick (1.0, 2.0 s), so a swap shifts the stamps by one period (M2 verifier, round 2). | Accept the one-period shift and document it |
| A11 | §1, §4.3, §8, §9.3 (contract-006 §1-3) | Lane core makes three changes so attach works: an `In` marked `optional` (every attach-proxy input) may stay unwired and gets no slot; `health.code` comes from the component's state (`LeaseHeld`), defaulting to `Exception`; and the backend protocol gains `run.subscribe(leaf, port, sink)` as the one seam that attach, `lk top` and the dora Python host subscribe through (replacing `run._box(leaf).sinks`), kept through M4. | T5 `attached`, T8 and three of criterion 11's four tests fail at `describe()` without them. | Revert the three changes; T5/T8 return to red |
| A12 | §5.5, §6.4, §9.2, §9.3 (contract-006 §4-13) | Ratified: §6.4's dora call list gains `event_as_arrow_input_with_info`, `new_metadata`, `Metadata::set_int`/`get_int` and the metadata overload of `send_arrow_output` (everything §6.4 bans stays banned). §5.5 under dora: frames carry `lk_t`, and the last frame of a step, or an empty one, carries the sender's promise `lk_h`. Also ratified: the acknowledged hello handshake, `<process>.host`, `start()` returning once every node has activated, the gateway writing recordings and ending lockstep runs, `hz` required for a native group under dora lockstep, the attach server's own records, and `dataflow.yml` written as JSON. | Each fills a gap or fixes an unbuildable line without changing a component's code. | Amend the specific reading |
| A13 | §6.4, T11 | The init watchdog stays 2 s: T11 needs a skewed daemon refused within 2 s, and before `init_dora_node()` returns a slow peer and a skewed daemon look the same. Exit 4 therefore means "a peer was slow or the daemon is skewed"; the timing tier reports how often a healthy start comes near 2 s. | Splitting the barrier from the skew check needs a probe dora 0.5.0 does not offer (contract-006 §6). | Bound the barrier at 10 s and accept a slower T11 |
| A14 | `kpi/` (the ruler) | A second ruler amendment, "kpi: M0.2", becomes the frozen ruler. It contains: ruler-005 (a `realtime=False` half of `test_health`, and of the facility fixture, for every inproc combination; strict `ok` under lockstep); ruler-006 §1 (`test_clients_cannot_hurt_the_node` compares against a no-client baseline taken just before it, e.g. ≥ 0.9 × the baseline's ticks and a max gap ≤ the baseline's + 20 ms) and §2 (T5's 1 s bound moves to the timing tier, and the default tier asserts the hold plus commands again within 5 s); the two default-tier real-time conformance bounds the M2 verifier saw flake (`test_now_is_scheduled_time`, `test_forward_exception_goes_error`): their absolute wall-clock halves move to the timing tier; and the reference cell's guard (`kpi/tasks/arm_cell/guard.hpp` and its Python twin) **recovers instead of latching**: after a runaway hold it accepts the next plan that starts within the start tolerance of the measured position, and it logs the event (Zach, 2026-11-13: robots recover, they don't stop). Every change adds or corrects; nothing weakens what a test asserts in the tier where it can be judged. | Each is a ruler defect a builder may not fix, measured by an independent verifier or a builder on this host. | `git revert` the M0.2 commit |
| A15 | `kpi/` (the ruler), ruler-006 §1 | The next ruler commit makes `test_clients_cannot_hurt_the_node` compare against the **worse of two no-client windows** (one before the bad clients, one after), keeping 0.9 × ticks and + 20 ms on the max gap. Until then a failure of that one test on this host is read as host noise, never as a lane failure. | M0.2's single-window baseline still failed 2 runs in 12, and two no-client windows back to back broke the gap rule 1 time in 8 (ruler lane, M0.2). | Keep the single window |
| A16 | §7.4 (consumer-001) | Sample-major C++ views (`rm::JointTrajectory`) carry `joint_names` (`std::array<std::string_view, lk::kMaxJoints>` + count) pointing into the received buffer, valid for the step. Lane core's generator. | A C++ controller must map a plan by name, not by column order; a reordered plan moved the wrong boards. | Drop the field; map by order |
| A17 | §5.1, §6.1, §6.3 (consumer-002) | `In<T, Converted>`: a C++ input may name a conversion type with a static `Converted from(const T&)`. like_keras calls it on the **pushing** thread inside `lk_push` (allocation allowed there), hands the RT thread a pointer, and retires it through the graveyard when the input moves on. No new public name. This is the raw node's PlanMailbox, lifted. | A latest view dies under a component still using it, so every plan was copied on the RT thread. Choosing the host-side conversion over a `keep()` pin keeps O(N·J) work and validation off the RT thread. | `In<T>::keep()` pins instead |
| A18 | §5.7, §6.4 (consumer-003) | SIGTERM and SIGHUP end a deployment the way SIGINT does: `Deployment.start()` installs handlers (when none is installed) that request `stop()`, `lk-dora-host` takes them on a `sigwait` thread that calls `lk_stop`, and the facility ports are drained once more after the last `deactivate`. | Every component's `deactivate` (the controller's hand-back) must run on the ways a process is usually ended. | Leave signal handling to the application |
| A19 | §5.1, §6.1, G12 (consumer-004) | The RT thread's scheduling ladder is the library's: SCHED_FIFO, else rtkit's SCHED_RR (requested from a host thread before the loop starts), else SCHED_OTHER, reported in the box's metrics; when rtkit grants, RLIMIT_RTTIME is set soft < hard and SIGXCPU demotes instead of killing (the raw node's sentinel, lifted with credit). `mlockall` stays limited to a dedicated controller process (G12). No hook for components. | The real arm's host has no rtprio limit and rtkit caps grants; the raw node needed six rounds to make this safe, so it belongs in one place. | An `on_rt_start()` hook; components bring their own ladder |
| A20 | §3, §4.1, §6.1 (consumer-005) | `period()`: a component reads its group's period in seconds (0.0 when rate-less), from `activate()` on, in both languages. One new public name (C++ and Python), accepted against criterion 4. | A controller must refuse at open time when one bus exchange cannot fit in its tick. | A field in the params `activate` sees |
| A21 | §4.2, §8 (consumer-006) | A Python `lk.System` has `log`, `health` and `metrics` ports that carry every inner component's records with their full paths, as a native box's do. | A Python ↔ C++ swap must not change how the chain's facilities are read. | Read facilities per child |
| A22 | §3, §6.1, §8 (consumer-007, A11) | `health(level, code)` in both languages, held until called again; it replaces A11's internal `state.code`, and health reports the worse of the component's level and the facility's own. `log.error` sets health `error` / `ErrorLogged` until an `ok` round with no error logged. The C++ `Log` takes `%s` arguments that are string literals (no allocation). One new public name. | A refused or braked chain read `warn` for one second, then `ok`. | Errors only through the log |
| A23 | §6.1, §7.2 (consumer-008) | A C++ `In` may be optional (`InOpts{.optional = true}`): unwired is legal and reads `nullptr`; `describe()` lists it; an attach client may write it (A11 in C++). A guard's verdict is a `robot_msgs` `DiagnosticRecord` (no new type): `header.stamp` = the judged state's stamp, level `error` means stop, the code names why. | The guard deadman and stop/brake requests must be portable; the vocabulary stays small (robot_msgs owns types). | Add `robot_msgs.control.GuardVerdict` |
| A24 | §6.1 (consumer-009) | `LK_PARAMS` reflects `std::vector<double>` and `std::vector<Nested>` of an `LK_PARAMS` aggregate (filled at `lk_create`, never on the RT thread), and `describe()` shows every row; an unknown key in a row is refused naming `rows[2].key`. | A per-joint table is the controller's config, and the datasheet must show it. | Keep the `.ini` path |
| A25 | §4.1 (consumer-010) | `lk.In(T, fresh=True)`: `forward` gets `None` for that input unless a value arrived since the last step. An option, not a new public name. | Python needs the new-since-last-step signal C++ `take()` has; guessing by identity differs under a swap. | Keep identity guessing |
| A26 | inproc, T2 (consumer-011) | The Python export of a message is cached per type (node layout, counts, formats) or built in C (`lk_export`), so a 7 × 1000 plan's push stays near `lk_push`'s cost; T2 reports export and `lk_push` apart. | A2's 0.1 ms push budget was spent in Python, not in the library's C side (0.17 ms vs 0.013 ms). | Leave the export as it is |
| A27 | `kpi/` (the ruler) | A third ruler amendment, "kpi: M0.3", becomes the frozen ruler before any A16-A26 code lands. It contains: A15's two-window attach baseline; ruler-007 §2 (T11's hang case checks "did not answer"); and one new frozen test per consumer note 001-011, as each note's "New ruler test" describes and as A16-A26 decide. Each new test must fail on today's code (or skip only where its feature does not exist yet, failing once the feature lands half-done) and is never weakened later. | Consumer friction becomes frozen tests through the ruler lane (§11 land rule), committed before the fixes they judge. | `git revert` the M0.3 commit |
| A28 | §6.3, §8, `cmake/like_keras.cmake` (consumer-012, consumer-013) | Library fixes, no surface change: every stop of a native box drains its log, health and metrics once more after the last `deactivate()` (not only on signals, cf. A18); and `lk_component` links each component library with a version script so it exports only the `lk_*` C ABI. | A component's last words were lost, and a component library exported every static library it linked plus like_keras's own `std::` symbols, so two components could interpose. | Leave both as they are |
| A29 | `kpi/` (the ruler), ruler-008 | The next ruler commit moves the A16 test's trajectory source to module level, so its `[dora]` case can be rebuilt in another process as §4.1 requires. Until then that one case is a ruler defect, never a lane failure. | §4.1 forbids rebuilding a class defined inside a function in another process; the library is right to refuse it (lane core, `tests/test_trajectory_names.py` passes both placements with a module-level source). | Keep the case and accept it red |
| A30 | §6.3, §9.2, §8 (contract-007) | Ratified: a host marks a queue input it wires by retracting it once before `lk_start`, so an exposed optional queue gates only when wired; a dora `stop()` signals native nodes first (SIGTERM to `lk-dora-host`, pid checked against its `started` marker), waits up to 10 s for their `drained` marker, then stops the rest; a native box's log port is drained after every step under lockstep and every 0.1 s in real time (health and metrics stay at 1 Hz). Lines logged in `deactivate()` reach recordings and attach, not reader components, in both languages alike. | These keep a Python ↔ C++ swap equal for readers and make the final drain reach live readers under dora (library verifier, round 1). | Amend the specific reading |
| A31 | §6.1, A17 (consumer-014) | A conversion may see its owner: `static Converted from(const T&, const Owner&)`, where `Owner` is the component class declaring the input; like_keras calls it only between `activate()` and `deactivate()` and the owner may only read what it set before `activate()` returned (params, config built there). The message-only form stays valid. | A conversion that needs the chain's config needed a process-wide global, so two chains could not share a process. | Message-only conversions |
| A32 | §4.1, A23 (consumer-015) | `lk.In(T, optional=True)`: unwired is legal, `forward` gets `None` for it, `describe()` lists it — A23's meaning in Python. An option, no new name. | The Python fallback had to set a private attribute to mirror the C++ verdict input. | Keep the private attribute |
| A33 | §6.3 (consumer-016) | `lk_describe()` prints a non-finite params default as `NaN`, `Infinity` or `-Infinity` (what Python's `json` and `strtod` read), and `lk.native()` reads them back. | A component's natural "NaN = off" default broke `lk.native()`. | Forbid non-finite defaults |
| A34 | §4.4, §5.7, §9.2 (consumer-017) | A refused `activate()` is final under dora too: the host writes its `refused` marker and exits with a code dora does not restart, and `Deployment.start()` raises naming the refusal and stops the dataflow, in real time and lockstep alike. | A refused controller was hot-restarted 30-100 times a second and `start()` reported success. | Keep restarts and report refusals only in health |
| A35 | §6.4, §9.2, A19 (consumer-018) | The dora boundary gets the raw node's treatment: `lk-dora-host` raises its RX/TX threads, the dora threads of its process and its private daemon to rtkit SCHED_RR below the RT thread's priority, through A19's ladder, with the raw node's proven rules lifted with credit (realtime only while needed, paused around a peer's disconnect, every grant demoted at stop, no grant outliving stop); it reports `io_sched_policy`, and it counts per exposed `Out` the steps that never went out and the worst gap between sends. Exit: on the same twin scenario, `target`'s states lost at the subscriber and hand-off p99 are within noise of the raw-dora node's (the consumer's `ab_compare.py`). | `target` lost 2-3 % of states (raw: 0-0.1 %) with a hand-off tail ~10x the raw node's, because only the RT thread was raised. | Accept the gap and document it |
| A36 | §9.3, T8, D5 (consumer-019) | The attach write lease is per exposed input, not per socket: a planner holding `trajectory` and a guard holding `verdict` lapse and retract independently. T8 is unchanged for two writers on one input. | A guard in its own process could not send verdicts while a planner held the lease. | One lease per socket |
| A37 | A28, A30 (ruler lane, M0.4) | Under a plain real-time dora `stop()`, a **Python** node's deactivate log line and offline health reach live readers and the recording as a native node's do: the stop order of A30 includes the Python hosts. | M0.4's probe: a Python box's line reached neither (3/3), a C++ box's reached both. | Accept the asymmetry |
| A38 | `kpi/` (the ruler) | Ruler M0.5 freezes one test per A31-A37 before the code lands (each fails on today's library), including the consumer's A/B as a frozen default-tier check for A35's accounting and policy report and a timing-tier check for its "within noise of raw dora" exit. | Consumer friction becomes frozen tests before the fixes they judge. | `git revert` the M0.5 commit |
| A39 | A34, A30, A37 (contract-008) | Ratified: under dora 0.5.0 (which restarts on any exit under `restart_policy: always`) a refused node marks its refusal and holds its process, sending nothing, until dora's Stop, then exits non-zero; `Deployment.start()` stops the dataflow at once. A real-time `stop()` signals, in waves, every node whose outputs another component's node reads, upstream first (in a cycle, native nodes first), waiting for each wave's `drained` marker; a signalled node waits up to 15 s for dora's Stop. A34 and A30 read accordingly. | Exit codes cannot stop dora 0.5's restarts; readers must outlive the node whose last words they need. | Use `restart_policy: on-failure` and exit 0 on refusal |
| A40 | `kpi/` (the ruler), ruler-009 | The next ruler commit makes `conformance::test_activate_raising_sends_nothing[dora]` expect `start()` to raise naming the refusal (as A34 decided), keeping its no-data-frame assertion. | The M0 case expected `start()` to return; A34 forbids that. | Keep the M0 case and accept it red |
| A41 | §4.4 `describe()`, §6.3 (consumer-020) | A native box's `describe()` lists every `LK_PARAMS` field with its default, overlaid by the config given, as a Python component's does (`lk.native` already reads the defaults from `lk_describe()`). | The datasheet must show the controller's whole configuration in both languages. | Show only the given config |
| A42 | §6.3, §8, A17/A31 (consumer-021) | A push the box refuses (a conversion that throws, a push outside the activate/deactivate window, more than `kMaxJoints`) is counted per input as `refused.<port>` in the component's metrics and logged on the box's log port (the first, then rate-limited), in both hosts. | A refused plan was counted nowhere, while a library comment claimed it was. | Keep stderr-only reporting |
| A43 | `kpi/` (the ruler) | Ruler M0.7 freezes one test each for A41 and A42 before the code lands. | As A27/A38. | `git revert` the M0.7 commit |
| A44 | A24, A41 (library verifier, round after A31-A42) | A given row of a params table is filled from the row type's **member defaults** (a value-initialized row), never from the default table's row at the same index; `describe()` lists each given row filled in the same way (so `lk_describe()` exposes each table's row defaults). The datasheet is then exactly what the box runs. | A partly given row ran the default row's values at its index while `describe()` showed the row as given (both backends). | Refuse a row that does not name every key |
| A45 | `kpi/` (the ruler) | Ruler M0.8 freezes a test for A44 (a partly given row: what the box runs equals what `describe()` shows, both backends) before the fix lands. | As A27. | `git revert` the M0.8 commit |

### M4 (added by A5)

The consumer's library work (A16-A26) lands before M4, on top of the M0.3 ruler (A27), and the controllers lane re-ports onto it before the `target` stage (M3′).

| Step | Who | Lands | Exit (green on the scoreboard) |
|---|---|---|---|
| M4 | compaction (lanes core and cpp, one at a time) | fewer first-party lines in core, `inproc` and the C++ runtime, behaviour unchanged | every gate green; criterion 7 rises; no other criterion falls by more than its noise band; the consumer's tests (`moteus_controller_node/lk/tests`) still pass |
