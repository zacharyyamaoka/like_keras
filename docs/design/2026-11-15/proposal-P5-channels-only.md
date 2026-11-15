# P5: Channels only (two primitives, plain code, tiny transports)

*2026-11-15. Paper score ≈ 81/100 (§8).*

## 1. Thesis and concepts

A component is a black box: typed `In`/`Out` ports, constructor params, and a `step` that maps input values to output values, holding no handle to anything outside itself. Systems and deployments are plain code: a system is a Component whose `__init__` creates children and calls `connect`, and a deployment is a list of `Loop`s passed to `run` with one transport. A backend is only a transport (4 methods in Python, 3 in C++), and like_keras owns the wire format, so dora churns behind one file per language.

**Top-level names (8):** `Component`, `In`, `Out`, `connect`, `Loop`, `run`, `native` (a C++ `.so` as a component), and `remote` (a running node as a component).

**Other public names (≈22):**
- Hooks: `step`, `start`, `stop`.
- Reserved ports: `log`, `health`, `metrics`.
- The run handle: `advance`, `wait`, `describe`.
- Backends: `Transport` (`role`, `launch`, `attach`, `close`), `Dora`, `Zenoh`.
- C++ only: `fresh`, `value`, `send`, `lk::log`, `LK_FIELDS`, `LK_EXPORT`.

There is no System, Deployment, Graph, Context or registry class.

**Coined terms:** a **Loop** is one thread running its components in wire order, every `period` or on a trigger input. A **Frame** is our wire envelope: a binary header (schema hash, seq, stamp, source) plus raw arrays.

## 2. Repo architecture

```
/home/bam/like_keras/                               est. SLOC
  src/like_keras/
    __init__.py         15  the 8 names
    component.py       150  Component, In, Out, reserved ports, exports by attribute
    connect.py          55  connect(), type/schema checks, one writer per input
    run.py             190  Loop, run(): checks, wire order, threads, manual clock, metrics
    wire.py            130  Frame codec for frozen dataclasses, 64-bit schema hash
    native.py          100  ctypes over each .so's C ABI; `like_keras gen-cpp`
    cli.py              60  lk ls / lk top
    dora.py            240  [extra] yml from the plan, version pin, I/O thread, Arrow
    zenoh.py           200  [extra] keys lk/<process>/<port>, late join
  include/like_keras/   380  component.hpp, rt.hpp (LatestBox, SpscRing, LogRing), transport.hpp
  include/robot_msgs/   240  GENERATED span views of 6 controller types (counted)
  cpp/runtime.cpp       300  RT loop (SCHED_FIFO, affinity, mlockall), C ABI, plan reader
  cpp/lk_host.cpp        60  generic process: dlopen component .so + transport .so
  cpp/{dora,zenoh}_transport.cpp  180 each  [extras]
```

**Size:** Python core ≈ 700, C++ runtime ≈ 740 (+240 generated), dora ≈ 420 across both languages (over the 350 target), zenoh ≈ 380. Total ≈ 2,480.

**Dependencies:** `import like_keras` is stdlib-only. `robot-msgs` is the one runtime dependency, loaded at the first boundary Frame. Extras: `dora = [dora-rs==0.5.0, pyarrow]` and `zenoh`. The C++ is plain C++20.

**robot_msgs is the type vocabulary.** Any frozen dataclass is a port type, and robot_msgs `Msg`s get the struct-of-arrays codec. `like_keras gen-cpp robot_msgs.trajectory:JointTrajectory` emits the C++ view with the same schema hash, and golden bytes pin both languages.

## 3. Python API

```python
import logging
import like_keras as lk
from robot_msgs.control import ServoCommand, ServoState

class Hold(lk.Component):                          # leaf
    command = lk.In(ServoCommand)                  # sampled: latest or None
    state = lk.In(ServoState, trigger=True)        # each arrival runs step
    safe = lk.Out(ServoCommand)

    def __init__(self, max_age_s: float = 0.25):
        self.max_age_s = max_age_s

    def step(self, t, command, state):             # names checked against In ports
        if command is None or t - command.header.stamp > self.max_age_s:
            logging.getLogger(__name__).warning("stale; holding")   # → log port, health WARN
            return hold_at(state)
        return command                             # several Outs: a dict; None: send nothing

class Cell(lk.Component):                          # system: __init__ wires, no step
    def __init__(self, ctl: lk.Component):
        self.hold, self.ctl = Hold(), ctl          # paths: cell.hold, cell.ctl
        lk.connect(self.ctl.command, self.hold.command)        # queue=1: latest wins
        lk.connect(self.ctl.state, self.hold.state, queue=8)   # drop-oldest, counted
        self.safe = self.hold.safe                 # export = alias

cell = Cell(ctl)
lk.run(cell).wait()                                                        # one thread
lk.run(cell, lk.Loop(cell.ctl, period=0.01, cpus=(20,), fifo=80)).wait()   # own thread
lk.run(cell, lk.Loop(cell.ctl, period=0.01, process="ctl"), transport=Dora("0.5.0")).wait()
lk.run(cell, lk.Loop(cell.ctl, period=0.01, process="ctl", host="jetson"), transport=Dora("0.5.0")).wait()
with lk.run(cell, realtime=False) as r: r.advance(1.0)                     # logical clock
```

**How `run()` builds the tree (all checks happen before any thread starts):**
- Unplaced components use the calling thread. A Loop runs its components in wire order. A cycle needs one `delay=1` wire, Zach's rule that "feedback must pass through an initial value".
- Refused, naming `path.port` and the fix: a Loop mixing languages, `process=` without a transport, two writers on one input, a type or schema-hash mismatch, a queue the transport can't honour, `cpus` on the dora daemon's cores.
- Every Python process runs the same script, as [torchrun](https://pytorch.org/docs/stable/elastic/run.html) does. `transport.role()` names each copy's process.

**Facilities, with zero user code:**

| Facility | How |
|---|---|
| log | stdlib `logging`, credited to the component stepping on that thread (C++: a lock-free ring) |
| metrics | step time p50/p99/max, overruns, input age, drops; 1 Hz |
| health | UNKNOWN/OK/WARN/ERROR/STALE with `expected_period_s`. STALE comes from input age; WARN/ERROR from the log level |
| params | a dataclass, or a C++ struct with `LK_FIELDS`, shown in `describe()`; static in v0 |
| lifecycle | `start()`/`stop()`; a failed `start()` exits before any port opens |
| recording | stock `Recorder`/`Player` components; one `connect` per tap |
| introspection | `describe()` JSON (tree, ports, hashes, queues, loops, transport); `lk ls`, `lk top` |

## 4. The C++ side

```cpp
// <like_keras/component.hpp>: the whole author surface
namespace lk {
struct Component { virtual void start() {} virtual void step(double t) {} virtual void stop() {} };
template <class T> struct In  { const T* fresh(); const T& value() const; };  // new? / latest
template <class T> struct Out { T& value(); void send(); };                   // fill in place, publish
template <class T> void connect(Out<T>& src, In<T>& dst, int queue = 1, int delay = 0);
void log(Level, const char* fmt, ...);                                        // RT-safe
}
#define LK_FIELDS(...)           // names ports, children, params
#define LK_EXPORT(Type, Params)  // C ABI: one .so for lk_host, Python in-process, tests

struct Jtc : lk::Component {
  struct Params { int joints = 7; int max_samples = 2000; LK_FIELDS(joints, max_samples) };
  lk::In<JointTrajectory> trajectory;  lk::In<ServoState> state;
  lk::Out<ServoCommand> command;       lk::Out<ArmStatus> status;
  explicit Jtc(Params p) : p_(p) {}
  void start() override { follower_.reserve(p_.joints, p_.max_samples); command.value().reserve(p_.joints); }
  void step(double t) override {                     // no allocation, lock or transport
    if (const JointTrajectory* fresh = trajectory.fresh()) follower_.replace(*fresh, state.value());
    follower_.sample(t, command.value());            // past the end: hold, qd = 0
    command.send();
  }
  LK_FIELDS(trajectory, state, command, status)
  Params p_; Follower follower_;
};
```

**A chain inside one Loop:**
- `connect` stores a pointer; `fresh()` compares sequence numbers. No copy, no lock.
- `runtime.cpp` runs leaves in topological order, cut at the `delay=1` wire, under the plan's SCHED_FIFO, affinity and mlockall.

**At the boundary:**
- The transport's I/O thread fills a `LatestBox` (or an `SpscRing` when `queue>1`), and `fresh()` swaps a pointer.
- `send()` copies into a preallocated ring slot, which the I/O thread encodes and sends.
- The RT thread never calls the transport. This is [realtime_tools](https://control.ros.org/rolling/doc/realtime_tools/doc/index.html)' box/publisher split.

**On dora today:** `dora_transport.cpp` uses the [cxx-bridge API at v0.5.0](https://github.com/dora-rs/dora/blob/v0.5.0/apis/c++/node/src/lib.rs), not lk_dora's C ABI. Its I/O thread blocks in `next_event`, maps `event_as_arrow_input_with_info`'s struct children to spans with no copy (releasing the event when the RT side lets go), and sends with `allocate_data_sample` + `send_data_sample_with_metadata`.

**Another transport:** implement `transport.hpp` (`attach(plan, deliver)`, `send(key, frame)`, `release(token)`) in one file exporting `lk_transport_create`. [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) maps one to one (`loan_uninit` → `write_payload` → `send`); zenoh's is `Publisher::put`.

## 5. The consumer case end to end

**`planner.py`** declares only its outputs:

```python
import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.trajectory import JointTrajectory
from bam_pick.types import PickTarget

class PickPlanner(lk.Component):
    target = lk.In(PickTarget, trigger=True)    # one step per target
    state = lk.In(ServoState)                   # latest, never waited on
    trajectory = lk.Out(JointTrajectory)

    def __init__(self, lead_s: float = 0.05):
        self.lead_s = lead_s

    def step(self, t, target, state):
        if state is None:
            return None
        plan = plan_pick(target, start=state.joint_state())   # seconds of numpy
        return plan.starting_at(t + self.lead_s)              # absolute start; the runtime sends it
```

**`arm_controller.cpp`** runs the bus and the chain on one thread:

```cpp
#include <like_keras/component.hpp>
#include "moteus_bus.hpp"   // MoteusBus|FakeBus over mjbots' moteus C++; start(): BUS_LOCK, take_over
#include "jtc.hpp"
#include "guard.hpp"

struct ArmConfig { MoteusBus::Params bus; Jtc::Params jtc; Td4Wrist::Params td4; Guard::Params guard;
                   LK_FIELDS(bus, jtc, td4, guard) };

struct ArmController : lk::Component {
  MoteusBus bus; Jtc jtc; Td4Wrist td4; Guard guard;
  lk::In<JointTrajectory>& trajectory = jtc.trajectory;   // boundary = references to inner ports
  lk::In<Claim>&           claim      = guard.claim;
  lk::Out<ServoState>&     state      = bus.state;
  lk::Out<ServoCommand>&   command    = guard.safe;
  lk::Out<ArmStatus>&      status     = jtc.status;
  explicit ArmController(const ArmConfig& c) : bus(c.bus), jtc(c.jtc), td4(c.td4), guard(c.guard) {
    lk::connect(bus.state, jtc.state);
    lk::connect(bus.state, td4.state);
    lk::connect(bus.state, guard.state);
    lk::connect(jtc.command, td4.fivebar);
    lk::connect(td4.command, guard.proposed);
    lk::connect(guard.safe, bus.command, 1, /*delay=*/1);  // order: bus, jtc, td4, guard
  }
  LK_FIELDS(bus, jtc, td4, guard, trajectory, claim, state, command, status)
};
LK_EXPORT(ArmController, ArmConfig)
```

**`deploy.py`** holds the wiring and placement; each variant is one line away:

```python
import like_keras as lk
from like_keras.dora import Dora
from planner import PickPlanner
from bam_rack_twin.nodes import ArmControllerPy   # ArmPlant + robot_controllers JTC, same ports

class PickCell(lk.Component):
    def __init__(self, arm: lk.Component):
        self.planner, self.arm = PickPlanner(), arm
        lk.connect(self.planner.trajectory, self.arm.trajectory)
        lk.connect(self.arm.state, self.planner.state)
        self.target = self.planner.target

CFG = "records/bam-fb-arm/arm_controller.toml"
cell = PickCell(lk.native("build/libarm_controller.so", CFG))   # fallback: PickCell(ArmControllerPy.from_toml(CFG))
arm = lk.Loop(cell.arm, period=0.01, cpus=(20,), fifo=80, process="arm-controller")   # same_box: no process=
if __name__ == "__main__":
    lk.run(cell, arm, transport=Dora("0.5.0")).wait()           # same_box, fallback: transport=None
```

**Target:** the launcher writes a two-node `dataflow.yml` (`main` = `python deploy.py`, `arm-controller` = `lk_host` + plan) and runs `dora start --attach`. The planner's return value crosses dora shared memory into the C++ `LatestBox`, and `fresh()` takes it next tick. The RT thread runs exchange → jtc → td4 → guard, and `state` returns through the ring. Neither side waits on the other.

**Same box:** the same `.so` loads through ctypes; `lk_push` copies with the GIL released. **Fallback:** `ArmControllerPy` on a Python thread, no C++, no dora; bam_rack_twin's scripts are untouched.

**Tests:** T4 reports STALE `input_stale` while the chain ticks. T6/T7: the bus's `start()` locks and takes over before any port opens. T8: planners stamp `header.meta["claim"]`, and `Guard` compares (consumer logic).

## 6. Backend independence

| Change | Inside the backend | Lines elsewhere |
|---|---|---|
| dora 0.5 → 1.x | `dora.py`: `_unstable_deploy` becomes `deploy`, drop `custom:`. `dora_transport.cpp`: `DataSampleHandle` is gone ([migration](https://github.com/dora-rs/dora/blob/main/docs/migration-from-0.x.md)), so use `send_arrow_output`; rebuild | 1 (`Dora("1.1.0")`) |
| dora → zenoh | none | 1 |
| dora → iceoryx2 | about 300 new lines in two files | 1 |

**Insulated:** we own the Frame, so dora wire or Arrow changes stay in `dora.*`. Only dora nodes are used, never operators. `Dora(version)` is checked against every host's `dora --version` (2 s timeout) and each node's linked version, and a mismatch is refused naming both. `kpi/contract` runs in-process, on dora and on zenoh.

**Leaks:**
- **Launch:** dora's daemon spawns nodes; under zenoh, like_keras spawns local processes and remote ones are systemd units.
- **Late join:** dora 0.5 never answers a dynamic node joining a running multi-machine dataflow, so `remote()` needs zenoh.
- **Stalls:** a 4 s daemon stall shows only as input age, so the control loop never crosses a transport.
- **Restarts:** dora's `restart_policy` passes through; there is no like_keras supervisor.

## 7. Overhead compared with raw dora (estimates, measured in milestone 2)

| Path | Added per message | Extra copies | RT allocs |
|---|---|---|---|
| C++ → C++ in a Loop | ~1–3 ns per edge + one virtual `step` | 0 | 0 |
| Python → Python in-process | ~0.3–0.6 µs per edge | 0 | n/a |
| Python → dora, one buffer | ~2–3 µs to pack the header into metadata | 0 (`pa.py_buffer`) | n/a |
| Python → dora, multi-array ≥ 4 KiB | ~1–2 µs per field (`Array.from_buffers`) | 0 | n/a |
| Python → dora, multi-array < 4 KiB | < 0.5 µs gather | 1 (cheaper than Arrow objects) | n/a |
| dora → C++ RT | ~0.2 µs spans + an atomic swap | 0 | 0 |
| C++ RT → dora | a ~200 B ring push; encoding happens off the RT thread | 1 (RT handoff) | 0 |
| Python ↔ C++, same process | ~1 µs ctypes + memcpy (7×1000 trajectory ≈ 170 KB: ~5–20 µs) | 1 | 0 |

Typed Python receives pay robot_msgs' constructor checks (~10–30 µs), as any typed receiver would. Import: ≤ 5 ms (estimate).

## 8. Self-score, risks and milestones

| # | Criterion (weight) | 0–10 | Why |
|---|---|---|---|
| 1 | Consumer fit (15) | 8 | T1, T3 and T4 follow from the design. T5–T7 rest on dora restarts and the bus's `start()`. T8 is component logic. |
| 2 | Speed (13) | 8 | No JSON on the wire; in-thread edges cost ns. One gather below 4 KiB. dora Python send/receive threading is unverified. |
| 3 | Black box (11) | 10 | `step` maps values to values. No handle can name a peer. |
| 4 | Succinctness (11) | 8 | 8 top-level names, ≈ 30 total, widest signature 6. Port declarations cost ≈ 1.3× plain functions. |
| 5 | Swap cost (9) | 10 | One deployment line per swap; 0 component or system lines. |
| 6 | Dora insulation (8) | 8 | One backend, one file per language; we own the wire; skew is checked. Launch leaks. |
| 7 | Compactness (8) | 8 | ≈ 2,480 total, ≈ 700 core. The dora backend is over target. |
| 8 | Python + C++ (7) | 7 | One `.so` in 3 placements, with golden bytes. But a Loop holds one language, and port lists are written twice. |
| 9 | Facilities (6) | 5 | Log, metrics, health and introspection are free. Params are static, lifecycle is 2 hooks, and recording takes a `connect`. |
| 10 | Testability (5) | 9 | The manual clock drives Python and C++ (`lk_step`). A twin is one constructor argument. `Player` replays. |
| 11 | Network attach (4) | 5 | Works on zenoh only. |
| 12 | Wiring checks (3) | 8 | Broad build-time checks. An unplaced component is allowed. |
| | **Weighted total** | **≈ 81** | |

**Risks:**
1. dora 0.5's Python `Node` may refuse `send_output` while another thread is blocked in `next` (a PyO3 borrow). If so, sends wait on the I/O thread's timeout. Measure this first.
2. The same script runs in every process, so module-level side effects repeat, and every process must build an identical tree.
3. Facilities are thin and there is no supervisor. That is the price of compactness.
4. A box's Python and C++ port lists can drift. `describe()` equality is checked, not generated.
5. C++ capacity is fixed (`max_samples`). An oversized trajectory is refused on `status`.

**Milestones:**
1. **In-process spine:** Python core, C++ headers, `runtime.cpp`, generated headers with golden bytes. arm_cell `same_box` and `fallback` on a `FakeBus` (3.5 ms exchange). Pass: 0 allocations per 100k ticks, equivalence, import ≤ 20 ms, T1, T3, T9.
2. **dora transport:** `dora.py`, `dora_transport.cpp`, `lk_host`, skew refusal. Pass: T2, T4–T7, overhead bench vs raw dora (64 B / 4 KiB / 1 MiB / 7×1000). Then the controllers lane ports onto it and files friction notes.
3. **Zenoh and attach:** `zenoh.py`, `zenoh_transport.cpp`, `remote()`, `lk ls`/`lk top`. Pass: T8, T12, attach tests, conformance on two transports.
