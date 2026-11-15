# P2 · Explicit diagram (Drake + RobMoSys): like_keras v0 proposal

*2026-11-15 · one of five proposals · evolves `~/bam_ws/src/core/lk_dora_rs` · self-score ≈ 75/100, no gate failed by construction*

## 1. Thesis and concepts

1. A **Component** is a black box. It declares typed `In`/`Out` ports and one `forward`, and it never names a peer, transport, thread or host. That is the RobMoSys *component supplier* role.
2. A **System** wires components from outside (`connect`/`expose`, as Drake's `DiagramBuilder` does). It checks the whole graph before anything runs: types, schema hashes, one writer per input, and no cycle without a `lagged` input. That is the *system builder* role.
3. A **Deployment** cuts the checked diagram into threads, processes and hosts.
   - Inside a cut, edges are direct reads in an order the build derives, so the controller chain runs on one RT thread with no messages.
   - Across a cut, the Deployment inserts the transport endpoints. Unlike Drake's `LcmPublisherSystem`, a System never contains a transport.

**Every public noun: 8 at the top level, 29 in total.**

| Where | Names |
|---|---|
| Top level | `Component`, `In`, `Out`, `System`, `Deployment`, `queue`, `lagged`, `native` |
| Component | `forward`, `now`, `log`, `health` (+ its 4 levels), `params`/`Params`, `activate`, `deactivate` |
| System | `add`, `connect`, `expose`, `describe` |
| Deployment | `process`, `thread`, `record`, `run` |
| Backend | `like_keras.backends.dora.Dora` |
| C++ only | `get`, `take`, `write`, `LK_EXPORT` |

**[RobMoSys roles](https://robmosys.eu/wiki/general_principles:ecosystem:roles) → files:** service designer → `robot_msgs`; component supplier → component files; system builder → `cell()`; deployer → the deployment functions, the only place that names dora.

## 2. Repo architecture

```
like_keras/
  pyproject.toml   deps = ["robot_msgs"]; extras: dora = ["dora-rs==0.5.0", "pyarrow"]
  src/like_keras/                                                    core ≈ 690
    __init__.py 15 · ports.py 90 · component.py 120 · system.py 200 (checks, topo order, describe)
    deployment.py 150 (cuts, placement checks) · codec.py 115 (dataclass → layout + schema hash)
  src/like_keras/backends/inproc.py   330   executor, pacing, logical stepping, boxes, ctypes for .so
  src/like_keras/backends/dora/       170   descriptor, launch, skew check, generic Python node
  cpp/backends/dora/transport.cpp     150   cxx-bridge transport (same backend)
  include/like_keras/ + cpp/src/      870   component.hpp, system, executor, boxes, C ABI, lk-node
  third_party/nanoarrow/                    vendored, not counted
  tools/gen_msgs.py → robot_msgs_lk/*.hpp   ≈ 250 generated, counted
```

**Size:** about 2,450 SLOC against a 2,500 target. lk_dora's `graph.py`/`deployment.py` shrink into `system.py`/`deployment.py`, `runtime.py` becomes the dora node, and the JSON codec is replaced.

**robot_msgs** is the only runtime dependency, imported lazily. Every port carries a robot_msgs `Msg`; like_keras owns the wire layout and schema hash, robot_msgs the vocabulary. robot_msgs gains `std.LogRecord`, `std.DiagnosticRecord` (vault `C - Diagnostics Record`) and `control.Claim`; like_keras defines no types.

## 3. Python API

A component is a class. `forward` arguments must match its `In` names, which is checked when the class is created (see the planner in §5).

**Port semantics.** The set is closed. The component supplier declares them, because they are facts about the box:

| Declaration | Meaning |
|---|---|
| (default) | Latest value |
| `queue(n)` | Every message. Drop oldest, count drops |
| `lagged()` | Read the previous tick's value. This breaks the ordering edge, so a feedback loop is legal only through a lagged port |

**Triggering belongs to the partition:** one with a rate (or a self-pacing component) steps every member each tick; one without steps when a `queue` input arrives.

```python
s = lk.System("cell")
p, c = s.add(PickPlanner(), "planner"), s.add(Ctrl(), "arm")
s.connect(p.trajectory, c.trajectory)
s.connect(c.state, p.state)
s.expose(p.target)                                                      # boundary port: attach point

lk.Deployment(s).run()                                                  # one process, one thread
lk.Deployment(s).thread(s.arm, rate_hz=100).run()                       # arm on its own thread
lk.Deployment(s, Dora("0.5.0")).process(s.arm, "arm-controller").run()  # separate process
lk.Deployment(s, Dora("0.5.0")).process(s.arm, "arm-controller", host="rack-pc").run()  # machine
lk.Deployment(s).run(steps=1000)                                        # logical clock, no sleeps
```

**Facilities.** Each is a real port, auto-wired to a per-process sink that the backend exports. None needs user code.

| Facility | What it does |
|---|---|
| metrics | The executor records step count, step time p50/p99/max, overruns, input age and drops |
| health | UNKNOWN until set. Goes STALE after missed periods. `self.health(WARN, msg)` is optional |
| log | `self.log(msg)` lines carry the tick, wall time and component path |
| params | A nested frozen `Params` dataclass, read as `self.params`. Changes arrive on an implicit input and apply between steps, only while inactive |
| lifecycle | `activate`/`deactivate`, driven from outside. Downstream components activate first; an ERROR deactivates the chain |
| clock | `self.now()` is the logical tick time, owned by the Deployment |
| record | `Deployment.record(path)` taps every edge that crosses a cut. Replay is a source with the same ports |
| introspection | `describe()` returns JSON. `lk ls` and `lk top` read it |

## 4. C++ side

**Components include one header, `like_keras/component.hpp`.**
- Ports are members, as in OROCOS.
- `Out<T>` holds preallocated storage, written once per tick.
- A same-thread `In<T>` is a pointer into the upstream `Out`, the way Drake's output cache works.
- `get()` returns the latest value. `take()` returns new data or `nullptr`.

**Message types** are generated from the robot_msgs dataclasses: fixed-capacity arrays (≤ 16 joints) for RT outputs, spans over an I/O-owned buffer for variable-size inputs such as `JointTrajectory`. Values cross threads through `LatestBox` (triple buffer, one atomic exchange) or `Spsc<T,N>`; retired buffers go back to the I/O thread through a graveyard queue, so the RT thread never frees memory.

**The executor thread** applies `SCHED_FIFO`, CPU affinity and `mlockall`, and reports them in `health`. It paces with `clock_nanosleep(TIMER_ABSTIME)` and never catches up after an overrun ([ros2_control #736](https://github.com/ros-controls/ros2_control/issues/736)). A blocking component, such as the bus exchange, can set the pace instead.

**One `.so` runs in three places.** `LK_EXPORT(factory)` emits a C ABI (`lk_describe`, `lk_create`, `lk_start`, `lk_push`, `lk_pull`, `lk_stop`), loaded by:
- `lk.native()` through ctypes. The C++ thread never calls Python, so it never takes the GIL.
- `lk-node`, a generic dora node built once.
- The twin, through a params switch.

A Python diagram whose C++ leaves fall in one cut compiles into the same `lk::System` through `lk_create`.

**The transport seam** is non-RT and has 4 virtuals:

```cpp
struct Transport {
  virtual void open(const NodeSpec&) = 0;                    // ports, schema hashes, versions
  virtual bool recv(Incoming&, int timeout_ms) = 0;          // port index + owned ArrowArray
  virtual void send(int port, ArrowArray*, uint64_t schema) = 0;
  virtual void close() = 0;
};
```

**The dora transport today** uses dora's official cxx API at v0.5.0, not the C ABI that `cpp_scale` uses: `init_dora_node`, `next_event`, `event_as_arrow_input_with_info`, `send_arrow_output`, with nanoarrow for the Arrow side. Another transport is another `.cpp`.

**Boilerplate per component:** 1 `.cpp` plus `lk_add_component(arm_controller arm_controller.cpp)`.

## 5. The consumer case, end to end

**File 1: `bam_pick/planner.py`.** It declares its outputs and nothing else.

```python
import dataclasses
import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.manipulation import GraspPose
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory
from bam_pick.plan import plan_pick

class PickPlanner(lk.Component):
    target: lk.In[GraspPose] = lk.queue(8)     # forward runs once per target
    state: lk.In[ServoState]                   # latest; sampled, never waited on
    trajectory: lk.Out[JointTrajectory]

    def forward(self, target, state):
        plan = plan_pick(target, start=state.joint_state())                      # seconds of numpy
        return dataclasses.replace(plan, header=Header(stamp=self.now() + 0.05))  # absolute start
```

**Why the send is non-blocking.** `forward` returns, and the executor hands the value to the edge: `lk_push` (about 7 µs including a 56 KB copy) or `send_output`. The planner then goes on to the next target. It contains no send call and no topic.

**File 2: `arm_controller/arm_controller.cpp`.** One process, one RT thread.

```cpp
#include <like_keras/component.hpp>
#include <robot_msgs_lk/msgs.hpp>          // generated
#include "moteus_bus.hpp"                  // mjbots lib/cpp over fdcanusb: exclusive lock, take_over
#include "jtc_core.hpp"                    // pure follower: replace, sample, hold the end with qd = 0
namespace rm = robot_msgs;

struct MoteusBus : lk::Component {
  lk::In<rm::ServoCommand> command{this, "command", lk::lagged()};   // goes out in the next exchange
  lk::Out<rm::ServoState>  state{this, "state"};
  void activate() override   { bus_.open_exclusive(params.str("adapter")); bus_.take_over(); }  // R1 R2 T7
  void forward() override    { state.write(bus_.exchange(command.get())); }   // ~3.5 ms, paces the tick
  void deactivate() override { bus_.stop_feeding(); }                          // boards: timeout mode
  MoteusBusCore bus_;
};

struct Jtc : lk::Component {
  lk::In<rm::JointTrajectory> trajectory{this, "trajectory", lk::queue(2)};
  lk::In<rm::ServoState>      state{this, "state"};
  lk::Out<rm::ServoCommand>   command{this, "command"};
  void forward() override {
    if (auto* t = trajectory.take()) core_.replace(*t, state.get());   // start = t->header.stamp
    command.write(core_.sample(now(), state.get()));
    if (core_.holding()) health(lk::Warn, "trajectory ended: holding");
  }
  JtcCore core_;
};

struct Guard : lk::Component { /* proposed, state, claim -> safe: fault, runaway, PD+FF only (R6) */ };

lk::System arm_controller(const lk::Params& p) {
  lk::System s{"arm_controller"};
  auto& bus = s.add<MoteusBus>("bus", p["bus"]);
  auto& jtc = s.add<Jtc>("jtc", p["jtc"]);
  auto& guard = s.add<Guard>("guard", p["guard"]);
  s.connect(bus.state, jtc.state);  s.connect(bus.state, guard.state);
  s.connect(jtc.command, guard.proposed);
  s.connect(guard.safe, bus.command);            // legal cycle: bus.command is lagged
  s.expose(jtc.trajectory); s.expose(guard.claim); s.expose(bus.state); s.expose(guard.safe, "command");
  return s;                                      // derived order: bus -> jtc -> guard
}
LK_EXPORT(arm_controller)
```

**The order is derived, not authored.** The consumer spec's `tick_order(...)` goes away. moteus sends the command and the query in one frame, so the lag is a fact about the bus box, and the build turns it into the order.

**File 3: `bam_pick/cell.py`.** System builder and deployer.

```python
import like_keras as lk
from like_keras.backends.dora import Dora
from bam_pick.planner import PickPlanner
from bam_rack_twin.nodes import ArmControllerPy    # new wrapper over today's ArmPlant, same ports

SO, CFG = "build/libarm_controller.so", "records/bam-fb-arm/arm_controller.toml"

def cell(arm):
    s = lk.System("pick_cell")
    planner, arm = s.add(PickPlanner(), "planner"), s.add(arm, "arm")
    s.connect(planner.trajectory, arm.trajectory)
    s.connect(arm.state, planner.state)
    s.expose(planner.target)
    return s

def fallback():
    c = cell(ArmControllerPy.from_config(CFG))
    return lk.Deployment(c).thread(c.arm, rate_hz=50)

def same_box():
    c = cell(lk.native(SO, CFG))
    return lk.Deployment(c).thread(c.arm, rate_hz=100, cpus=[20], fifo=80)

def target():
    c = cell(lk.native(SO, CFG))
    return lk.Deployment(c, Dora("0.5.0")).process(c.arm, "arm-controller").thread(c.arm, rate_hz=100, cpus=[20], fifo=80)
```

**The build refuses** `fifo` on a Python partition (GIL priority inversion), core 22 in a CPU set, and a second writer on `arm.trajectory`. `arm_hold`, `pick_loop_v1` and `fivebar_run` stay untouched.

**T4** (dora daemon stopped): the RT thread never touches dora, so the trajectory finishes and holds, and `health` goes STALE as the input ages. **T6** (node killed): `watchdog_timeout=0.25` puts the boards in timeout mode; on restart `activate()` runs `take_over`.

## 6. Backend independence

**dora 0.5 → 1.x** ([migration guide](https://github.com/dora-rs/dora/blob/main/docs/migration-from-0.x.md)): `_unstable_deploy` → `deploy` is one line in `backends/dora/__init__.py`; the Python event shape lands in `node.py`; cxx renames (including the removal of `DataSampleHandle`) in `transport.cpp`; the Cargo pin in CMake; and the user's deployment changes `Dora("0.5.0")` → `Dora("1.1.0")`. Components and Systems: 0 lines. `grep -ril dora src include --exclude-dir=dora` finds nothing.

**Version skew.** At launch, the backend checks the CLI version, each daemon's version and the version each node linked against the pin. On a mismatch it refuses within 2 s and names both versions.

**dora → iceoryx2 (same host) or zenoh (LAN).** Each is one new `backends/<name>/`, made of the `Transport` plus a Python launcher.

**What still leaks:**
- **Supervision.** iceoryx2 is only a transport, so it needs about 120 LOC of launcher, or systemd.
- **The `latest` policy.** dora's `queue_size: 1` had a stale-slot bug (PR #652), so the receiver emulates `latest` itself.
- **Late join.** In dora 0.5, a dynamic node that joins a running multi-machine dataflow is never answered. Attach therefore goes through a gateway node (the OBSBOT pattern).
- **Versions.** Every host must run the same dora version.

## 7. Overhead versus raw dora

All costs are estimates, still to be measured.

| Path | Added cost | Extra copies | RT allocations |
|---|---|---|---|
| C++ edge, same thread | pointer read + virtual call, ≈ 2–5 ns | 0 | 0 |
| C++ I/O → RT thread | triple-buffer exchange, ≈ 20–50 ns | 0 | 0 |
| Python edge, same thread | precompiled slot loop, ≈ 0.3–0.6 µs | 0 | n/a |
| Python → C++, in process | ctypes ≈ 1.5 µs + memcpy of 7×1000 f64 ≈ 5 µs | 1 (budget ≤ 2) | 0 |
| C++ → dora | nanoarrow view of the `Out` buffers, < 1 µs | 0 | 0 |
| dora → C++ | hash compare + view, < 1 µs | 0 (pooled copy if dora's sample limits bite) | 0 |
| Python → dora | `StructArray` over numpy buffers, ≈ 5–15 µs | 0 | n/a |
| dora → Python | `to_numpy(zero_copy_only=True)` + robot_msgs validation, ≈ 10–30 µs | 0 | n/a |
| `import like_keras` | ≈ 5 ms (robot_msgs and numpy load lazily) | n/a | n/a |

**The weak spot is decoding in Python.** The fix is a trusted constructor in robot_msgs that skips validation when the schema hash matches.

## 8. Self-score, risks, milestones

| # (weight) | Score /10 | Why |
|---|---|---|
| 1 Consumer fit (15) | 8 | All 3 placements share one set of files, and T4, T6 and T7 hold by construction. T8 needs `Claim`, and T5 re-attach comes in milestone 3. |
| 2 Speed (13) | 7 | C++ edges cost nanoseconds. Python decode over dora risks the 10 µs budget. |
| 3 Black-box (11) | 9 | No API can name a peer. The only ambient read is `now()`. |
| 4 Succinctness (11) | 7 | 8 top-level names and 29 in total, at most 4 params. Annotations cost about 1.3× plain Python. |
| 5 Swap cost (9) | 7 | No component or System lines change. Python↔C++ is 1 line. In-process → separate process is 2 Deployment lines. |
| 6 Dora insulation (8) | 8 | dora is named only in `backends/dora/`, and the wire format is like_keras's own. Supervision leaks. |
| 7 Compactness (8) | 6 | About 2,450 SLOC, right at the limit. Attach and recording push past it. |
| 8 Python + C++ reach (7) | 8 | One `.so` runs in 3 hosts with a generated codec. Each component is 1 file plus 1 CMake line. |
| 9 Facilities (6) | 7 | All 7 are ports and need no user code. Needs 3 new robot_msgs types. |
| 10 Testability (5) | 9 | Logical `run(steps=)`, twin through params, replay as a source, and `forward` callable directly. |
| 11 Network attach (4) | 5 | Designed (exported ports, `process(..., attach=True)`, a gateway) but scheduled late. |
| 12 Wiring checks (3) | 9 | A Drake-style build checks types, hashes, cycles without a lag, single writers, dangling inputs, placement, FIFO on Python, and core 22. |

**Weighted total:** Σ w·s/10 ≈ **75**.

**Risks:**
1. **Equivalence across a cut.** Delivery delay changes what a `latest` read sees, so the equivalence gate needs a lockstep replay mode per backend.
2. **One writer per input** clashes with several attached clients; arbitration lives only at exported ports, via `claim`.
3. **C++ and Python Systems can drift;** both build through `lk_create`.
4. **Explicit wiring gets verbose;** auto-wiring beyond facility ports would erode the black-box rule.
5. **A stale `.so`** is refused at build: `lk_describe` carries the ABI version and schema hashes.

**Milestones:**
1. **Python core + in-process backend** (the contract barrier, lands alone): ports, build checks, cuts, logical stepping, codec hash; lk_dora's tests ported; `kpi/miswire` and `kpi/validgraphs` pass; the all-Python `arm_cell` identical over 10 twin runs.
2. **C++ runtime + `same_box`:** `MoteusBus` ports `arm_hold.py`'s `take_over`-then-hold onto a 3.5 ms fake bus; 0 allocations in 100k ticks; ≤ 100 ns per edge.
3. **dora backend + `target`:** Python node and cxx transport, skew refusal, `kpi/conformance` on both backends, T4 and T6; then the gateway and `lk ls`.
