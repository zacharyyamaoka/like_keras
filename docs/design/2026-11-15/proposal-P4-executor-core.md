# P4: real-time executor core (like_keras v0 proposal)

*2026-11-15. Angle: a copper-rs/OROCOS-style C++ executor is the heart; Python and the network attach at a System's boundary.*

## 1. Thesis and concepts

**Thesis.**
- **A C++20 executor of about 1,100 lines is the core.** It runs a fixed-order chain of black-box components on one thread over preallocated port slots: ros2_control's read → update → write, and copper-rs's "zero-allocation hot path". Its edge has three lock-free primitives: a latest box, an SPSC ring and a deferred-free ring.
- **Everything else attaches only at a System's exposed ports, through one `Edge` interface.** That includes Python components, dora, iceoryx2 and the IP-camera gateway. The hot path never names a transport.
- **Python authors every System and Deployment, and is a first-class component language.** Its components run on core-owned threads that take the GIL only inside `forward`, so one `.so` runs as a dora node, as a GIL-free thread beside the planner, or on a twin bus.

**Concepts.**
- **Top-level Python names (8).**
  - `Component`: ports, `Params` and `forward`.
  - `In[T]` and `Out[T]`: ports carrying robot_msgs types.
  - `latest` and `queue(n)`: policies. A port declares a default, and a wire may override it.
  - `System`: pure wiring, and itself a component.
  - `Deployment`: threads, processes, hosts.
  - `native`: a `.so` used as a component.
- **Also public:** `Ctx` (the facility handle, `self.ctx`) and `Edge` (backend authors only).

## 2. Repo architecture

```
/home/bam/like_keras/
  include/like_keras/lk.hpp  Component, In, Out, Ctx, System, LK_EXPORT*        ~280
  include/like_keras/rt.hpp  LatestBox, SpscRing, Graveyard, LogRing            ~160
  src/cpp/executor.cpp       order, bind slots, pace, overruns, stats           ~200
  src/cpp/abi.cpp            component .so C ABI (lk_abi, lk_describe, lk_create) ~110
  src/cpp/host.cpp           lk-host: runs a plan with no Python                 ~60
  src/cpp/arrow.cpp          robot_msgs view <-> Arrow C Data, schema hash       ~110
  src/cpp/python.cpp         nanobind: in-process native, PyComponent, boxes     ~180
  src/like_keras/            component 150, system 170, deployment 160,
                             codec 60 (no pyarrow), cli 100, __init__ 15
  backends/dora/             compile.py (plan -> dataflow.yml) 150, edge.cpp 150
  backends/gateway/edge.cpp  serves exposed ports on unix/TCP                    ~200
```

**Size.** The total is about **2,255** lines: the Python core is 555 and the C++ runtime 1,100, against a target of 900.

**Dependencies.**
- **Runtime:** only `robot_msgs`, plus the numpy it already brings.
- **Build:** nanobind and scikit-build-core.
- **`like_keras[dora]`:** the `dora-rs-cli` binary, plus `dora-node-api-cxx =0.5.0`. cargo builds that as a staticlib, which links only into `like_keras_dora/_edge.so`.
- **What import loads:** `import like_keras` loads none of `_lk`, dora, pyarrow or yaml.

**robot_msgs** stays the source of truth. Its own `python -m robot_msgs.cpp_gen` emits, per type, a header, a canonical Arrow schema and a 64-bit hash (about 600 lines for 6 types). The C++ form depends on the message:
- **Per-joint messages** (`ServoState`, `ServoCommand`) become values. They use `lk::Joints<double>` (`std::array<double,16>` plus a count), with the names interned at `configure`.
- **Sample-major messages** (`JointTrajectory`) become immutable `lk::Ref` views over Arrow buffers.

## 3. Python API

**A component**, the Python peer of the C++ follower (used by the fallback and in twin tests):

```python
import like_keras as lk

class Jtc(lk.Component):
    trajectory: lk.In[JointTrajectory] = lk.latest()
    state: lk.In[ServoState] = lk.latest(stale=0.05)   # None when stale; health: input_stale
    command: lk.Out[ServoCommand]

    @dataclass(frozen=True)
    class Params:
        hold_kd: float = 1.5

    def forward(self, trajectory, state) -> ServoCommand:
        return follower.sample(self.params, trajectory, state, self.ctx.now())
```

**A System, wired from outside:**

```python
def cell(arm) -> lk.System:                     # arm: any box with these ports
    s = lk.System("pick_cell")
    planner, ctl = s.add(PickPlanner()), s.add(arm)
    s.wire(planner.trajectory, ctl.trajectory)  # optional 3rd arg overrides the policy
    s.wire(ctl.state, planner.state)
    s.expose(planner.target, ctl.state, ctl.status, ctl.claim)
    return s
```

**Deployments.** Each placement is one line:

```python
one_process = lk.Deployment(cell(arm))
rt_thread   = lk.Deployment(cell(arm)).thread(arm, cpus=[20], fifo=80, period=0.01)
own_process = lk.Deployment(cell(arm)).process(arm, via=Dora("0.5.0"), cpus=[20], fifo=80)
two_hosts   = lk.Deployment(cell(arm)).process(arm, via=Dora("0.5.0"), host="arm-pc")
```

The other methods are `serve(address)`, `record(*ports, path)`, `check()`, `run()`, `step(seconds)` and `describe()`. No signature takes more than 6 parameters, and none takes `**kwargs` or a string enum. `Dora("0.5.0")` pins the dora version, which the skew check then enforces.

**Semantics.** These are normative, and the conformance suite runs them on every backend.
- **Groups.** A thread group runs in topological order every `period`, or is paced by a component that blocks (the bus exchange).
- **Cycles.** A cycle needs `s.order(...)`. Any edge that runs backwards in that order reads the previous tick, and shows as `delay=1`.
- **Firing.** A `queue` input fires its component once per message. Everything else fires every tick.
- **`check()` rejects:**
  - Python in a `fifo>0` group;
  - core 22.
- **`step(seconds)`** runs every group on one thread on a logical clock: deterministic and faster than real time.

**Facilities.** Each is a port, and none needs user code.
- **`log`:** tick, wall time and path.
- **`metrics`:** step count, step time p50/p99/max, overruns, input age, drops.
- **`health`:** UNKNOWN to STALE plus `expected_period_s`. The executor sets STALE and `input_stale`.
- **`params`:** a frozen dataclass (a C++ aggregate) that round-trips through `from_config`.
- **`lifecycle`:** an input that drives `configure`/`activate`/`deactivate`. After a restart a component is inactive.
- **Clock:** `ctx.now()`, epoch time on hardware and logical under `step`.
- **Recording:** `Deployment.record(...)` taps a wire; replay is a source with the same ports.
- **Introspection:** `describe()` returns JSON (ports, types, hashes, policies, placement), which `lk ls` and `lk top` read.

## 4. The C++ side

**`like_keras/lk.hpp`** (abridged):

```cpp
namespace lk {
using Time = std::int64_t;                                   // ns, deployment clock domain
struct Ctx {
  Time now() const;  std::int64_t tick() const;
  void log(Level, const char* static_fmt, double a = 0, double b = 0);   // -> LogRing
  void metric(int id, double v);  void warn(const char*);  void error(const char*);
};
class Component {
 public:
  virtual ~Component() = default;
  virtual void configure(Ctx&) {}      // allocate, open devices; a throw exits before any frame
  virtual void activate(Ctx&) {}
  virtual void forward(Ctx&) = 0;      // RT: no alloc, no lock
  virtual void deactivate(Ctx&) {}
};
template <class T> struct In {
  In(Component*, const char* name, Policy = latest, double stale_s = 0);
  const T* get() const;  const T& operator*() const;  bool fresh() const;
};
template <class T> struct Out { Out(Component*, const char* name); T& write(); };
}
#define LK_EXPORT(Class, Params, ...)
#define LK_EXPORT_SYSTEM(fn, Params)
```

**A C++ component** takes 2 files and 5 lines outside the body:

```cpp
class Jtc : public lk::Component {
 public:
  lk::In<robot_msgs::JointTrajectory> trajectory{this, "trajectory"};
  lk::In<robot_msgs::ServoState> state{this, "state", lk::latest, 0.05};
  lk::Out<robot_msgs::ServoCommand> command{this, "command"};
  explicit Jtc(const JtcParams& p) : f_(p) {}
  void forward(lk::Ctx& ctx) override {
    if (trajectory.fresh()) f_.replace(*trajectory, state.get());   // Ref swap
    f_.sample(ctx.now(), state.get(), command.write());
  }
 private: Follower f_;
};
LK_EXPORT(Jtc, JtcParams, hold_kd)
```

**How a chain runs in one thread.**
- **`check()`** sorts the group (or takes its `order`). It points each `In` at its writer's `Out` slot, or at a box slot when the writer is across a boundary. It then sizes the slots in `configure`, pre-faults the stack and calls `mlockall`.
- **The loop:**

```
wait_until(next) | paced -> inbound.take_all()     latest: pointer swap; queue: SPSC pop
for c in order: c->forward(ctx)                    edge = pointer deref + seq compare
outbound.publish()       -> exported Outs into ring slots; old Refs -> Graveyard
stats.add(dt)            -> fixed-bin histograms; overrun skips ahead, never bursts
```

- **The service thread.** Each process has one non-RT service thread. It runs every Edge's I/O and formats the LogRing (NanoLog-style). It publishes `health` at 1 Hz and `metrics`, and frees the Graveyard. The RT thread never waits on it.

**dora today.** `backends/dora/edge.cpp` implements `lk::Edge` on dora's cxx API at v0.5.0 (`apis/c++/node/src/lib.rs`):
- **Input.** The service thread blocks in `next_event`, which has no timeout, until a Stop event. Each input goes through `event_as_arrow_input_with_info`, then the codec view, then the box.
- **Output.** `send_arrow_output`, with an `ArrowSchema` built once per port.
- **No operators**, ever.
- **Dataflow.** `compile.py` (trimmed from lk_dora's `dora_backend.py`) emits `dataflow.yml` with nodes of the form `lk-host --plan arm.plan.json`. That argument replaces the `LK_DORA_INPUT_PLAN` env var.

**Another transport** is one more `Edge`, about 150 lines.
- **iceoryx2** sends with `loan_slice_uninit`, then the codec writes into the loan, then `send`. The reader wraps the sample as an `ArrowArray` whose release callback drops it, so there are 0 copies.
- **zenoh** uses `put` and a subscriber.

## 5. The consumer case end to end

**`pick_planner.py`.** The black box declares only its outputs:

```python
class PickPlanner(lk.Component):
    state: lk.In[ServoState] = lk.latest()        # sampled, never waited on
    target: lk.In[PickTarget] = lk.queue(8)       # fires once per target
    trajectory: lk.Out[JointTrajectory]

    def forward(self, target, state) -> JointTrajectory:
        plan = plan_pick(target, start=state.joint_state())   # seconds of numpy
        return plan.starting_at(self.ctx.now() + 0.05)        # absolute start; return = send
```

**`arm_controller.cpp`.** One thread:

```cpp
lk::System arm_controller(const ArmConfig& c) {
  lk::System s{"arm_controller"};
  auto& bus = c.twin ? s.add<MoteusBusTwin>("bus", c.bus) : s.add<MoteusBus>("bus", c.bus);
  auto& jtc = s.add<Jtc>("jtc", c.jtc);
  auto& td4 = s.add<Td4Wrist>("td4", c.td4);
  auto& guard = s.add<Guard>("guard", c.guard);    // fault, runaway, tilt, W2, claim lease
  s.wire(bus.state, jtc.state);  s.wire(bus.state, td4.state);  s.wire(bus.state, guard.state);
  s.wire(jtc.command, td4.fivebar);
  s.wire(td4.command, guard.proposed);
  s.wire(guard.safe, bus.command);
  s.order(bus, jtc, td4, guard);    // guard.safe -> bus is delay=1: one query+command frame/tick
  s.expose(jtc.trajectory, jtc.sequence, guard.claim, bus.state, guard.safe, jtc.status);
  return s;
}
LK_EXPORT_SYSTEM(arm_controller, ArmConfig)
```

**`cell.py`.** Three placements, with both files above unchanged:

```python
arm_cpp = lk.native("build/libarm_controller.so", config="records/bam-fb-arm/arm_controller.toml")
arm_py  = arm_controller_py(ArmConfig.load("records/bam-fb-arm/arm_controller.toml"))  # same ports

target   = lk.Deployment(cell(arm_cpp)).process(arm_cpp, via=Dora("0.5.0"), cpus=[20], fifo=80).serve("unix:arm")
same_box = lk.Deployment(cell(arm_cpp)).thread(arm_cpp, cpus=[20], fifo=80)
fallback = lk.Deployment(cell(arm_py))
```

**What happens.** `forward` returns and the planner continues at once.
- **`same_box`:** the numpy buffers are wrapped as an `ArrowArray` with no copy, and the pointer goes into the latest box.
- **`target`:** `send_arrow_output` runs inline, with one copy into dora's sample, as raw dora does.
- **The RT thread** swaps the pointer in at its next bus-paced tick (`tick_s` = 0.02). State returns through the outbound ring.
- **`arm_py`** is a Python System in `bam_rack_twin` that wraps today's `ArmPlant`. The original scripts are untouched.
- **As a systemd unit (R2),** the plan runs with `via=None`, and planners attach through the gateway.

## 6. Backend independence

**dora 0.5 → 1.x.** Only `backends/dora/` changes:
- `_unstable_deploy` becomes `deploy`;
- the crate pin moves;
- the Python floor becomes 3.11.

Components and Systems change by 0 lines and the Deployment by 1 (`Dora("1.1.0")`). like_keras owns the wire: the Arrow C struct layout plus the schema hash.

**Skew.** Every edge sends an `lk_hello` on a reserved output that the generated dataflow wires to every peer. The hello carries the like_keras version, the backend and its version, and the per-port hashes. Data is held until the hello verifies. A mismatch, or no hello within 2 s, exits non-zero naming both versions. `init_dora_node` runs under a 2 s watchdog, so it never hangs.

**dora → iceoryx2 or zenoh.** This is a new `backends/<x>/`, plus a launcher that emits systemd units.

**What leaks:**
1. **Supervision.** dora's `restart_policy` differs from systemd's `Restart=`, so v0 exposes neither.
2. **Latency.** dora's 4 KiB shared-memory threshold shapes the profile.
3. **Late join.** dora 0.5 cannot do it, so attach always uses the lk gateway.
4. **Rebuilds.** Each dora version needs a cargo rebuild.
5. **`latest`.** The receiver overwrites its own box instead of using dora's `queue_size: 1`, which has a stale-slot bug (PR #652).

## 7. Overhead analysis

Estimates; milestone 1 measures them.

| Path | Raw | like_keras adds |
|---|---|---|
| C++ edge, same thread | member call | 0 copies or allocations; a pointer deref plus a seq compare, about 1–3 ns |
| C++ edge across threads, per-joint | — | 1 seqlock copy of about 0.6 KiB, about 50–150 ns |
| C++ edge across threads, trajectory | — | 0 copies; an atomic exchange plus a Graveyard push, about 30 ns |
| Python → Python, same group | call | about 0.3–0.6 µs |
| Python ↔ C++, same process | — | 0 copies, but building the `Msg` takes 3–10 µs: **misses** the 1 µs target |
| Python sender over dora | `send_output(pa.array)` | no pyarrow; the same 1 copy; 0 per-message metadata, because hashes travel in the hello |
| C++ RT sender over dora | inline send | an SPSC push (0.2 µs) plus a futex wake (5–30 µs at p99 on the generic kernel): **can miss** the 10 µs target. Mitigation: a 50 µs post-tick spin, or accept it as the price of never blocking on a daemon that stalls for 4 s |

Allocations on the RT thread: 0, enforced by the gate. The service thread adds nothing beyond dora's own per-send `Box<Metadata>`. Import takes about 5 ms.

## 8. Self-score, risks, milestones

| # | Criterion (weight) | 1–5 | Why |
|---|---|---|---|
| 1 | Consumer fit (15) | 4 | T1–4, T6, T7 and T12 hold by construction. T5 in `same_box` dies with the planner. T8 needs fan-in on `queue` with source paths. |
| 2 | Speed (13) | 4 | ns edges and 0 RT allocations, but the RT→edge wake and the `Msg` build cost time. |
| 3 | Black-box (11) | 5 | No API can name a peer or a transport. |
| 4 | Succinctness (11) | 4 | 8 top-level names, about 33 in total. |
| 5 | Swap cost (9) | 5 | 1 Deployment line per swap, Python ↔ C++ included. |
| 6 | dora insulation (8) | 4 | One directory; like_keras owns the wire. The edge needs a cargo rebuild per dora version. |
| 7 | Compactness (8) | 3 | The C++ runtime is about 1,100 lines against 900. With generated messages the total is about 2,850. |
| 8 | Reach (7) | 4 | One `.so` in 3 placements, a bit-exact codec, plus a CMake line per component. |
| 9 | Facilities (6) | 4 | 7 of 7 as ports. Params need the `LK_EXPORT` field list. MCAP is v1. |
| 10 | Testability (5) | 5 | `step()` on a logical clock, replay through taps, the twin set by config. |
| 11 | Network attach (4) | 4 | The gateway plus `System.attach(address)`. LAN comes later. |
| 12 | Wiring checks (3) | 5 | Types, hashes, unordered cycles, double writers, placement, core 22. |

**Paper total ≈ 80/100. No gate fails by construction.**

**Risks:**
1. **A compiled core everywhere.** The Windows box, cloud VMs and pure-Python users all need a built wheel, against "use more vanilla python".
2. **Python is second-class in the scheduler,** never allowed in an RT group.
3. **Generic kernel.** Without PREEMPT_RT, a 1 ms max under a 24-core load is unproven.
4. **Codegen drift.** The hash catches it but does not prevent it.
5. **OROCOS-shaped scope.** The line-count ratchet is the only brake.

**Milestones:**
1. **C++ only.** `lk.hpp`, `rt.hpp`, the executor and `lk-host`. It must pass the allocation gate (an `operator new` counter over 100k ticks) and `bench_chain` at ≤ 100 ns per edge. It must also run `arm_cell`'s System on the fake bus with a 3.5 ms exchange and `arm_hold`'s take-over-then-hold.
2. **Python layer.** nanobind, `robot_msgs.cpp_gen` with golden bytes, `same_box` and `fallback`, and `step()`. It must pass T1–T4 in-process and the codec test.
3. **dora 0.5.0.** The edge and `compile.py`, the hello and skew check, `serve`/`attach`, and `lk ls`/`top`. Then `target`, T4–T8, T11 and T12, and conformance on both backends.
