# P3: like_keras as a spec-as-file compiler

*2026-11-15 · proposal 3 of 5 · angle: the wiring and the interfaces are a declarative file; like_keras compiles that file to backends.*

## 1. Thesis and concepts

**Thesis.**
1. Components are black boxes whose ports are typed by robot_msgs. Systems wire them from outside in plain Python. `lk build` lowers both into one deterministic spec file, `*.lk.json`, and every backend, generator, gateway, viewer and check reads that file and nothing else.
2. A backend is an emitter of that file:
   - in-process: a generated Python step function;
   - C++: a generated header per component, plus a statically scheduled chain in one `.so`;
   - dora: `dataflow.yml`;
   - zenoh or iceoryx2 later: new emitters, not new APIs.
3. Every placement compiles from one file, so equivalence, mis-wiring rejection, introspection and the "IP camera" device description share one artifact. The file is to like_keras what ONNX is to a Keras model.

**Prior art.**
- [Lingua Franca](https://www.lf-lang.org/docs/): `lfc` generates C, C++, Python, TypeScript and Rust programs, plus the communication fabric for federated runs.
- [copper-rs](https://github.com/copper-project/copper-rs): its graph is "wired at compile time".
- **The warning:** rosidl runs 10 generators per interface, and px4_msgs takes 72 s to build ([rosidlcpp](https://discourse.openrobotics.org/t/introducing-rosidlcpp-building-interface-packages-10x-faster/42748)). So there are two generators, ever: Python → spec, and spec → C++.

**Public nouns.**

| Where | Names |
|---|---|
| Python (8) | `Component`, `In`, `Out`, `latest`, `queue`, `System`, `Deployment`, `load` |
| The file | `*.lk.json` |
| C++ | `lk::In`, `lk::Out`, `lk::Ctx` |
| CLI | `lk build`, `lk run`, `lk ls`, `lk top` |

## 2. Repo architecture

```
like_keras/                                                     est. SLOC
├─ pyproject.toml        deps = ["robot_msgs"]; extra dora = ["dora-rs==0.5.0", "pyarrow"]
├─ src/like_keras/       core: stdlib only, robot_msgs imported lazily          ≈ 690
│  ├─ component.py       Component, In/Out, latest/queue, Params, facility ports  150
│  ├─ system.py          add / connect / expose                                    90
│  ├─ deployment.py      place / serve / record / run / compile                    90
│  ├─ spec.py            front end: → spec dict; robot_msgs → Arrow format +
│  │                     64-bit hash; ~20 checks naming path.port, fix, file:line 200
│  ├─ facilities.py      log / metrics / health records                           80
│  └─ cli.py, __init__   lk build | run | ls | top                                80
├─ src/like_keras/backends/
│  ├─ inproc/  executor.py (generated step fn, logical clock) 150, native.py (ctypes) 100,
│  │           arrow_c.py (robot_msgs ↔ Arrow C Data, zero-copy) 100              ≈ 350
│  ├─ dora/    emit.py (dataflow.yml, nodes only) 100, node.py 120, host.cpp 150  ≈ 370
│  ├─ gateway/ gateway.py: serve() = spec + ports on a socket (OBSBOT pattern)     110
│  └─ cpp/     gen.py: spec → headers + chain glue + C ABI                         180
├─ include/like_keras/lk.hpp, arrow_c.h   In/Out/Ctx, LatestBox, rings, C ABI      320
├─ cpp/rt.cpp            SCHED_FIFO, affinity, mlockall, pacing, overruns          140
└─ cmake/LikeKeras.cmake lk_add_component()                                        40
```

**Size.** About 2,350 SLOC, including about 150 generated per task (cap 2,500).

**robot_msgs is the type system.** `spec.py` walks each `Msg` dataclass and maps:
- `Array` to an Arrow list of float64, using the `dtype` that `sample()` already stores;
- `tuple[str, ...]` to a list of utf8;
- `Header` to a struct;
- `X | None` to a nullable field.

The hash is the first 64 bits of sha256 over `type_name` plus the format. This needs two small robot_msgs changes:
- `Annotated[Array, "i64"]` metadata, because `ServoState.fault` is int64 only inside `__post_init__`;
- a public twin of `_core.adopt`, for zero-copy receive.

## 3. The Python API

```python
import like_keras as lk

class PickPlanner(lk.Component):
    target: lk.In[PickTarget] = lk.queue(8)   # queue input: triggers forward(), one message per call
    state: lk.In[ServoState] = lk.latest()    # latest input: sampled, never waited on
    trajectory: lk.Out[JointTrajectory]

    @dataclass(frozen=True)
    class Params:
        lead_s: float = 0.05

    def forward(self, target: PickTarget, state: ServoState) -> JointTrajectory:
        plan = plan_pick(target, start=state.joint_state())
        self.log.info("planned", samples=len(plan))
        return plan.starting_at(self.clock.now() + self.params.lead_s)
```

**Rules.**
- A component with no queue input runs at its placement's rate.
- Several outputs come back as a tuple, and `None` sends nothing.
- A component has no API that names a peer, topic or transport.

```python
def cell(arm: lk.Component) -> lk.System:            # wiring lives outside every component
    s = lk.System("pick_cell")
    s.add(planner=PickPlanner(), ctl=arm)
    s.connect(s.planner.trajectory, s.ctl.trajectory)  # connect(src, *dsts, policy=None, delay=0)
    s.connect(s.ctl.state, s.planner.state)
    s.expose(target=s.planner.target)
    return s

c = cell(arm)
lk.Deployment(c).place(c.ctl, rate_hz=100)                                       # one process, own thread
lk.Deployment(c, transport="dora").place(c.ctl, process="arm", rate_hz=100)      # two processes
lk.Deployment(c, transport="dora").place(c.ctl, process="arm", machine="rack-pc", rate_hz=100)  # two machines
```

**`Deployment` rules.**
- Each `place(block, process=, machine=, cpus=, fifo=, rate_hz=)` call is one thread, and `process` and `machine` say where it lives.
- `transport` applies only to edges that cross a process.
- `run(steps=n)` steps a logical clock deterministically. `compile(dir)` writes the spec and backend files.

The spec is generated, and a content hash refuses hand edits. An excerpt:

```json
{"like_keras": 1, "transport": "dora==0.5.0",
 "types": {"control/ServoState": {"hash": "c41d9e07a2f3b815", "arrow": "+s{name:+l<u>,position:+l<g>,…}"}},
 "blocks": {"pick_cell.planner": {"impl": "pick_cell.planner:PickPlanner", "params": {"lead_s": 0.05}, "in": {…}}},
 "edges": [["pick_cell.planner.trajectory", "pick_cell.ctl.trajectory", {"policy": "latest", "at": "cell.py:5"}]],
 "places": [{"block": "pick_cell.ctl", "process": "arm", "fifo": 80, "rate_hz": 100}]}
```

**Standard facilities.** The compiler adds these to every block, so they cost zero user lines:

| Facility | What the block gets |
|---|---|
| log | The `log` port (`self.log` / `ctx.log`), with tick, wall time and path. |
| metrics | The `metrics` port at 1 Hz: steps, step time p50/p99/max, overruns, input age, drops. |
| health | The `health` port at 1 Hz: UNKNOWN first, STALE after missed periods. |
| params | `Params`, generated as a C++ struct, with values from a per-machine TOML. |
| lifecycle | The `lifecycle` input. The `on_*` hooks are optional, and a restart comes back inactive. |
| clock | `self.clock.now()`, the logical time. |
| recording | `dep.record("run.npz", *ports)` compiles a tap. Replay is a source with the same ports. |
| introspection | The spec file itself. |

## 4. The C++ side

`lk_add_component(arm_controller SYSTEM box.py:arm_controller SOURCES arm_controller.cpp)` runs `gen.py` at CMake configure. The author sees only the generated header:

```cpp
namespace lk::gen {
struct Jtc {
  lk::In<rm::JointTrajectory>   trajectory;  // take(): a view; its buffer is freed off the RT thread
  lk::In<rm::ServoState>        state;       // get(): a const& to the upstream output
  lk::Out<rm::ServoCommand>     command;     // loan(): a T& preallocated at configure
  lk::Out<rm::ControllerStatus> status;
  JtcParams params;
};}
```

**`lk.hpp` holds:**
- `In`, `Out` and `Ctx` (`now`, `log`, `metric`, `health`);
- `LatestBox`: an atomic pointer swap. Old pointers return through an SPSC "graveyard" ring, so frees never run on the RT thread;
- the C ABI:

```c
typedef struct lk_box_v1 {
  const char* spec_json;                                        /* the .so carries its own datasheet */
  void* (*create)(const uint8_t* params, size_t n);
  int  (*start)(void*, const lk_place*);                        /* spawns the RT thread */
  int  (*push)(void*, uint16_t in, struct ArrowArray*, struct ArrowSchema*);  /* never blocks */
  int  (*pull)(void*, uint16_t out, struct ArrowArray*, int timeout_ms);
  void (*stop)(void*);
} lk_box_v1;
```

**A chain runs in one thread.** `gen.py` turns the Python System into glue that the `.cpp` includes. Edges become pointer binds made once, and the tick is the topological order as non-virtual calls:

```cpp
struct ArmController {
  MoteusBus bus; Jtc jtc; Td4Wrist td4; Guard guard; lk::Delay<rm::ServoCommand> safe_d1;
  void wire() { jtc.state.bind(bus.state); /* …each edge… */ bus.command.bind(safe_d1); }
  void tick(lk::Ctx& c) { bus.step(c); jtc.step(c); td4.step(c); guard.step(c); safe_d1 = guard.safe.value(); }
};
LK_EXPORT(ArmController)
```

**Bound to dora today.** `backends/dora/host.cpp` is a dora node built on the upstream cxx API v0.5.0, not lk_dora's C ABI.
- It `dlopen`s the `.so`.
- A receive thread runs `next_event` → `event_as_arrow_input_with_info` → `push`.
- A drain thread runs `pull` → `send_arrow_output`.
- `rt.cpp` owns the tick. The `.so` never links dora.

**Another transport** is another host binary against the same `lk_box_v1`. The in-process host is ctypes. `CDLL` calls release the GIL ([ctypes](https://docs.python.org/3/library/ctypes.html)), so the RT `std::thread` never touches Python.

## 5. The consumer case, end to end

**File (a)** is `PickPlanner` from §3. It declares outputs only.

**File (b)** is the controller: interfaces and chain wiring in Python, bodies in C++.

```python
# arm_controller/box.py: interfaces only; the Python fallback subclasses them
class Jtc(lk.Component):
    trajectory: lk.In[JointTrajectory] = lk.latest(); state: lk.In[ServoState]
    command: lk.Out[ServoCommand]; status: lk.Out[ControllerStatus]
class Guard(lk.Component):
    proposed: lk.In[ServoCommand]; state: lk.In[ServoState]; claim: lk.In[Claim] = lk.latest()
    safe: lk.Out[ServoCommand]
# MoteusBus (command → state; Params.adapter "0607623A" | "twin") and Td4Wrist alike

def arm_controller() -> lk.System:
    s = lk.System("arm_controller")
    s.add(bus=MoteusBus(), jtc=Jtc(), td4=Td4Wrist(), guard=Guard())
    s.connect(s.bus.state, s.jtc.state, s.td4.state, s.guard.state)
    s.connect(s.jtc.command, s.td4.fivebar)
    s.connect(s.td4.command, s.guard.proposed)
    s.connect(s.guard.safe, s.bus.command, delay=1)   # one query+command frame per tick
    s.expose(trajectory=s.jtc.trajectory, claim=s.guard.claim,
             state=s.bus.state, command=s.guard.safe, status=s.jtc.status)
    return s
```

```cpp
// arm_controller/arm_controller.cpp
#include "arm_controller.lk.hpp"
struct Jtc : lk::gen::Jtc {
  rm::JointTrajectoryView active;
  void step(lk::Ctx& c) {
    if (auto t = trajectory.take()) active = *t;                          // swap at the tick
    auto& cmd = command.loan();
    if (active.empty() || c.now() > active.end()) hold(state.get(), cmd); // R5
    else active.sample(c.now(), cmd);                                     // absolute start
  }
};
struct MoteusBus : lk::gen::MoteusBus { /* mjbots lib/cpp; BUS_LOCK, take_over */ };
struct Guard : lk::gen::Guard { /* faults, runaway → hold, claim lease */ };
struct Td4Wrist : lk::gen::Td4Wrist { /* … */ };
#include "arm_controller.lk.glue.hpp"
```

**File (c)** is the cell. `cell()` is the function from §3.

```python
arm_cpp = lk.load("build/libarm_controller.so", params="records/bam-fb-arm/arm_controller.toml")
arm_py  = lk.load("bam_rack_twin.nodes:ArmControllerPy")   # ~30-line adapter over today's ArmPlant

target   = lk.Deployment(c := cell(arm_cpp), transport="dora").place(c.ctl, process="arm", cpus=[20], fifo=80, rate_hz=100)
same_box = lk.Deployment(c := cell(arm_cpp)).place(c.ctl, cpus=[20], fifo=80, rate_hz=100)
fallback = lk.Deployment(c := cell(arm_py)).place(c.ctl, rate_hz=50)
```

**What happens at run time.**
- **The planner never blocks.** When `forward` returns, the runtime wraps the numpy buffers as Arrow with no copy and calls `send_output` (dora) or `push` (`same_box`, GIL released). State comes back on a `latest` edge.
- **Swaps are checked** by exposed names and schema hashes at build.
- **The original scripts stay the real fallback.** `arm_hold`, `pick_loop_v1` and `fivebar_run` are untouched.
- **The always-on service form (R2, R17)** places the controller box alone with `.serve(at="0.0.0.0:7447")` under systemd. Planners attach with `lk.load("lk://host:7447/arm_controller")`, which keeps dora off the control path.

## 6. Backend independence

| Change | Touches | User lines |
|---|---|---|
| dora 0.5 → 1.x | `dora/emit.py` (`_unstable_deploy` → `deploy`), `dora/host.cpp` (removed `init_dora_node_from_id` and `DataSampleHandle`, per the [migration guide](https://github.com/dora-rs/dora/blob/main/docs/migration-from-0.x.md)), and the extra's pin | 0 |
| dora → zenoh | new `backends/zenoh/` (a manifest, plus a host on `<process>/<port>` keys) | 1 (`transport`) |
| same host → iceoryx2 | a new host binary on `lk_box_v1` | 1 |

**Skew.** The spec pins `dora==0.5.0`. Each host checks its linked version at start and exits naming both. Cross-machine daemon skew is still unsolved.

**What leaks.**
1. **dora's supervision model** defines `process`. Zenoh has none, so we would generate systemd units.
2. **`latest`** maps to dora's `queue_size: 1`, which has a stale-slot bug (dora PR #652). Our LatestBox enforces `latest` anyway, and a backend that cannot honour a policy refuses it at build.
3. **Arrow is our wire**, so zenoh or iceoryx2 costs Arrow IPC framing.
4. **Late join** needs the gateway on dora 0.5.

## 7. Overhead compared with raw dora

| Path | Added work per message | Extra copies | Estimate |
|---|---|---|---|
| C++ → C++, in-process | a pointer bind made once; an inlinable call | 0 | ~1 ns per edge |
| Python → Python, in-process | a generated step function (exec, as [dataclasses](https://github.com/python/cpython/blob/main/Lib/dataclasses.py) does); a local passed | 0 | 0.2–0.5 µs per edge |
| Python → C++, same process | ctypes `push`, about 6 ArrowArray structs over numpy | 0 | 5–10 µs, GIL released |
| Python → dora | `pa.Array.from_buffers` per field; hash and seq in metadata | 0 | 5–10 µs of Python frames |
| dora → C++ | a view decode, an 8-byte hash compare, a box swap | 0 | ~100 ns, off the RT thread |
| dora → Python | numpy views over Arrow | 0 with a public `adopt`, otherwise +1 (24 kB ≈ 3 µs) | — |

The RT tick makes 0 allocations, and `import like_keras` takes about 10 ms because it is stdlib only. The schedule, port indices and hashes are fixed at build, so the hot path has nothing to look up.

## 8. Self-score, risks, milestones

**Gates.**
- **Equivalence** holds by construction: one spec feeds every backend.
- **Fallback untouched**: like_keras is optional.
- **RT hot path**: a static chain, `loan()` and the graveyard ring.
- **Lean core**: robot_msgs is the only dependency, and it is imported lazily.

| # | Criterion (weight) | /10 | Why |
|---|---|---|---|
| 1 | Consumer fit (15) | 8 | T1–T8 and T12 follow from the design. But a C++ chain edit means a rebuild, and the C++ side is 2 files. |
| 2 | Speed (13) | 8 | Static code, 0 copies, free C++ edges. The Python codec frames are the 64 B risk. |
| 3 | Black-box (11) | 9 | A generated ports struct cannot name a peer. In Python only the lint stops it. |
| 4 | Succinctness (11) | 7 | 8 top-level names, about 28 in all, at most 6 params. But C++ boxes need a Python stub, and `transport` is a string. |
| 5 | Swap cost (9) | 9 | 1 Deployment line per swap. |
| 6 | dora insulation (8) | 8 | dora appears only in `backends/dora/`, nodes only, and we own the wire. Daemon skew is open. |
| 7 | Compactness (8) | 6 | About 2,350, including generated code. Generators grow, as rosidl's did. |
| 8 | Reach (7) | 9 | One declaration generates the C++ ports and codec. One `.so` runs in 3 placements, with about 8 non-body lines. |
| 9 | Facilities (6) | 8 | 7 of 7, added by the compiler. Recording in v0 is npz. |
| 10 | Testability (5) | 9 | A stepped logical clock, a deterministic schedule, a twin as a params line. |
| 11 | Network attach (4) | 7 | `serve` plus `lk://`, with the spec as the device description. Same host only. |
| 12 | Wiring check (3) | 10 | Compiling is the check, with path.port, the fix and `file:line`. |

**Weighted total ≈ 81/100.**

**Risks.**
1. **A C++ box's interface lives in Python**, so C++ builds need Python.
2. **Generator sprawl and opaque errors.** Two generators only, and every generated line traces to a `file:line`.
3. **robot_msgs gaps** block the C++ side, and they are another repo's change.
4. **Crossing a process** needs `module:Class` with params that round-trip, so closures cannot cross.
5. **C++ chain edits need a rebuild** (copper-rs's trade-off).
6. **The spec could drift into the config language Zach rejected.** The hash refuses hand edits; only the params TOML is hand-written.

**Milestones.**
1. **Spine, Python only.** Spec generation, schema hashes and golden bytes, the generated step function with a logical clock, the 20 mis-wiring cases. Gates: criteria 10 and 12, import ≤ 20 ms.
2. **C++ in-process.** `gen.py`, `lk.hpp`, `rt.cpp` and `native.py`. A malloc-counting gtest over 100k ticks. `arm_hold` (`take_over`, then a 20 ms hold) as the first box on the fake bus. Gates: RT hot path and T1–T3.
3. **dora plus serve.** `emit.py`, `node.py`, `host.cpp`, the version pin, the gateway, and `lk ls`/`lk top`. Conformance green on inproc and dora 0.5.0, plus T4–T8, T11 and T12.
