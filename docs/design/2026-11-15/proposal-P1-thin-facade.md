# P1: thin facade (the Keras 3 shape)

*2026-11-15 · proposal lane P1 · scored against `criteria.md` · grounded in `map-code.md`, `consumer-spec.md` and the research notes.*

## 1. Thesis and concepts

**Thesis.** like_keras is a facade of about 2.5k lines: the same six names in Python and C++ describe black-box components, outside wiring and placement, and nothing else. A `Deployment` compiles the wired `System` into one plain-data plan, and a backend module (`inproc` or `dora`) turns each wire into its own mechanism (a local variable, a pointer, a lock-free slot, a dora input with `queue_size`), then steps aside. At run time only a small host loop and the frame codec remain: nothing per in-process edge, a 40-byte header per cross-process message.

**Concepts: 28 public names (target ≤ 30), 6 top-level (target ≤ 8).**

| Scope | Names |
|---|---|
| Top level (6) | `Component`, `In`, `Out`, `System`, `Deployment`, `load` |
| Component (10) | `forward`, `Params`, `configure`, `activate`, `deactivate`, `log`, `metric`, `now`, `age`; implicit ports `log`, `metrics`, `health`, `params` |
| System (4) | `add`, `wire`, `feedback` (a wire delayed one tick), `expose` |
| Deployment (6) | `place`, `record`, `check`, `start`, `stop`, `describe` |
| C++ only (2) | `get`, `put` (plus the `LK_EXPORT` macro) |

Not public: the backend protocol, the frame layout, the CLI (`lk ls`, `lk top`).

## 2. Repo architecture

```text
like_keras/                                       est. SLOC
  pyproject.toml   deps: robot_msgs · extra [dora]: dora-rs==0.5.0, pyarrow
  src/like_keras/
    __init__.py          15  the 6 names; stdlib imports only (robot_msgs loaded lazily)
    component.py        160  Component, In, Out, Params, facilities, eager __call__
    system.py           170  add/wire/feedback/expose; type, cycle, coverage, one-writer checks
    deployment.py       180  place/record/check/start/stop/describe → Plan (frozen dataclasses, the one IR)
    codec.py            150  robot_msgs ⇄ LKF frame, schema hash, C++ view generator
    load.py              45  "pkg.mod:Class" | "*.so" | "lk:///run/…sock" | "*.lkrec"
    backends/inproc.py  300  fused threads, slots, ctypes native host, logical clock, attach socket
    backends/dora/      330  Plan → dataflow.yml + plan.json, version check (190); Python host (140)
  cpp/
    include/like_keras/lk.hpp  450  Component, In, Out, System, TripleSlot, SpscRing, log ring, C ABI
    src/rt_host.cpp            180  RT thread: affinity, SCHED_FIFO, mlockall, absolute pacing, metrics
    hosts/dora_host.cpp        170  lk-dora-host: dlopen(component .so) ↔ dora C API (the only C++ linking dora)
    cmake/like_keras.cmake      40  lk_component(...), lk_messages(robot_msgs.control ...)
  generated C++ views (6 types)  ~300 (counted)
  tests/conformance/  one suite, parametrized over every backend      kpi/  frozen ruler
```

Total ≈ 2,490: core ≈ 720 (3% over 700), backends 300/330, C++ runtime 840.

**robot_msgs** is the one runtime dependency, loaded lazily: `codec.py` derives the frame layout and a schema hash (SHA-256 of the field list, cut to `u64`) from a `Msg`'s dataclass fields. Any Python object may travel an in-process Python wire; only `Msg` types may cross a C++, process or host boundary (`check()` enforces it). `lk_messages()` generates the C++ views at build time; like_keras never defines a message.

## 3. Python API

```python
import like_keras as lk
from dataclasses import dataclass
from robot_msgs.control import ServoState
from robot_msgs.trajectory import JointTrajectory

class PickPlanner(lk.Component):                  # black box: ports in, ports out
    target = lk.In(PickTarget, queue=8)           # queue: forward runs once per message
    state = lk.In(ServoState)                     # latest: sampled, never waited on
    trajectory = lk.Out(JointTrajectory)

    @dataclass(frozen=True)
    class Params:
        lead_s: float = 0.05

    def forward(self, target, state):             # names must equal the In names (checked at class creation)
        if self.age("state") > 0.1:
            self.log.warning("state stale")       # → log port; health shows WARN
            return None                           # nothing sent
        plan = plan_pick(target, start=state.joint_state())
        return plan.starting_at(self.now() + self.params.lead_s)   # absolute start
```

`PickPlanner()(target=t, state=s)` runs `forward` eagerly on a test clock. One output returns the bare value; several return a tuple in declaration order, with `None` meaning "not sent".

**Wiring from outside.** A component has no way to name a peer, a transport, a System or a Deployment.

```python
def pick_cell(arm: lk.Component) -> lk.System:
    s = lk.System("pick_cell")
    targets, planner, ctl = s.add(TargetFile("targets.jsonl")), s.add(PickPlanner()), s.add(arm)
    s.wire(targets.target, planner.target)
    s.wire(planner.trajectory, ctl.trajectory)
    s.wire(ctl.state, planner.state)              # crosses threads: the slot is the delay
    s.expose(ctl.state, ctl.command, ctl.health, ctl.metrics)   # what an attaching process sees
    return s
```

`check()` fails at build, naming `path.port` and the fix, on: a type or schema-hash mismatch, an unwired In, two writers on one In, a source placed without `hz`, a same-thread cycle not cut by `feedback`, a capability the backend lacks, or core 22.

**Deployment.** `place(*components, process=, host=, hz=, cpus=, fifo=)` takes six parameters. Each `place` call is one thread, and each distinct `process` is one OS process. Under `inproc` the processes collapse into one, and every thread stays a thread with the same slot semantics.

```python
cell = pick_cell(arm)
dep = lk.Deployment(cell, backend="dora")                  # "inproc"; realtime=False → logical clock
dep.place(arm, process="arm-controller", hz=50, cpus=[20], fifo=80)
dep.place(cell["targets"], cell["planner"], hz=2)          # default process "main"
dep.record("runs/pick.lkrec")                              # taps boundary wires + exposed ports
dep.start()                                                # non-blocking; start(until=12.0) steps a sim clock
```

**Normative triggering** (Keras leaks at its loop; this is ours, so every backend must honour it):
- A thread with `hz` steps its sources and latest-only components each period, in wire order.
- A `queue` input steps its component once per message.
- A thread without `hz` steps only on `queue` messages, then runs its downstream as plain calls.
- `latest` never triggers. An overrun never catches up.

**Standard facilities, with no user code:**

| Facility | Author writes | Framework does |
|---|---|---|
| log | `self.log.info(...)` (stdlib Logger) | `log` port; in C++ a lock-free ring drained off the RT thread |
| metrics | optional `self.metric(name, x)` | 1 Hz: forwards, exec p50/p99/max, period error, overruns, input age, drops (ros2_control's `periodicity`, `execution_time`) |
| health | nothing | 1 Hz: lifecycle state + worst log level; STALE after 3 missed periods; exception → ERROR → `deactivate` |
| params | nested frozen `Params` | a latest `params` input, applied between forwards via `configure` |
| lifecycle | optional hooks | host-driven; a restarted component comes back inactive |
| clock | `self.now()` | logical tick time; `realtime=False` steps it flat out |
| record, introspect | nothing | `.lkrec` = LKF frames + log/publish time, replayed by `lk.load`; `describe()` lists ports, hashes, policies, threads, transport per wire; `lk top` reads it over attach |

## 4. C++ side

Every message is a generated fixed-layout view over an LKF frame, so an `Out` is its own wire buffer.

```cpp
#include <like_keras/lk.hpp>
#include <robot_msgs/lk/control.hpp>        // generated: ServoState, ServoCommand
#include <robot_msgs/lk/trajectory.hpp>     // generated: JointTrajectory (spans into the frame)

struct Jtc : lk::Component {
  lk::In<JointTrajectory> trajectory{this, "trajectory"};
  lk::In<ServoState>      state{this, "state"};
  lk::Out<ServoCommand>   command{this, "command"};
  void configure() override { sampler_.reserve(7, 4096); }      // allocate here, never in forward
  void forward() {
    const ServoState* s = state.get();      // nullptr until the first sample
    if (!s) return;
    if (trajectory.get() != active_) active_ = sampler_.replace(trajectory.get(), *s);
    sampler_.sample(now(), *s, command.put());                   // in place; no put() → nothing sent
  }
  TrajSampler sampler_; const JointTrajectory* active_ = nullptr;
};
```

**One chain, one thread.**
- `s.add<T>()` stores `{void* self, void (*fn)(void*)}` in a vector reserved once.
- `wire(out, in...)` sets `in.src = &out.storage`. A C++ mis-wire is a compile error, and an edge costs one pointer read.
- A topological sort, with `feedback` edges cut, fixes the order.
- `rt_host.cpp` then loops:

```cpp
for (;;) {
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
  for (auto& b : boundary_in)  b.latch();      // TripleSlot pointer swap: no copy, no lock
  for (auto& c : chain)        c.fn(c.self);   // plain calls in wire order
  for (auto& b : boundary_out) b.publish();    // written outputs → SpscRing
  next = lk::advance(next, period, &overruns); // never catches up
}
```

**dora today.** `lk-dora-host libarm_controller.so plan.json arm-controller` is built once against `dora-node-api-c 0.5.0`.
- It runs `dlopen`, `lk_create(config)`, `lk_start(hz, cpus, fifo)`.
- Its main thread hands each `dora_next_event` to `lk_attach(port, ptr, len, free_dora_event, event)`: zero-copy, the slot frees the event off the RT thread.
- A sender thread drains `lk_take(out)` into `dora_send_output`. Component `.so` files never link dora.

**Another transport** (iceoryx2, zenoh) is a new ~150-line host on the same C ABI (`lk_describe`, `lk_create`, `lk_start`, `lk_attach`, `lk_take`, `lk_stop`); no component recompiles.

**In-process.** `inproc` opens the same `.so` with `ctypes.CDLL`, which releases the GIL for every call. The RT thread never touches Python.

## 5. The consumer case end to end

**File 1, `bam_pick/planner.py`.** `PickPlanner` from §3: it only declares outputs. `TargetFile` is a 10-line source.

**File 2, `bam_arm_node/arm_controller.cpp`.** The bus, JTC and guard in one thread; ports exist only at the boundary.

```cpp
lk::System arm_controller(std::string_view config_toml) {   // the author parses config (e.g. toml++)
  auto c = ArmConfig::parse(config_toml);
  lk::System s{"arm_controller"};
  auto& bus   = s.add<MoteusBus>("bus", c.bus);       // mjbots lib/cpp; BUS_LOCK + take_over() in activate()
  auto& jtc   = s.add<Jtc>("jtc");
  auto& guard = s.add<Guard>("guard", c.guard);       // fault, runaway → hold; overspeed logged only
  s.feedback(bus.state, jtc.state, guard.state);      // state from exchange k feeds tick k+1
  s.wire(jtc.command, guard.proposed);
  s.wire(guard.safe, bus.command);                    // bus.forward = one query+command exchange
  s.expose(jtc.trajectory, guard.claim, bus.state, guard.safe);
  return s;
}
LK_EXPORT(arm_controller)   // libarm_controller.so: dora node, in-process thread, or twin bus via config
```

**The Python peer**, `bam_arm_node/py_controller.py:ArmControllerPy`, wraps today's `ArmPlant` behind the same ports (`activate` → `take_over()`, `forward` → one exchange, `deactivate` → stop feeding). It lives outside `bam_rack_twin`, so `arm_hold`, `pick_loop_v1` and `fivebar_run` stay byte-for-byte untouched as the real fallback.

**File 3, `bam_pick/cell.py`.** Wiring and placement, owned by the system builder.

```python
import signal, like_keras as lk
from bam_pick.planner import PickPlanner, TargetFile

def pick_cell(arm): ...                               # exactly as in §3

arm = lk.load("build/libarm_controller.so", config="records/bam-fb-arm/arm_controller.toml")
# arm = lk.load("bam_arm_node.py_controller:ArmControllerPy")   # all-Python: swap this line
cell = pick_cell(arm)
dep = lk.Deployment(cell, backend="dora")             # "inproc": same box, C++ on its own thread
dep.place(arm, process="arm-controller", hz=50, cpus=[20], fifo=80)
dep.place(cell["targets"], cell["planner"], hz=2)
dep.start()
try: signal.pause()
finally: dep.stop()
```

**Under dora:**
1. `check()` matches Python port types against the `.so`'s `lk_describe()` hashes, confirms dora 0.5.0, and writes `dataflow.yml` with nodes `main` (Python host) and `arm-controller` (`lk-dora-host`). Policies compile to `queue_size` (latest → 1, `queue=8` → 8).
2. The planner's `forward` runs for seconds on the host thread. dora keeps the newest `state` and up to 8 targets; a 1 Hz facilities thread keeps `health` alive.
3. `forward` returns, the host packs the frame, and `send_output` returns at once: the non-blocking send.
4. The next 20 ms tick latches the trajectory from its TripleSlot, and the JTC switches at its absolute `header.stamp`.
5. If the daemon stalls for 4 s (T4), the RT thread finishes the trajectory, holds, and `metrics` shows the input age.

**Same box.** `backend="inproc"`: ctypes loads the `.so`, `lk_start` spawns the SCHED_FIFO thread, the trajectory goes `lk_loan` → encode → `lk_commit` (one copy, ~20 µs for 7×1000×3 float64), and `state` is sampled with `lk_take` before each forward.

**All Python.** Swap the `load` line; `ArmControllerPy` ticks on a Python thread at 50 Hz. Each swap (backend, `load`, `host="rig"`) is one line.

## 6. Backend independence

**dora 0.5 → 1.x.** Only three files change:
- `backends/dora/__init__.py`: `_unstable_deploy` becomes `deploy`, and the pin moves.
- `backends/dora/host.py`: the 1.x node API is frozen, so the edit is small.
- `hosts/dora_host.cpp`: rebuild against `dora-node-api-c` 1.x.

Components, Systems, Deployments, every component `.so` and the wire format do not change. LKF is ours, and dora carries it as opaque `UInt8`.

**Skew.** The plan pins `dora_version`; `check()` refuses a mismatched CLI or daemon, naming both; attach clients are refused within 2 s via a hello frame.

**dora → iceoryx2 (same host) or zenoh (LAN).** One backend of about 300 lines plus a C++ host of about 150.

**What leaks:**
1. **Launch and supervision**, the biggest leak. dora spawns, restarts (`restart_policy`) and deploys remotely; iceoryx2 and zenoh do not, so that backend generates systemd user units (R2 wants them anyway). With no supervision API, restart is a backend default (v0: restart, 1 s backoff, come back inactive).
2. **Policy capability.** zenoh has no subscriber queue, so the host implements one. A backend declares its capabilities as data and refuses at `check()`; it never degrades silently.
3. **Host naming.** `host=` means a dora machine id in one backend and a zenoh locator in another. The leak stays inside Deployment.
4. **Latency profile.** dora uses TCP below 4 KiB and shared memory at or above it. `describe()` reports the transport per wire rather than hiding it.

Late join does not leak: attach is our own per-process unix socket carrying exposed ports in LKF (the OBSBOT gateway pattern), which also sidesteps dora 0.5's unanswered dynamic nodes.

## 7. Overhead versus raw dora

| Path | Baseline | like_keras adds | Extra copies |
|---|---|---|---|
| C++ edge in a chain | direct member call | a pointer read per edge; one indirect call per component (~1–2 ns); exec time sampled every 64th tick | 0 |
| Python edge, same thread | `b(a(x))` | a generated straight-line function with outputs as locals; ~0.2 µs per component for timing | 0 |
| Python edge, other thread | none | `deque.append` + `Event.set`, ≈ 1–3 µs, plus the GIL handoff | 0 |
| Python → C++, in-process | none | 2 ctypes calls ≈ 2 µs, plus an encode into the loaned slot | 1 |
| Python → dora send | `send_output(id, pa.array(arr))` | header and array table via `struct.pack_into`, ≈ 2–4 µs | 0 (raw dora copies too) |
| dora → Python receive | `event["value"].to_numpy()` | header parse ≈ 1 µs; `np.frombuffer` views; `Msg` built without re-validation when the hash matches, ≈ 3–5 µs | 0 |
| dora → C++ receive | read the event bytes | ~20 ns header check | 0 |
| C++ → dora send | `dora_send_output` inside your loop | ~200 B memcpy into the ring, plus a futex wake (≈ 2–10 µs latency) | 0, plus one thread hop |

No allocation per C++ tick after `configure` (malloc counter, 100k ticks). `import like_keras` is stdlib-only, est. 5–8 ms. Weak spot: the C++ → dora hop sits near the 10 µs ceiling, the price of never calling dora from the RT thread.

## 8. Self-score, risks, milestones

| # | Criterion (weight) | 0–10 | Why |
|---|---|---|---|
| 1 | Consumer fit (15) | 8 | 3 placements, one `.so`, one System; T3/T4 from RT thread + slots; T5/T6 lean on dora restart and board timeouts |
| 2 | Speed (13) | 8 | Zero per in-process edge and no extra copies; Python decode ≈ 5 µs; one hop on the C++ out path |
| 3 | Black box (11) | 9 | No API names a peer or transport; a stray `import dora` is caught only by lint |
| 4 | Succinctness (11) | 9 | 6 top-level and 28 total names, at most 6 parameters, about 15 lines of consumer wiring |
| 5 | Swap cost (9) | 9 | One line per swap; two when the rates differ |
| 6 | dora insulation (8) | 8 | dora is named in one Python module plus one C++ host |
| 7 | Compactness (8) | 7 | ≈ 2,490 total; the core is 3% over its target |
| 8 | Python + C++ reach (7) | 7 | Generated bit-exact views, 2-file components; Python and C++ never share a thread |
| 9 | Facilities (6) | 7 | 7/7 with no user code; log-derived health may be coarse; live C++ params in v1 |
| 10 | Testability (5) | 8 | Eager `__call__`, logical clock, replay; bit-exact only within a thread |
| 11 | Network attach (4) | 5 | Same-host unix-socket attach; other hosts only via `ssh -L` |
| 12 | Wiring checked (3) | 8 | Build-time Python checks, C++ compile errors, hash checks at `check()` |

**Self-total: 80.6/100** (Σ w·s/10); discount my own scores about 10%. No gate fails by construction: equivalence suite on both backends, no `bam_rack_twin` edits, zero-allocation tick, stdlib-only core import.

**Risks.**
1. **The loop is where Keras leaks.** If `inproc` and `dora` disagree on drops or ordering, the equivalence gate fails. So the rules in §3 and the conformance suite land before either backend.
2. **"Owns almost no runtime" is half true.** The hosts (Python 140, C++ 350 lines) are a runtime; if they absorb supervision and recording this is v1–v4 again, so `kpi/` caps each at 500.
3. **Schema hash versus robot_msgs edits.** Any field change orphans old `.lkrec` files unless decode is versioned.
4. **Health inferred from logs** may miss the Diagnostics Record (typed measurements, `expected_period_s`). Fixing it means `self.health(...)`, one more name.
5. **No supervision API**, so restart behaviour differs per backend.

**Milestones.**
- **M1 (the barrier).** Python core, `inproc`, codec and conformance suite, run on `pick_cell` with a Python fake arm on the 3.5 ms fake bus, plus the mis-wiring suite. Gates: lean core, import ≤ 20 ms, zero per-edge overhead.
- **M2.** `lk.hpp`, `rt_host` and the C ABI in-process through ctypes. The first native component is the controllers lane's `arm_hold` port (`take_over`, then a hold every 20 ms). Gates: the malloc-counter gtest, and ≤ 100 ns per edge in Google Benchmark.
- **M3.** The `dora` backend (YAML generation, Python host, `lk-dora-host`), the attach socket and `lk top`. Gates: T1–T4, T7 and T11, plus the A/B overhead benchmark against raw dora 0.5.0.
