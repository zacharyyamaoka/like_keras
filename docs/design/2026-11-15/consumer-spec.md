# Consumer spec: robot controllers on like_keras (2026-11-15)

From the controller lane, the library's first consumer. Grounded in `bam_ws/src/dev/bam_rack_twin/bam_rack_twin/hardware/`, `/home/bam/robot_controllers` (CONTRACT v1) and `/home/bam/robot_msgs`.

**Bottom line:** the library has to do three things. (1) Keep one always-on process that owns the bus and runs its tick free of Python's stalls. (2) Carry typed robot_msgs across a boundary that may stall, without blocking the sender. (3) Move components between processes without touching their code. What limits the rate is the adapter firmware and the Python tail, not Python's median speed (§4).

## 1. Requirements

| # | Requirement | Pass condition |
|---|---|---|
| R1 | One owner per bus | The bus component holds the CAN adapter (`fdcanusb 0607623A`) for its whole life. It takes `BUS_LOCK` and refuses when `find_port_holders` is non-empty. A second instance exits non-zero having sent nothing. |
| R2 | Always-on hand-off, like arm-hold | Clients claim a port; they never take the bus. Start is `take_over`: a stop only if the board is in mode 11, then a hold in the same tick. It never sends a bare stop. Shutdown means it stops feeding, so each board falls into its timeout mode (10 = hold, 15 = brake). Runs as a systemd user service with `CPUWeight=2000` and `MemorySwapMax=0`. |
| R3 | Deterministic tick | Period 2-20 ms. Defaults: 10 ms for the five-bar, 20 ms for the full arm (`FULL_ARM_TICK_S`). Commands are taken at the plan tick, with catch-up and never a burst (PEP 0040). The tick is logged as int64. Tick-start jitter: p99 ≤ 0.2 ms and max ≤ 1 ms for 1 h with `stress-ng --cpu 24` running. |
| R4 | Exchange budget | Refuse to start if the exchange p95 over 20 cycles is above 0.8 × the period (`arm_plant.EXCHANGE_BUDGET_FRACTION`). Keep measuring it and publish it. |
| R5 | Two watchdog layers | Every frame carries `watchdog_timeout=0.25`. A stale input never stops the bus feed. After a trajectory ends, the node holds its last sample with qd = 0. When a claim lease lapses, it holds where it is. |
| R6 | Safety in the chain | Fault 32-50 or mode 1: stop commanding that board and apply one stop policy to the rest. A runaway triggers a hold. Overspeed is only logged. Commands are PD + feed-forward only, `ilimit_scale=0`. Torque caps come from the datasheet. No stop for a person. |
| R7 | Ports carry robot_msgs only | Inputs: `trajectory: JointTrajectory` (latest wins; `header.stamp` is the absolute start), `sequence: JointCommandSequence`, `claim`. Outputs: `state: ServoState` every tick, `command: ServoCommand` as sent, `status`, `health`, `metrics`, `log`. |
| R8 | Non-blocking send | Sending a 7-joint × 1000-sample trajectory takes p99 ≤ 0.1 ms in-process and ≤ 1 ms over dora. Each edge declares its policy (latest, queue N, or drop-oldest), and drops are counted. |
| R9 | Delivery latency | From send to the controller seeing it, p99: ≤ 0.1 ms in-process, ≤ 2 ms on the same host, ≤ 10 ms over the LAN. |
| R10 | Typed codec | Bit-exact Python↔C++ (float64, NaN payloads, names, header), at most 2 copies, schema hash checked at connect. The JSON-in-Arrow codec in `lk_dora/codec.py` does not qualify. |
| R11 | One component, three placements | One C++ `.so` that runs (a) as a dora node, (b) as a `std::thread` inside a Python host that never takes the GIL, or (c) on a twin bus. The Python implementation of the same box (today's plant code) is a peer. |
| R12 | Chain in one thread | In-process edges run as an ordered call sequence. ≤ 1 µs per edge, zero heap allocations in `tick`, and no lock shared with a non-realtime thread. |
| R13 | Realtime settings from the deployment | SCHED_FIFO, CPU pin and `mlockall` are set by the deployment and read back into `health`. Never pin to core 22, where the dora daemon has stalled for up to 4 s. |
| R14 | Standard introspection, no user code | `health` at 1 Hz. `metrics`: tick p50/p99/max, overruns, exchange p95, bus errors, watchdog lapses, drops. Structured `log` with tick and wall time. A recorder tap writes npz rows in `board.FIELDS` order. |
| R15 | Fallback untouched | `arm_hold`, `pick_loop_v1` and `fivebar_run` behave byte-for-byte as today. like_keras stays an optional dependency. |
| R16 | Backend-independent | No component imports dora. A dora version mismatch is refused at connect, naming both versions; it never hangs. |
| R17 | Late join (the IP-camera idea) | Another process or machine attaches to the running node's ports without a restart, like `obsbot_gateway`. `lk ls <host>` lists ports and their types. |

## 2. Target code

**(a) Planner: declares its outputs, nothing else.**

```python
import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.trajectory import JointTrajectory

class PickPlanner(lk.Component):
    state: lk.In[ServoState] = lk.latest()      # sampled, never waited on
    target: lk.In[PickTarget] = lk.queue(8)     # forward() runs once per target
    trajectory: lk.Out[JointTrajectory]

    def forward(self, target: PickTarget, state: ServoState) -> JointTrajectory:
        plan = plan_pick(target, start=state.joint_state())   # seconds of numpy
        return plan.starting_at(lk.now() + 0.05)              # absolute start; the runtime sends
```

**(b) Controller node: C++ chain plus the moteus bus in one process, with ports only at the boundary.**

```cpp
// MoteusBus wraps mjbots' moteus C++ library (lib/cpp): fdcanusb or socketcan
lk::System arm_controller(const ArmConfig& c) {
  lk::System s{"arm_controller"};
  auto& bus   = s.add<MoteusBus>("bus", c.bus);   // R1, R2
  auto& jtc   = s.add<Jtc>("jtc", c.jtc);
  auto& td4   = s.add<Td4Wrist>("td4", c.td4);    // W1 follows the measured L2 angle
  auto& guard = s.add<Guard>("guard", c.guard);   // fault, runaway, tilt, W2
  s.wire(bus.state, jtc.state, td4.state, guard.state);
  s.wire(jtc.command, td4.fivebar);
  s.wire(td4.command, guard.proposed);
  s.wire(guard.safe, bus.command);
  s.expose_in("trajectory", jtc.trajectory);
  s.expose_in("sequence", jtc.sequence);
  s.expose_in("claim", guard.claim);
  s.expose_out("state", bus.state);
  s.expose_out("command", guard.safe);
  s.expose_out("status", jtc.status);
  s.tick_order(bus.read, jtc, td4, guard, bus.write);   // one thread, fixed order
  return s;
}
LK_EXPORT_SYSTEM(arm_controller, ArmConfig)   // one .so: dora node or in-process plugin
```

**(c) Wiring and deployment: three placements, (a) and (b) unchanged.**

```python
arm_cpp = lk.native("build/libarm_controller.so", config="records/bam-fb-arm/arm_controller.toml")
arm_py  = lk.python("bam_rack_twin.nodes:ArmControllerPy")   # today's ArmPlant, same ports

def cell(arm):
    s = lk.System("pick_cell")
    planner, ctl = s.add(PickPlanner()), s.add(arm)
    s.wire(planner.trajectory, ctl.trajectory)
    s.wire(ctl.state, planner.state)
    return s

target   = lk.Deployment(cell(arm_cpp)).process(arm_cpp, service="arm-controller", cpus=[20], fifo=80).backend("dora")
same_box = lk.Deployment(cell(arm_cpp)).backend("inproc")   # C++ on its own GIL-free thread
fallback = lk.Deployment(cell(arm_py)).backend("inproc")    # all Python, one process
```

## 3. Acceptance tests

| Test | Setup | Pass |
|---|---|---|
| T1 Swap | Same file hashes across all 3 deployments; fake bus | C++ and Python send the same command on every plan tick, to 1e-9 rad |
| T2 Send cost | 1000 sends while the node ticks | R8 met |
| T3 Isolation | Planner spinning pure Python with `gc.collect()` every 50 ms, plus `stress-ng --cpu 24`; 1 h at 100 Hz on a fake bus with a 3.5 ms exchange | R3 met, both in-process and as a separate process |
| T4 Boundary stall | `kill -STOP` the dora daemon for 4 s mid-trajectory | Trajectory finishes on time, then holds; 0 missed ticks; `input_stale` reported |
| T5 Planner crash | `kill -9` the planner, then restart it | Arm holds; planner re-attaches and commands within 1 s |
| T6 Node crash | `kill -9` the node | Boards in timeout mode within 0.25 s; on restart, `take_over` and no mode 0 |
| T7 Exclusivity | Start a second node | Refused before the port opens |
| T8 Claim | Two planners send at once | Only the claim holder runs; the other sees a refusal on `status`; a lapsed lease means hold |
| T9 Overhead | Google Benchmark of the chain against hand-written calls, plus a malloc hook | ≤ 1 µs per edge; 0 allocations in `tick` |
| T10 Codec | Committed golden bytes per message type | Bit-exact both ways; a schema mismatch is refused |
| T11 Version skew | Client on another dora version | A clear refusal within 2 s |
| T12 Introspection | `lk top arm-controller` | Shows every metric in R14 |
| T13 Rung parity | C++ JTC on the robot_controllers ladder R1-R7 | Same gates as the Python JTC; `bam_rack_twin` tests still pass |

## 4. Where the request's assumptions may be wrong

1. **Python is not what limits the rate; the adapter firmware is.** In Python, 4 boards with batched queries took 3.416 ms mean and 4.29 ms max (`moteus_servo/DATASHEET.md`). mjbots' C++ `bandwidth_test` on old adapter firmware reaches 260 Hz for 4 devices (≈3.8 ms) and 140 Hz for 7 ([mjbots 2025-09-25](https://blog-575.pages.dev/2025/09/25/improved-performance-for-mjcanfd-usb-1x)). 0607623A shows up as `-if00`, which means legacy firmware. So on one bus, C++ caps near 140 Hz, or about 70 Hz after mjbots' advised 2× derating ([mjbots 2024-05-16](https://blog-575.pages.dev/2024/05/16/optimizing-moteus-command-rate)). Three things actually raise the rate:
   - flashing the 2025-09-19 firmware (7 devices at 570 Hz; needs an SWD programmer);
   - two adapters, one thread each;
   - the gs_usb/socketcan path, which also removes the tty behind the 0.5 s bus hang.
2. **What C++ buys is the tail.** The 50 Hz full-arm tick comes from a p95 of 9.3 ms, not the median. KeepFed exchanges measured 9-25 ms against 6.5 ms on the ladder (GIL switch interval). A 145 ms stall tripped a 0.1 s watchdog, and stalls of 0.51-0.59 s tripped three ladders. A loop outside the GIL removes those, and `keep_fed.py` goes with them.
3. **1 kHz is probably not the goal.** moteus closes PD on the board, and the JTC is `runs_on: board`. mjbots says 10-100 Hz is enough for board-side trajectories. Aim for 100-200 Hz with tight jitter. Faster only pays off for host-side laws (rung R7 at host 50 vs 500 Hz).
4. **In-process GIL-free thread versus separate process.** A C++ thread that never calls Python is immune to the GIL and GC and delivers in microseconds. But it shares the planner's fate: a segfault or an OOM kill takes it down too (the 12 s stall had 278 major page faults with swap full). Holding the arm between runs and the IP-camera goal both need a separately supervised process. So: a separate process on the real arm, in-process for twins, tests and the fallback, all from one `.so`.
5. **Keep dora off the control path.** The shared daemon stalled for up to 4 s, every machine must run the same version, and a replaced remote daemon left a dataflow marked "Running" with its nodes gone. dora carries trajectories, decimated state and health; the bus → controller → bus loop never touches it.
6. **"t = 0 is now" breaks across a process.** `replace_input` restarts the clock on arrival, so a delivery 40 ms late shifts the whole plan. Trajectories need an absolute start in a declared clock domain; `Header.stamp` is epoch time.
7. **The guards are the expensive part, not the loop.** `fivebar_plant.py` (1994 lines) and `arm_plant.py` hold the fold, runaway, tilt, W2 and self-collision guards. First C++ milestone: port `arm_hold.py` (140 lines: `take_over`, then a hold every 20 ms) as the always-on bus owner. Then the JTC with fault, runaway and hold. Port the slower guards once their twin tests pass.
