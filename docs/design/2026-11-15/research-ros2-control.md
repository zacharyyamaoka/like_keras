# ros2_control: prior art for like_keras controller nodes

*2026-11-15 · research lane, read-only. Sources: control.ros.org (Rolling), `master` source of ros2_control, ros2_controllers and realtime_tools (fetched today), GitHub issues, ROS Discourse.*

## Verdict

- **The core idea is right, and small.** One process, one real-time thread: `read() → update(every active controller, in chain order) → write()`, with controllers passing values through shared memory, never messages. The maintainers' reasons (1 kHz loops, DDS jitter) match Zach's plan: chain controllers in one process, dora at the edge.
- **The packaging hurts:** pluginlib, a plugin XML, a URDF `<ros2_control>` tag, a LifecycleNode per controller, string-keyed scalar handles each with its own mutex, a 5,066-line controller manager `.cpp`. No Python controllers.
- **The planner→controller seam is wrong for us.** The JointTrajectoryController (JTC) owns a ROS action server: wiring inside the component. Copy its real-time handoff (a latest-value box swapped in at the next tick) but put the port outside, wired by the composer.

## 1. Architecture

```
planner ──ROS action/topic──► [ ros2_control_node process ]
                                main thread: rclcpp executor (services, subscribers, action servers)
                                RT thread (SCHED_FIFO, prio 50):
                                  rm.read()   → hardware components fill state handles
                                  cm.update() → active controllers, chain-sorted
                                  rm.write()  → hardware components consume command handles
```

- **controller_manager (CM)** loads, configures, activates and switches controllers. It matches the interfaces controllers claim against the ones hardware provides, and it runs the loop. Users reach it through services or the `ros2 control` CLI ([getting started](https://control.ros.org/rolling/doc/getting_started/getting_started.html)).
- **ResourceManager** loads hardware components through pluginlib and owns their lifecycle and every state and command interface. A command interface has one owner at a time. Maintainer destogl calls that "not a bug, but a feature", and another user recalls two publishers on one command topic causing an overtorque fault ([Discourse 38751](https://discourse.openrobotics.org/t/question-regarding-ros2-control-design/38751)).
- **Hardware components**: `SystemInterface` (multi-DOF, one comms channel), `ActuatorInterface` (one DOF), `SensorInterface` (read-only).
- **Controllers** derive from `ControllerInterface` or `ChainableControllerInterface`. Each is backed by an `rclcpp_lifecycle::LifecycleNode` (`get_node()` in [controller_interface_base.hpp](https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/controller_interface/controller_interface_base.hpp)).

## 2. The loop

[`ros2_control_node.cpp`](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp) starts a `std::thread` that applies `cpu_affinity`, calls `configure_sched_fifo(thread_priority)` (default 50) and optionally `lock_memory` (default on only with an RT kernel), then loops `cm->read(); cm->update(); cm->write();` and sleeps until the next period. The main thread spins the ROS executor. Other parameters ([CM userdoc](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)): `update_rate` (default 100), `overruns.manage`, `hardware_synchronization.expect_blocking_read_write`.

Built-in statistics: `periodicity` (Hz) and `execution_time` (µs), each with `mean_error` and `standard_deviation` warn/error thresholds, for the CM, each controller and each hardware component. **Ready-made KPIs; copy them as they are.**

## 3. Hardware components

**Lifecycle:** `on_init → on_configure` (open comms) `→ on_activate` (enable power) `→ on_deactivate → on_cleanup / on_shutdown / on_error`.

**Per-cycle calls:**
- `read(time, period)` and `write(time, period)` return `OK`, `ERROR` or `DEACTIVATE`.
- An `ERROR` stops every controller that uses that component's interfaces.
- Control-mode changes go through `prepare_command_mode_switch` and `perform_command_mode_switch`.

**Boilerplate:** the [writing-a-component guide](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html) takes 9 steps and 5 files (header, `.cpp`, plugin XML, load test, CMake/package.xml edits), plus `PLUGINLIB_EXPORT_CLASS`, `pluginlib_export_plugin_description_file` and the URDF `<ros2_control>` block naming the plugin.

**Interfaces:**
- Interfaces are named strings such as `joint1/position`.
- They were `double`-only for years. Lyrical added float32, int8–32 and uint8–32 ([Lyrical what's new](https://discourse.openrobotics.org/t/ros2-control-in-lyrical-luth-whats-new/59593)).
- Each `Handle` holds a `std::variant` plus a `std::shared_mutex`. `get_optional()` uses `try_to_lock` and returns `nullopt` when the lock is contended ([handle.hpp](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/handle.hpp)).

## 4. Controllers and chaining

A controller declares what it claims with `command_interface_configuration()` and `state_interface_configuration()`. `update(time, period)` runs every cycle.

A **chainable** controller also exports **reference interfaces** through `on_export_reference_interfaces_list()`. These are virtual command interfaces that an upstream controller can claim. It can also export state interfaces. Its update is split in two ([chainable_controller_interface.hpp](https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/controller_interface/chainable_controller_interface.hpp), [chaining doc](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/controller_chaining.html)):
- `update_reference_from_subscribers()`, skipped while chained;
- `update_and_write_commands()`.

**How a chain runs in one loop:**
- `update_list_with_controller_chain` sorts `ordered_controllers_names_` so a controller runs before the ones whose reference interfaces it claims ([controller_manager.cpp](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp)). A value A writes, B reads later in the same cycle: a plain memory read, no message.
- When A claims B, `on_set_chained_mode(true)` makes B ignore its own subscriber (PID ignores `~/pid_reference`, diff_drive ignores `~/cmd_vel`).
- Downstream controllers activate first. A failing controller deactivates its whole chain (#2681); `fallback_controllers` take over.

## 5. How the planner hands a trajectory to the JTC without blocking

From [joint_trajectory_controller.cpp](https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/src/joint_trajectory_controller.cpp):

1. On the executor thread, `goal_accepted_callback` preempts the old goal, copies the trajectory into a `shared_ptr`, and calls `rt_new_trajectory_msg_.set(...)` on a `realtime_tools::RealtimeThreadSafeBox` (priority-inheritance mutex, best effort). The planner already has its "accepted" reply and carries on.
2. In the RT thread, `update()` calls `rt_new_trajectory_msg_.try_get(...)`. If the pointer changed, it re-sorts joints, fills missing ones, optionally runs `blend_with_active_trajectory` (`allow_trajectory_replacement`), then samples.
3. Feedback and results sit in a `RealtimeServerGoalHandle`; a non-RT wall timer (`goal_handle_timer_`) calls `runNonRealtime` to send them. State goes out through a `RealtimePublisher` (try-lock plus its own thread).

With no trajectory it holds position (`set_hold_position`); on cancel it decelerates (`max_deceleration_on_cancel`, Lyrical).

**realtime_tools guideline** ([docs](https://control.ros.org/rolling/doc/realtime_tools/doc/index.html)):

| Direction | What matters | Use |
|---|---|---|
| non-RT → RT | only the latest value | `RealtimeThreadSafeBox` |
| non-RT → RT | every message | `LockFreeQueue` (Boost.Lockfree SPSC/MPMC) |
| RT → non-RT | publishing | `RealtimePublisher` |
| either way | a flag | `std::atomic<bool>` |

The old `RealtimeBuffer` is `[[deprecated]]` in master. Its writer polled with 500 µs sleeps.

## 6. Rates and async (Jazzy → Rolling)

- **Controller `update_rate`:** the controller runs every Nth CM cycle. An unachievable rate is rounded to an achievable one (#2828).
- **Hardware `rw_rate`:** set on the `<ros2_control>` tag and capped at the CM rate ([different rates](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html)).
- **Async parts** (Jazzy era): async hardware (`is_async="true"`) and async controllers (`is_async: true`) run on `realtime_tools::AsyncFunctionHandler` threads with `thread_priority`, `affinity` and `scheduling_policy` ([async doc](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/asynchronous_components.html), [async_function_handler.hpp](https://github.com/ros-controls/realtime_tools/blob/master/realtime_tools/include/realtime_tools/async_function_handler.hpp)): `synchronized` (main loop triggers each call), `detached` (own clock), and in master `SELF_PACED` (the hardware sets the pace "by blocking at start of read()").
- **Handoff between async parts and the loop** goes through the per-handle mutex; maintainer saikishor gives that as why handles use mutexes, not atomics ([#3479](https://github.com/ros-controls/ros2_control/issues/3479)).

## 7. Mock hardware

`mock_components/GenericSystem` copies commands into states; `calculate_dynamics` integrates instead, `position_state_following_offset` fakes a tracking error, `disable_commands` simulates lost feedback, `mock_sensor_commands` injects sensor values ([mock docs](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)). Mock vs real is one `<plugin>` line.

## 8. Python

C++ only. Asked whether a ros2_control package can be written in Python, maintainer christophfroehlich: "No, currently this is not possible" ([#1615](https://github.com/ros-controls/ros2_control/issues/1615)). Workarounds: Gepetto's pybind11 bindings ("need more work"), ICube's [pytroller](https://github.com/ICube-Robotics/pytroller) generator, PickNik's [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control) for drivers without hard real-time needs. Python exists only outside the loop: the `controller_manager` service wrappers and the `spawner`.

## 9. Known pains (primary sources)

| Pain | Evidence |
|---|---|
| Plugin, XML and CMake boilerplate; the XML repeats the code | The 9-step guide above. A gripper author found his `<ros2_control>` XML "seems redundant" and wrote a plain node instead ([Discourse 35122](https://discourse.openrobotics.org/t/looking-for-feedback-on-design-on-ros2-control-interface-for-robotiq-3f-gripper/35122)) |
| Tied to the URDF | Config arrives as the `robot_description` string; "Reloading is not supported"; every `<ros2_control>` joint must exist in the URDF (CM userdoc). `hardware_interface` depends on `urdf`, `sdformat_urdf`, `tinyxml2` |
| Heavy ROS dependencies | `hardware_interface` needs rclcpp, rclcpp_lifecycle, pluginlib, control_msgs, pal_statistics, realtime_tools. Master today: CM `.cpp` 5,066 lines, ResourceManager 2,767, JTC 2,465 |
| Scalar, string-keyed interfaces | `double` only until Jazzy/Lyrical. For bools and enums there is "no standard way" (Discourse 35122) |
| Handle reads can fail | A per-handle try-lock returns `nullopt`, which forces verbose error paths ([#3479](https://github.com/ros-controls/ros2_control/issues/3479)) |
| Locks in the RT path | `publish_activity()` took a recursive mutex inside `update()` during fallback ([#3145](https://github.com/ros-controls/ros2_control/issues/3145)). `get_lifecycle_state()` is not RT-safe (CM userdoc) |
| Overrun "catch-up" | After a missed period the loop fired near-zero-dt cycles, and position commands spiked (tylerjw, [#736](https://github.com/ros-controls/ros2_control/issues/736)). The fix was `overruns.manage` |
| Not networked | The loop runs "without any ROS networking magic", and DDS adds jitter above 1 kHz. destogl estimated a node-based split at 6–12 months, later "1–2 years" (Discourse 38751) |

## 10. Steal / avoid, for a lean non-ROS library driving moteus

| Area | Steal | Avoid |
|---|---|---|
| Process model | One RT thread running `read → controllers in sorted order → write`, with SCHED_FIFO, CPU affinity and mlockall | An executor and services sharing the process with no hard boundary |
| Chaining | A sorted, in-process call list; values passed in memory; downstream controllers activate first; a failure deactivates the whole chain | A controller switching off its own subscriber (`on_set_chained_mode`): that is wiring inside the component. In like_keras the external wiring decides whether an input comes from an upstream controller (a call) or from dora (a port) |
| Data between controllers | Typed per-cycle structs (robot_msgs `JointState` / `ServoCommand`, struct-of-arrays) owned by the loop thread, with no locks | String-keyed `double` handles with a mutex each |
| Planner seam | A latest-value box (atomic `shared_ptr` swap) read at the start of each tick; replace or blend; hold when empty; decelerate on cancel | An action server inside the controller. The planner just emits a `Trajectory` on its output port |
| RT ↔ non-RT | Three primitives only: latest-value box, SPSC queue, RT publisher thread | `RealtimeBuffer` (polling writer, deprecated) |
| Hardware | Four verbs: configure (open bus), activate (torque on), exchange, deactivate. An `ERROR` stops dependent controllers and the fallback (hold) takes over | pluginlib, plugin XML, and the URDF `<ros2_control>` tag as the config carrier. Build hardware in code from a typed config instead |
| Pacing | Let the hardware pace the loop (`SELF_PACED`): the fdcanusb round-trip sets the tick. Integer rate dividers per controller. Never catch up after an overrun | Separate `read()` and `write()` calls: one moteus query-with-command frame per board does both |
| Testing | A mock that copies commands into states, with integration, offset and dropped-feedback options, chosen by config | The mock as a separate plugin package |
| KPIs | `periodicity` and `execution_time` (mean and standard deviation) per loop, controller and device, published off the RT thread | Packing statistics into strings |
| Languages | One controller contract, implemented in Python (reference and fallback) and in C++ | C++ only, with Python as an afterthought |

## 11. Sketch: the moteus node

```cpp
// One process: an RT thread plus a dora I/O thread.
LatestBox<std::shared_ptr<const Trajectory>> traj_in;  // dora receive thread calls set()
SpscQueue<ArmStatus, 64> status_out;                    // RT thread -> dora send thread
void rt_loop() {                                        // SCHED_FIFO, pinned core, mlockall
  for (;;) {                                            // paced by the bus exchange
    JointState s = bus.exchange(cmd);                   // moteus query+command, one frame per board
    if (auto t = traj_in.take()) jtc.replace(*t, s);    // never blocks
    Setpoint ref = jtc.sample(now);                     // the chain is plain function calls
    cmd = pd_ff.tick(ref, s, dt);
    status_out.try_push({s, ref, cmd, timing});
  }
}
```

The Python planner writes a `Trajectory` on its output port and returns at once; the wiring file connects that port to `arm.trajectory`. The same `jtc → pd_ff` chain runs as Python objects in the single-script fallback.

**Measure before promising a rate.** The full arm runs at 50 Hz because a 4-board exchange measured p95 9.3 ms. C++ removes interpreter cost, not bus time: time the C++ exchange first.
