# consumer-001: the C++ JointTrajectory view has no joint names, so a C++ controller can only map a plan by column order (controllers lane, M2′, 2026-11-16)

**The ask.** Put `joint_names` in the generated `rm::JointTrajectory` view (§7.4), as `std::array<std::string_view, lk::kMaxJoints> joint_names` plus a count, like `ServoState::name`. The wire already carries the names (§7.1: `tuple[str, ...]` is a list of utf8); only the generated view drops them.

## What I tried

The raw-dora controller node maps every trajectory onto its boards **by name** (`mcn/plan.cpp` `make_plan`: `config.joint_index(name)`), refuses an unknown or twice-named joint, and lets a plan name a **subset** of the joints (the rest keep holding). I ported that onto `lk::In<rm::JointTrajectory>`.

## Contract sections

§7.4 ("Sample-major types (JointTrajectory) are read-only views: std::span<const double> per field"), §7.1 (the wire table), §10 T1 (the swap must not change behaviour).

## Repro

```bash
grep -n "struct JointTrajectory" -A10 /home/bam/like_keras/cpp/include/like_keras/msgs/trajectory.hpp
grep -n "joint_names and limits" /home/bam/like_keras/src/like_keras/codec.py
```

```
13:struct JointTrajectory {
...
17:  std::span<const double> t{};
18:  Header header{};
19:  std::span<const double> positions{};
20:  std::optional<std::span<const double>> velocities{};
21:  std::optional<std::span<const double>> accelerations{};
22:  std::optional<std::span<const double>> effort{};
302:        if declaration:  # maps (Header.meta, channels), joint_names and limits of a plan stay Python-side in v0
```

## What I did instead

- The C++ port (`lk/arm_controller.cpp` `MoteusChain::admit`, on bam_ws branch `claude/cpp-controller-nodes-f9a521`) maps **by column**: a trajectory must have exactly the node's joints, in config order. Any other column count is dropped and counted (`plans_dropped`). Subsets are gone.
- The Python peer (`lk/arm_controller_py.py` `make_plan`) **can** read `joint_names`, so it also refuses names that are not the node's, in order.
- So the two placements **diverge on the same input**. A planner that sends the right 7 joints in the wrong order moves the wrong boards under `same_box`, and is refused under `fallback`. The swap test (`lk/tests/test_swap.py`) passes only because its planner sends the config order. A swap that changes safety behaviour is what T1 exists to prevent.

## Proposal

- **Library fix (codec generator, lane core).** Emit `joint_names` in sample-major views:
  - `std::size_t joints = 0; std::array<std::string_view, lk::kMaxJoints> joint_names{};`
  - the views point into the received utf8 buffer, valid for the step, like the spans.
  - The decode cost is J string views per message.
- **New ruler test.** A C++ echo box that reads `joint_names` of a pushed `JointTrajectory` and republishes them, e.g. as a `JointState` name. Compare in-process and over dora, with a non-ASCII name (the golden frame already has one).
- **Contract.** §7.4's sentence on sample-major types gains "and their per-joint names".

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).
