#!/usr/bin/env python3

"""
Examples demonstrating MultiBinary, Text, and Sequence distributions.

Shows how to use these gymnasium-style spaces with messages and data types.
"""

# BAM
from lk.msgs.random_msgs import (
    MultiBinaryDist,
    TextDist,
    SequenceDist,
    FloatDist,
    IntDist,
)
from lk.msgs.ros.geometry_msgs import Point, PoseStamped
import numpy as np


def example_1_multi_binary():
    """Example 1: MultiBinary for button states, flags, etc."""
    print("\n" + "=" * 70)
    print("Example 1: MultiBinary - Button States and Flags")
    print("=" * 70)

    # Single button state (pressed/not pressed)
    print("\n[1.1] Single button (70% press probability)")
    button_dist = MultiBinaryDist.single(p=0.7).with_seed(42)

    for i in range(5):
        state = button_dist.sample()
        print(f"  Frame {i+1}: Button {'PRESSED' if state else 'released'}")

    # Multiple buttons
    print("\n[1.2] Controller with 8 buttons")
    controller_dist = MultiBinaryDist.array(8, p=0.3).with_seed(42)

    button_names = ["A", "B", "X", "Y", "L", "R", "Start", "Select"]
    for frame in range(3):
        states = controller_dist.sample()
        pressed = [button_names[i] for i, state in enumerate(states) if state]
        print(f"  Frame {frame+1}: {pressed if pressed else 'No buttons pressed'}")

    # Object detection flags (has object, is visible, is graspable)
    print("\n[1.3] Object detection flags")
    object_flags_dist = MultiBinaryDist.array(3, p=0.5).with_seed(42)
    flag_names = ["has_object", "is_visible", "is_graspable"]

    for i in range(3):
        flags = object_flags_dist.sample()
        status = {flag_names[j]: bool(flag) for j, flag in enumerate(flags)}
        print(f"  Detection {i+1}: {status}")


def example_2_text():
    """Example 2: Text for commands, labels, messages."""
    print("\n" + "=" * 70)
    print("Example 2: Text - Commands, Labels, and Messages")
    print("=" * 70)

    # Robot commands
    print("\n[2.1] Robot commands")
    command_dist = TextDist.discrete(
        ["move_forward", "turn_left", "turn_right", "stop", "pick_up", "place_down"]
    ).with_seed(42)

    print("  Command sequence:")
    for i in range(5):
        cmd = command_dist.sample()
        print(f"    {i+1}. {cmd}")

    # Object labels
    print("\n[2.2] Object classification labels")
    label_dist = TextDist.discrete(
        ["cup", "bottle", "book", "pen", "phone", "keys"]
    ).with_seed(42)

    print("  Detected objects:")
    for i in range(5):
        label = label_dist.sample()
        print(f"    Frame {i+1}: {label}")

    # Random IDs
    print("\n[2.3] Random alphanumeric IDs")
    id_dist = TextDist.random_alphanumeric(6, 8).with_seed(42)

    print("  Generated IDs:")
    for i in range(5):
        id_str = id_dist.sample()
        print(f"    ID-{id_str}")

    # Status messages with templates
    print("\n[2.4] Status messages (template)")
    status_dist = TextDist.template(
        "Robot {robot_id}: Task {task_status} at position ({x}, {y})",
        {
            "robot_id": TextDist.discrete(["R1", "R2", "R3"]),
            "task_status": TextDist.discrete(["completed", "in_progress", "failed"]),
            "x": TextDist.discrete(["0.5", "1.2", "2.3"]),
            "y": TextDist.discrete(["1.0", "2.0", "3.0"]),
        },
    ).with_seed(42)

    print("  Status updates:")
    for i in range(5):
        status = status_dist.sample()
        print(f"    {status}")


def example_3_sequence():
    """Example 3: Sequence for variable-length data."""
    print("\n" + "=" * 70)
    print("Example 3: Sequence - Variable-Length Lists")
    print("=" * 70)

    # Waypoint sequence
    print("\n[3.1] Variable number of waypoints (1-5)")
    waypoint_dist = SequenceDist.uniform_length(
        min_len=1, max_len=5, element_dist=FloatDist.uniform(-10, 10)
    ).with_seed(42)

    print("  Waypoint paths:")
    for i in range(3):
        waypoints = waypoint_dist.sample()
        print(
            f"    Path {i+1} ({len(waypoints)} points): {[f'{w:.2f}' for w in waypoints]}"
        )

    # Detected objects (variable count)
    print("\n[3.2] Detected object IDs (0-5 objects)")
    detection_dist = SequenceDist.uniform_length(
        min_len=0, max_len=5, element_dist=IntDist.uniform(100, 200)
    ).with_seed(42)

    print("  Detection frames:")
    for i in range(4):
        detected_ids = detection_dist.sample()
        if detected_ids:
            print(
                f"    Frame {i+1}: Detected {len(detected_ids)} objects: {detected_ids}"
            )
        else:
            print(f"    Frame {i+1}: No objects detected")

    # Action sequence
    print("\n[3.3] Action sequence (2-4 actions)")
    action_seq_dist = SequenceDist.uniform_length(
        min_len=2,
        max_len=4,
        element_dist=TextDist.discrete(["move", "rotate", "grasp", "release"]),
    ).with_seed(42)

    print("  Action plans:")
    for i in range(3):
        actions = action_seq_dist.sample()
        print(f"    Plan {i+1}: {' -> '.join(actions)}")


def example_4_sequence_of_points():
    """Example 4: Sequence of composite messages (Points)."""
    print("\n" + "=" * 70)
    print("Example 4: Sequence of Points - Trajectory")
    print("=" * 70)

    # Trajectory: sequence of 3D points
    print("\n[4.1] Trajectory with 3-6 waypoints")
    trajectory_dist = SequenceDist.uniform_length(
        min_len=3, max_len=6, element_dist=Point.Dist.uniform(-1, 1)
    ).with_seed(42)

    print("  Trajectories:")
    for i in range(3):
        trajectory = trajectory_dist.sample()
        print(f"    Trajectory {i+1} ({len(trajectory)} points):")
        for j, point in enumerate(trajectory):
            print(f"      {j+1}. [{point.x:.2f}, {point.y:.2f}, {point.z:.2f}]")


def example_5_combined():
    """Example 5: Combining different space types."""
    print("\n" + "=" * 70)
    print("Example 5: Combined - Robot State with Multiple Space Types")
    print("=" * 70)

    # Simulate a robot state with multiple components
    print("\n[5.1] Complete robot state")

    # Component distributions
    joint_states_dist = SequenceDist.fixed_length(6, FloatDist.uniform(-3.14, 3.14))
    gripper_state_dist = MultiBinaryDist.single(p=0.5)
    mode_dist = TextDist.discrete(["idle", "moving", "grasping", "placing"])
    target_positions_dist = SequenceDist.uniform_length(
        min_len=0, max_len=3, element_dist=Point.Dist.uniform(-2, 2)
    )

    # Seed all for consistency
    joint_states_dist.seed(42)
    gripper_state_dist.seed(42)
    mode_dist.seed(42)
    target_positions_dist.seed(42)

    print("  Robot states over time:")
    for frame in range(3):
        print(f"\n  Frame {frame+1}:")

        # Sample all components
        joint_states = joint_states_dist.sample()
        gripper_closed = gripper_state_dist.sample()
        mode = mode_dist.sample()
        targets = target_positions_dist.sample()

        print(f"    Mode: {mode}")
        print(f"    Joint states: {[f'{j:.2f}' for j in joint_states]}")
        print(f"    Gripper: {'CLOSED' if gripper_closed else 'OPEN'}")
        print(f"    Targets ({len(targets)}): ", end="")
        if targets:
            print([f"[{p.x:.1f}, {p.y:.1f}, {p.z:.1f}]" for p in targets])
        else:
            print("None")


def example_6_interface_specs():
    """Example 6: Using these as interface specifications."""
    print("\n" + "=" * 70)
    print("Example 6: Interface Specifications")
    print("=" * 70)

    # Define interface specs
    print("\n[6.1] Component interface specifications:")

    specs = {
        "buttons": MultiBinaryDist.array(8),
        "command": TextDist.discrete(["start", "stop", "pause", "resume"]),
        "waypoints": SequenceDist.uniform_length(2, 10, Point.Dist.uniform(0, 5)),
        "joint_positions": SequenceDist.fixed_length(6, FloatDist.uniform(-3.14, 3.14)),
    }

    print("\nInterface:")
    for name, spec in specs.items():
        range_info = spec.get_range()
        print(f"  - {name}: {type(spec).__name__}")
        if isinstance(spec, SequenceDist):
            print(
                f"      Length: {range_info[0]['length']} to {range_info[1]['length']}"
            )
            print(
                f"      Element range: {range_info[0]['element']} to {range_info[1]['element']}"
            )

    # Sample and validate
    print("\n[6.2] Sampling and validation:")
    buttons_spec = specs["buttons"]

    valid_buttons = np.array([1, 0, 1, 0, 0, 1, 0, 1])
    invalid_buttons = np.array([1, 0, 2, 0, 0])  # Wrong value and length

    print(f"  Valid buttons {valid_buttons}: {buttons_spec.contains(valid_buttons)}")
    print(
        f"  Invalid buttons {invalid_buttons}: {buttons_spec.contains(invalid_buttons)}"
    )


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("MultiBinary, Text, and Sequence Distribution Examples")
    print("Gymnasium-like Space API for Messages")
    print("=" * 70)

    example_1_multi_binary()
    example_2_text()
    example_3_sequence()
    example_4_sequence_of_points()
    example_5_combined()
    example_6_interface_specs()

    print("\n" + "=" * 70)
    print("All examples completed!")
    print("=" * 70)
    print("\nKey takeaways:")
    print("  - MultiBinaryDist: Button states, flags, binary arrays")
    print("  - TextDist: Commands, labels, messages, templates")
    print("  - SequenceDist: Variable-length lists with consistent element types")
    print("  - All inherit from DataDist with gymnasium Space interface")
    print("  - Can combine with Point.Dist and other message distributions")
    print("=" * 70 + "\n")
