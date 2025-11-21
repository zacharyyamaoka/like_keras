#!/usr/bin/env python3

"""
Example: Viser Interactive Controls

Demonstrates the control panels for PinViser:
- Joint position control (sliders)
- Inertial properties (COM spheres and inertia boxes)
- Visual appearance (opacity and color)
- Collision visualization (show/hide and opacity)
- Frame control (size and labels for links and joints)

Note: By default, PinViser automatically adds all control panels.
This example uses auto_add_controls=False to demonstrate manual control addition.
"""

if __name__ == "__main__":

    # PYTHON
    import numpy as np

    # BAM
    from bam.utils.pin_utils import PinViser, PinRobotModel
    from robot_descriptions.loaders.pinocchio import load_robot_description

    # Load a robot description
    # Example: UR5 robot
    rd = load_robot_description("ur5_description")  # Interia on wrist is wrong!!!

    from bam.descriptions import UR

    rd = UR.make_UR5e()

    # Create PinViser with auto_add_controls=False to manually add controls
    # (by default, auto_add_controls=True and all controls are added automatically)
    viz = PinViser.from_robot_description(rd, auto_add_controls=False)

    viz.robot_model.inspect()

    # Manually add the control panels (optional when auto_add_controls=False):

    # Add joint position control sliders
    viz.add_robot_control_sliders()

    # Add inertial property controls (COM and inertia)
    viz.add_inertial_control()

    # Add visual appearance controls (opacity and color)
    viz.add_visual_control()

    # Add collision controls (show/hide and opacity)
    viz.add_collision_control()

    # Add frame control (size and labels)
    viz.add_frame_control()

    print("\n" + "=" * 60)
    print("Viser Interactive Controls Demo")
    print("=" * 60)
    print("\nNote: This example uses auto_add_controls=False for demonstration.")
    print("By default, PinViser automatically adds all control panels.")
    print("\nAvailable controls in the Viser GUI:")
    print("  1. Joint Position Control - Sliders for each joint")
    print("  2. Inertial Properties:")
    print("     - Show Center of Mass (red spheres)")
    print("     - Show Inertia (red boxes, 50% opacity)")
    print("     - Inertial Opacity slider")
    print("  3. Visual Appearance:")
    print("     - Show Visual Meshes checkbox (turn visual meshes on/off)")
    print("     - Opacity slider")
    print("     - Color picker")
    print("     - Wireframe Mode checkbox")
    print("  4. Collision:")
    print("     - Show Collision Meshes checkbox")
    print("     - Collision Opacity slider")
    print("  5. Frame Control:")
    print("     - Show Link Frames checkbox (body/link coordinate axes)")
    print("     - Show Joint Frames checkbox (joint coordinate axes)")
    print("     - Frame Axis Length slider")
    print("     - Show Link Frame Labels checkbox (independent)")
    print("     - Show Joint Frame Labels checkbox (independent)")
    print("     - Label size slider")
    print("     - Label font mode (screen/scene)")
    print("     - Label depth test")
    print("\nTo use default behavior (auto-add all controls):")
    print("  viz = PinViser.from_robot_description(rd)")
    print("  # or")
    print("  viz = PinViser.from_robot_description(rd, auto_add_controls=True)")
    print("\nTo manually add controls:")
    print("  viz = PinViser.from_robot_description(rd, auto_add_controls=False)")
    print("  viz.add_robot_control_sliders()  # Add specific panels as needed")
    print("\nOpen your web browser to view the Viser interface")
    print(f"URL: http://localhost:{viz.port}")
    print("=" * 60)

    # Keep the visualizer running
    input("\nPress Enter to exit...")
