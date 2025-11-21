#!/usr/bin/env python3

"""
Interactive URDF Joint Control Example

Demonstrates bidirectional control between GUI sliders and external code:
- Manual control: Use sliders to move robot joints
- External control: Code can update joints, sliders auto-sync
- Lock feature: Checkbox to prevent external updates during manual control
- Reset button: Return to initial configuration

This example shows how you can seamlessly switch between manual control
and programmatic control without conflicts.
"""

if __name__ == "__main__":
    # BAM
    from bam_artist import RobotArtist
    from bam_descriptions import UR
    import bam.msgs.visual_objects as viz

    # PYTHON
    import numpy as np
    import time
    import threading

    # Create robot and artist
    ur5e = UR.make_UR5e()
    artist = RobotArtist(ur5e)
    artist.attach_viser_viewer()

    # Draw robot
    artist.draw_urdf()

    # Add joint control GUI
    artist.draw_urdf_joint_control_gui(
        name="Joint Control",
        initial_lock=False,  # Allow external updates by default
        show_reset_button=True,
    )

    # Add grid and frame
    artist.draw(viz.Grid())

    print("\n" + "=" * 60)
    print("URDF Joint Control Demo")
    print("=" * 60)
    print("\nFeatures:")
    print("  1. Use sliders to manually control joints")
    print("  2. Code below will periodically update joints (watch sliders sync!)")
    print("  3. Check 'Lock URDF Control' to prevent external updates")
    print("  4. Click 'Reset to Initial' to return to start position")
    print("\nThe sliders and code can control the robot simultaneously!")
    print("=" * 60 + "\n")

    # Background thread that periodically updates the robot
    # This demonstrates external control while GUI is active
    def external_control_demo():
        """Simulate external code updating the robot periodically."""
        time.sleep(5)  # Wait a bit before starting

        # Create a simple sine wave motion for joint 1
        t = 0.0
        while True:
            # Generate a new configuration with sine wave on joint 1
            q = np.array(ur5e.q_initial)
            q[1] = np.sin(t) * 0.5  # Shoulder joint oscillates

            # Update the URDF (this will auto-sync the sliders if not locked)
            artist.draw_urdf(q.tolist())

            time.sleep(0.1)
            t += 0.1

    # Start background thread (daemon so it exits when main thread exits)
    control_thread = threading.Thread(target=external_control_demo, daemon=True)
    control_thread.start()

    # Run viewer (blocks until Ctrl+C)
    try:
        artist.render(block=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
