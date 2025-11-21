#!/usr/bin/env python3

"""
Example: Multi-Backend Advanced Usage

Demonstrates the power of the generic logger with multiple backends:
- Python backend: Console and file logging
- Foxglove backend: Live visualization and MCAP recording
- Artist backend: 3D visual object rendering

All from a single logger interface! This shows how you can:
1. Log string messages → Goes to Python and Foxglove
2. Log visual objects → Goes to Artist backend
3. Log Foxglove schemas → Goes to Foxglove backend

Requirements:
    - foxglove-sdk (pip install foxglove-sdk)
    - bam_artist
    - visual_objects
"""

# BAM
from bam.common.logger import Logger
import bam.msgs.visual_objects as viz
from bam_artist import Artist
from bam.msgs import Pose, Point, Quaternion

# PYTHON
import time
import numpy as np


def robot_simulation():
    """Simulate a robot task with mixed logging."""

    print("=" * 70)
    print("Multi-Backend Advanced Example")
    print("=" * 70)
    print()
    print("This demo shows a unified logger routing to multiple backends:")
    print("  • Python backend → Console output")
    print("  • Foxglove backend → ws://localhost:8765 + /tmp/robot_sim.mcap")
    print("  • Artist backend → http://localhost:8080 (Viser viewer)")
    print()
    print("Open Foxglove and Viser to see messages and visuals in real-time!")
    print()

    # Setup: Create artist
    artist = Artist()
    artist.attach_viser_viewer(verbose=False)

    # Setup: Create logger with all backends
    logger = Logger(run_id="robot_sim_001", robot_id="BAM-01", task="pick_and_place")

    # Attach all backends
    logger.get_python_backend(level="INFO", to_file="/tmp/robot_sim.log")
    logger.get_foxglove_backend(
        server_enabled=True,
        mcap_file="/tmp/robot_sim.mcap",
        server_port=8765,
        verbose=False,
    )
    logger.get_artist_backend(artist=artist, verbose=False)

    print("=" * 70)
    print("Starting robot simulation...")
    print("=" * 70)
    print()

    # Phase 1: Initialization
    logger.info("=== Phase 1: Initialization ===", phase=1)

    # Add workspace visualization
    grid = viz.Grid(cell_size=0.2, num_cells=20)
    logger.info(grid, topic="/viz/workspace/grid")

    # Add table
    table_pose = Pose(
        position=Point(x=0.5, y=0.0, z=-0.05),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    table = viz.Box(
        pose=table_pose,
        dimensions=(1.0, 0.8, 0.1),
        color=viz.RGBA.from_rgb(139, 69, 19, alpha=200),
        name="table",
    )
    logger.info(table, topic="/viz/workspace/table")
    logger.info("Workspace setup complete", objects_added=2)

    time.sleep(1)

    # Phase 2: Object detection
    logger.info("=== Phase 2: Object Detection ===", phase=2)

    detected_objects = []
    for i in range(3):
        x = 0.3 + i * 0.2
        y = -0.2 + i * 0.15

        obj_pose = Pose(
            position=Point(x=x, y=y, z=0.05),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        color = viz.RGBA.from_hsl(h=i / 3.0, s=0.8, l=0.6)
        obj_box = viz.Box(
            pose=obj_pose,
            dimensions=(0.08, 0.08, 0.08),
            color=color,
            name=f"object_{i}",
        )
        logger.info(obj_box, topic=f"/viz/objects/obj_{i}")

        detected_objects.append((x, y))
        logger.info(
            f"Object {i} detected",
            topic="/detection/results",
            object_id=i,
            position_x=x,
            position_y=y,
            confidence=0.95,
        )

        time.sleep(0.5)

    logger.info("Object detection complete", object_count=len(detected_objects))

    time.sleep(1)

    # Phase 3: Motion planning
    logger.info("=== Phase 3: Motion Planning ===", phase=3)

    # Plan path to first object
    start_pose = Pose(
        position=Point(x=0.0, y=-0.3, z=0.3),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    target_x, target_y = detected_objects[0]

    # Generate smooth path
    num_waypoints = 10
    waypoint_poses = []
    for i in range(num_waypoints):
        t = i / (num_waypoints - 1)
        x = start_pose.position.x + t * (target_x - start_pose.position.x)
        y = start_pose.position.y + t * (target_y - start_pose.position.y)
        z = start_pose.position.z + 0.1 * np.sin(t * np.pi)

        pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        waypoint_poses.append(pose)

    # Visualize planned path
    planned_path = viz.Path(
        name="planned_path",
        pose_list=waypoint_poses,
        frame_props=viz.Frame(axis_length=0.04),
        line_props=viz.LineSegments(colors=[viz.RGBA.green()], line_width=3.0),
    )
    logger.info(planned_path, topic="/viz/planning/path")
    logger.info(
        "Path planned",
        topic="/planning/result",
        waypoints=num_waypoints,
        distance_m=0.5,
        planning_time_ms=45,
    )

    time.sleep(2)

    # Phase 4: Execution
    logger.info("=== Phase 4: Path Execution ===", phase=4)

    for i, pose in enumerate(waypoint_poses):
        # Show current position
        current_frame = viz.Frame.from_Pose(pose, axis_length=0.1, name="end_effector")
        logger.info(current_frame, topic="/viz/robot/end_effector")

        # Log progress
        progress_pct = (i + 1) / len(waypoint_poses) * 100
        logger.info(
            f"Waypoint {i+1}/{len(waypoint_poses)}",
            topic="/execution/progress",
            progress_pct=progress_pct,
            throttle_duration_sec=0.2,
        )

        time.sleep(0.2)

    logger.info("Path execution complete", phase=4)

    time.sleep(1)

    # Phase 5: Completion
    logger.info("=== Phase 5: Task Complete ===", phase=5)
    logger.info(
        "Pick and place task completed successfully",
        topic="/task/result",
        success=True,
        total_time_s=time.time(),
        objects_moved=1,
    )

    print("\n" + "=" * 70)
    print("Simulation complete!")
    print("=" * 70)
    print()
    print("Check the outputs:")
    print("  • Python log: /tmp/robot_sim.log")
    print("  • MCAP file: /tmp/robot_sim.mcap (open in Foxglove)")
    print("  • Viser viewer: http://localhost:8080")
    print()
    print("Press Ctrl+C to exit...")

    # Keep alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    try:
        robot_simulation()
    except ImportError as e:
        print(f"\nError: {e}")
        print("\nMissing dependencies. Install with:")
        print("  pip install foxglove-sdk")
        print("  # + ensure bam_artist, visual_objects are available")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()
