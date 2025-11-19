#!/usr/bin/env python3

"""
    Example: Artist Backend for Visual Object Logging
    
    Demonstrates using the Artist backend to:
    - Log visual objects for rendering in viewers (Viser, Meshcat, O3D)
    - Combine string logging (Python backend) with visual logging (Artist backend)
    - Use the same logger interface for both text and visuals
    - Build complex visualizations progressively
    
    Requirements:
        - bam_artist
        - visual_objects
        - viser (for web-based viewing)
    
    Usage:
        1. Run this script
        2. Open browser to http://localhost:8080 (Viser viewer)
        3. Watch visual objects appear as they're logged
"""

# BAM
from bam.common.logger import Logger
import bam.msgs.visual_objects as viz
from bam_artist import Artist
from bam.msgs import Pose, Point, Quaternion

# PYTHON
import numpy as np
import time


def main():
    print("=" * 60)
    print("Artist Backend Example")
    print("=" * 60)
    print()
    print("Instructions:")
    print("1. Script will start Viser viewer on http://localhost:8080")
    print("2. Open that URL in your browser")
    print("3. Watch visual objects appear as they're logged!")
    print()
    
    # Create artist with viewer
    artist = Artist()
    artist.attach_viser_viewer(verbose=True)
    
    # Create logger with both Python and Artist backends
    logger = Logger(run_id="artist_demo", robot_id="BAM-01")
    logger.get_python_backend(level="INFO")
    logger.get_artist_backend(artist=artist, verbose=True)
    
    print("\n" + "=" * 60)
    print("Logging visual objects...")
    print("=" * 60 + "\n")
    
    # Log string message (goes to Python backend only)
    logger.info("Starting visualization demo")
    
    # Log visual object (goes to Artist backend, rendered in viewer)
    grid = viz.Grid(cell_size=0.5, num_cells=10)
    logger.info(grid, topic="/viz/grid")
    logger.info("Added grid to scene")
    
    time.sleep(0.5)
    
    # Log coordinate frames
    origin = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    frame = viz.Frame.from_Pose(origin, axis_length=0.3, name="origin")
    logger.info(frame, topic="/viz/frames/origin")
    logger.info("Added origin frame")
    
    time.sleep(0.5)
    
    # Log multiple frames in a pattern
    logger.info("Creating frame pattern...")
    for i in range(4):
        angle = i * np.pi / 2
        x = np.cos(angle)
        y = np.sin(angle)
        
        pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        frame = viz.Frame.from_Pose(pose, axis_length=0.15, name=f"frame_{i}")
        logger.info(frame, topic=f"/viz/frames/pattern_{i}")
        
        time.sleep(0.3)
    
    logger.info("Frame pattern complete", frame_count=4)
    
    # Log boxes
    logger.info("Adding boxes...")
    for i in range(3):
        z = 0.1 + i * 0.15
        pose = Pose(
            position=Point(x=0.0, y=0.0, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        color = viz.RGBA.from_hsl(h=i/3.0, s=0.8, l=0.5)
        box = viz.Box(
            pose=pose,
            dimensions=(0.2, 0.2, 0.1),
            color=color,
            name=f"box_{i}"
        )
        logger.info(box, topic=f"/viz/boxes/box_{i}")
        logger.info(f"Added box {i}", height=z)
        
        time.sleep(0.3)
    
    # Log a path (compound visual object)
    logger.info("Creating robot path...")
    waypoint_poses = []
    for i in range(8):
        t = i / 7.0
        x = 1.0 * np.cos(2 * np.pi * t)
        y = 1.0 * np.sin(2 * np.pi * t)
        z = 0.2 + 0.3 * t
        
        pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        waypoint_poses.append(pose)
    
    path = viz.Path(
        name="robot_path",
        pose_list=waypoint_poses,
        frame_props=viz.Frame(axis_length=0.08),
        line_props=viz.LineSegments(
            colors=[viz.RGBA.blue()],
            line_width=3.0
        )
    )
    logger.info(path, topic="/viz/paths/robot_path")
    logger.info("Path complete", waypoint_count=len(waypoint_poses))
    
    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)
    print()
    print("Visual objects are now in the viewer at http://localhost:8080")
    print("The script will keep running so you can interact with the viewer.")
    print("Press Ctrl+C to exit.")
    print()
    
    # Keep alive for viewing
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    try:
        main()
    except ImportError as e:
        print(f"\nError: {e}")
        print("\nMissing dependency. Ensure these packages are installed:")
        print("  - bam_artist")
        print("  - visual_objects")
        print("  - viser")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()

