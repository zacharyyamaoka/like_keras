#!/usr/bin/env python3
"""
Quick launcher for LK Viewer

Usage:
    python start_viewer.py                    # View simple_rr_robot.py
    python start_viewer.py path/to/robot.py   # View custom robot
"""

# PYTHON
import sys
import subprocess
from pathlib import Path


def main():
    robot_file = "simple_rr_robot.py"

    if len(sys.argv) > 1:
        robot_file = sys.argv[1]

    robot_path = Path(robot_file).resolve()

    if not robot_path.exists():
        print(f"❌ Error: Robot file not found: {robot_path}")
        sys.exit(1)

    print(f"🚀 Starting LK Viewer...")
    print(f"📍 Watching: {robot_path}")
    print(f"🌐 Open: http://localhost:8000")
    print(f"✏️  Edit {robot_path.name} to see live updates!")
    print()

    # Set environment variable for the server to know which file to watch
    import os

    os.environ["ROBOT_FILE"] = str(robot_path)

    # Start the server
    try:
        subprocess.run([sys.executable, "viewer_server.py"], check=True)
    except KeyboardInterrupt:
        print("\n👋 Shutting down viewer...")


if __name__ == "__main__":
    main()
