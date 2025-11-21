#!/usr/bin/env python3

"""
Child Logger Pattern Example

Demonstrates the simplified pattern where root.get_logger() creates
child loggers that inherit backends and context from the parent.
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import Logger

    print("=" * 70)
    print("BAM Logger - Child Logger Pattern Example")
    print("=" * 70)

    # Step 1: Create and configure root logger once
    print("\n1. Creating root logger with global context...")
    root = Logger(robot_id="BAM-01", run_id="exp_001")
    root.get_python_backend(level="INFO")
    print("   ✓ Root configured with Python backend")

    # Step 2: Get child loggers (inherit everything)
    print("\n2. Creating child loggers:\n")

    arm_logger = root.get_logger(name=__name__, component="arm")
    arm_logger.info("Arm controller initialized", dof=6, payload_kg=5.0)

    gripper_logger = root.get_logger(name=__name__, component="gripper")
    gripper_logger.info("Gripper ready", max_force_n=100)

    planner_logger = root.get_logger(name=__name__, component="planner")
    planner_logger.info("Motion planned", waypoints=25, duration_ms=45)

    # Step 3: Nested children
    print("\n3. Creating nested child loggers:\n")

    joints_logger = arm_logger.get_logger(name="joints", subsystem="joints")
    joints_logger.info("Joint states", positions=[0.1, 0.2, 0.3])

    gripper_sensors = gripper_logger.get_logger(name="sensors", subsystem="sensors")
    gripper_sensors.info("Force sensor reading", force_n=25.5)

    # Step 4: All children inherit parent's backends
    print("\n4. Demonstrating backend inheritance:")
    print("   All messages above used the same Python backend")
    print("   configured on the root logger!")

    # Step 5: Can add backends to root, children inherit
    print("\n5. Adding another backend to root:\n")

    import tempfile
    import os

    log_file = os.path.join(tempfile.gettempdir(), "child_logger_test.log")
    root.get_python_backend(name="file", level="DEBUG", to_file=log_file)

    # Now all children (and new children) use both backends
    vision_logger = root.get_logger(name=__name__, component="vision")
    vision_logger.info("Object detected", object_id="obj_042", confidence=0.95)

    print(f"   ✓ Message also written to {log_file}")

    # Show file contents
    print("\n6. File backend contents:")
    print("   " + "-" * 66)
    if os.path.exists(log_file):
        with open(log_file, "r") as f:
            for line in f:
                print("   " + line.rstrip())
        os.remove(log_file)
    print("   " + "-" * 66)

    # Summary
    print("\n" + "=" * 70)
    print("SUCCESS! Child logger pattern:")
    print("  ✓ Configure root once: root.get_python_backend()")
    print("  ✓ Get children: root.get_logger(name=__name__, component='arm')")
    print("  ✓ Children inherit backends automatically")
    print("  ✓ Children inherit parent context")
    print("  ✓ Can nest: child.get_logger(subsystem='joints')")
    print("  ✓ Simple and explicit - no caching magic")
    print("\nPattern:")
    print("  root = Logger(robot_id='BAM-01')")
    print("  root.get_python_backend()")
    print("  arm = root.get_logger(name=__name__, component='arm')")
    print("  arm.info('Ready!')")
    print("=" * 70)
