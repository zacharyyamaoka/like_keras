#!/usr/bin/env python3

"""
Logger Registry Example

Demonstrates the global logger registry (like Python logging.getLogger()).
Same name returns same instance, enabling level management and introspection.
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import get_logger, get_logger_registry, set_logger_level

    print("=" * 70)
    print("BAM Logger - Registry Example")
    print("=" * 70)

    # Step 1: Create loggers
    print("\n1. Creating loggers (registered automatically)...\n")

    robot = get_logger(name="robot")
    robot.info("Robot logger")

    arm = robot.get_logger(name="arm")
    arm.info("Arm logger")

    gripper = robot.get_logger(name="gripper")
    gripper.info("Gripper logger")

    joints = arm.get_logger(name="joints")
    joints.info("Joints logger")

    # Step 2: Same name returns same instance
    print("\n2. Same name returns same instance (like logging.getLogger()):\n")

    arm2 = get_logger(name="robot.arm")
    print(f"   arm is arm2: {arm is arm2}")
    print(f"   Same memory address: {id(arm) == id(arm2)}")

    # Step 3: Inspect registry
    print("\n3. Logger registry (get_logger_registry()):\n")

    registry = get_logger_registry()
    print(f"   Total loggers: {len(registry)}")
    for name in sorted(registry.keys()):
        print(f"   - {name}")

    # Step 4: Set level for specific logger
    print("\n4. Set level for specific logger:\n")

    print("   Before: arm at INFO level")
    arm.info("Info message visible")
    arm.debug("Debug message hidden")

    print("\n   Setting robot.arm to DEBUG...")
    set_logger_level(name="robot.arm", level="DEBUG")

    arm.debug("Debug now visible!")
    arm.info("Info still visible")

    # Step 5: Set level for all loggers
    print("\n5. Set level for ALL loggers:\n")

    print("   Setting all loggers to WARNING...")
    set_logger_level(level="WARNING")

    robot.info("Info hidden")
    robot.warning("Warning visible")
    gripper.info("Info hidden")
    gripper.warning("Warning visible")

    # Summary
    print("\n" + "=" * 70)
    print("SUCCESS! Logger registry works:")
    print("  ✓ Global registry (like Python logging)")
    print("  ✓ Same name = same instance")
    print("  ✓ Hierarchical names (robot.arm.joints)")
    print("  ✓ get_logger_registry() for introspection")
    print("  ✓ set_logger_level(name, level) for specific logger")
    print("  ✓ set_logger_level(level=level) for all loggers")
    print("\nJust like logging.getLogger():")
    print("  logger = get_logger(name='robot.arm')")
    print("  logger2 = get_logger(name='robot.arm')  # Same instance!")
    print("=" * 70)
