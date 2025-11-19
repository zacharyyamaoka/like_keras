#!/usr/bin/env python3

"""
    Global Pattern Example
    
    Demonstrates using configure() and get_logger() without creating root manually.
    Just like logging.getLogger() - no need to pass root around!
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import configure, get_logger
    
    print("=" * 70)
    print("BAM Logger - Global Pattern Example")
    print("=" * 70)
    
    # Step 1: Configure once (typically in main.py)
    print("\n1. Configuring global logger once...")
    configure(
        py=True,
        py_level="INFO",
        robot_id="BAM-01",
        run_id="exp_001"
    )
    print("   ✓ Global configuration set")
    
    # Step 2: Get loggers anywhere (no root needed!)
    print("\n2. Getting loggers in different 'modules':\n")
    
    # Simulate different modules getting their loggers
    arm_logger = get_logger(name=__name__, component="arm")
    arm_logger.info("Arm initialized", dof=6)
    
    gripper_logger = get_logger(name=__name__, component="gripper")
    gripper_logger.info("Gripper initialized", max_force_n=100)
    
    planner_logger = get_logger(name=__name__, component="planner")
    planner_logger.info("Planner ready", algorithm="RRT*")
    
    # Step 3: Nested children still work
    print("\n3. Creating nested child loggers:\n")
    
    joints = arm_logger.get_logger(name="joints", subsystem="joints")
    joints.info("Joints calibrated", count=6)
    
    # Step 4: All inherit global context
    print("\n4. All loggers inherit global context (robot_id, run_id)")
    print("   Notice all messages above include robot_id and run_id!")
    
    # Summary
    print("\n" + "=" * 70)
    print("SUCCESS! Global pattern works:")
    print("  ✓ configure() once in main.py")
    print("  ✓ get_logger() anywhere (no root to pass around!)")
    print("  ✓ Automatic backend setup")
    print("  ✓ Global context inheritance")
    print("\nJust like logging.getLogger():")
    print("  - No need to create root logger yourself")
    print("  - No need to pass root around")
    print("  - Just call get_logger() anywhere")
    print("\nUsage:")
    print("  # main.py")
    print("  from bam.common.logger import configure")
    print("  configure(py=True, robot_id='BAM-01')")
    print("\n  # any_module.py")
    print("  from bam.common.logger import get_logger")
    print("  logger = get_logger(name=__name__, component=__name__)")
    print("  logger.info('Ready!')")
    print("=" * 70)

