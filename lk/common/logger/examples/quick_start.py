#!/usr/bin/env python3

"""
    Quick Start - BAM Logger with Global Singleton Pattern
    
    Shows the recommended usage:
    - Global configuration in bam_logger/__init__.py
    - Just import and use get_logger() anywhere
    - Standardized name field for logger identification
"""


if __name__ == "__main__":
    # BAM - Already configured in __init__.py, just import and use!
    from bam.common.logger import get_logger, configure
    
    print("BAM Logger - Quick Start\n")
    print("=" * 70)
    
    # Step 1: Logger already configured by __init__.py
    print("\n1. Logger auto-configured on import")
    print("   ✓ Python backend enabled")
    print("   ✓ ROBOT_ID from environment (if set)")
    print("   ✓ BAM_LOG_LEVEL from environment (default: INFO)")
    
    # Step 2: Just use get_logger() anywhere!
    print("\n2. Using get_logger() with name field:\n")
    
    arm_logger = get_logger(name="arm_controller", component="arm")
    arm_logger.info("Arm initialized", dof=6)
    
    gripper_logger = get_logger(name="gripper_controller", component="gripper")
    gripper_logger.info("Gripper ready", max_force_n=100)
    
    planner_logger = get_logger(name="motion_planner")
    planner_logger.info("Planner ready", algorithm="RRT*")
    
    # Step 3: Common pattern - use __name__
    print("\n3. Using __name__ for module identification:\n")
    
    logger = get_logger(name=__name__)
    logger.info("Using module name as logger name")
    
    # Step 4: Override config if needed (dev/scripts)
    print("\n4. Override configuration for dev/debugging:\n")
    
    configure(py=True, robot_id="DEV-01", run_id="exp_123", py_level="DEBUG")
    
    dev_logger = get_logger(name="dev_logger")
    dev_logger.debug("Debug now visible")
    dev_logger.info("Using override config")
    
    # Step 5: Nested children
    print("\n5. Nested child loggers:\n")
    
    joints = arm_logger.get_logger(name="joint_controller", subsystem="joints")
    joints.info("Joints calibrated", count=6)
    
    # Step 6: ROS 2-style features
    print("\n6. ROS 2-style features:\n")
    
    # Once
    logger.info("Init complete", once=True)
    logger.info("Init complete", once=True)  # Won't log
    
    # Throttle
    import time
    for i in range(3):
        logger.info("Control loop", throttle_duration_sec=0.5, iteration=i)
        time.sleep(0.1)
    
    # Summary
    print("\n" + "=" * 70)
    print("SUCCESS! BAM Logger:")
    print("  ✓ Auto-configured in __init__.py")
    print("  ✓ Just import and use - no setup needed")
    print("  ✓ Standardized name field")
    print("  ✓ Can override for dev/scripts")
    print("\nRecommended usage:")
    print("  # In any module - just import and use!")
    print("  from bam.common.logger import get_logger")
    print("  logger = get_logger(name=__name__)")
    print("  logger.info('Ready!')")
    print("\n  # Override in dev/scripts if needed")
    print("  from bam.common.logger import configure")
    print("  configure(py=True, robot_id='DEV-01', py_level='DEBUG')")
    print("=" * 70)
