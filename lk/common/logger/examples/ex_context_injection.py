#!/usr/bin/env python3

"""
    Context Injection Example
    
    Demonstrates flexible context management:
    - Global context set once at logger creation
    - Per-call context via kwargs
    - Context updates during runtime
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import Logger
    
    print("=" * 60)
    print("BAM Logger - Context Injection Example")
    print("=" * 60)
    
    # Step 1: Create logger with global context
    print("\n1. Creating logger with global context...")
    logger = Logger(run_id="exp_42", robot_id="BAM-01")
    print(f"   Global context: {logger.get_context()}")
    
    # Step 2: Attach backend
    print("\n2. Attaching Python backend...")
    logger.get_python_backend(name="context_example", level="INFO")
    print("   ✓ Backend attached")
    
    # Step 3: Log with global context only
    print("\n3. Logging with global context:\n")
    logger.info("Controller initialized")
    logger.info("System ready")
    
    # Step 4: Log with additional per-call context
    print("\n4. Logging with per-call context:\n")
    logger.info("Motion executed", component="arm.control", duration_ms=123)
    logger.info("Grasp attempted", component="gripper", force_n=15.5, success=True)
    logger.warning("High latency detected", component="network", latency_ms=500)
    
    # Step 5: Update global context
    print("\n5. Updating global context...\n")
    logger.update_context(phase="execution", trial=1)
    logger.info("Starting trial", target_position=[0.5, 0.3, 0.2])
    logger.info("Trial complete", result="success", score=0.95)
    
    # Step 6: Mixed context
    print("\n6. Mixing global and per-call context:\n")
    logger.info(
        "Complex operation completed",
        component="planner",
        nodes_expanded=1523,
        planning_time_ms=45,
        cost=12.3
    )
    
    # Summary
    print("\n" + "=" * 60)
    print("SUCCESS! Context injection works:")
    print("  - Global context: run_id, robot_id, phase, trial")
    print("  - Per-call context: component, duration_ms, etc.")
    print("  - Flexible and composable context management")
    print(f"\nFinal context: {logger.get_context()}")
    print("=" * 60)

