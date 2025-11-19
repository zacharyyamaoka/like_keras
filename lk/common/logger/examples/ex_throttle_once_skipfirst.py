#!/usr/bin/env python3

"""
    Throttle, Once, and Skip First Example
    
    Demonstrates ROS 2-style logging features:
    - throttle_duration_sec: Limit message frequency
    - once: Log only the first time
    - skip_first: Skip the first occurrence
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import Logger
    
    # PYTHON
    import time
    
    print("=" * 70)
    print("BAM Logger - Throttle/Once/Skip First Example")
    print("=" * 70)
    
    # Step 1: Create logger with backend
    print("\n1. Setting up logger with Python backend...")
    logger = Logger(run_id="throttle_test", robot_id="BAM-01")
    logger.get_python_backend(level="INFO")
    print("   ✓ Logger ready\n")
    
    # Step 2: Demonstrate once (only logs first time)
    print("2. Testing 'once' (only logs first time):\n")
    for i in range(5):
        logger.info(f"Loop iteration {i}", once=True, feature="once")
        time.sleep(0.1)
    print("   ✓ Only first iteration logged\n")
    
    # Step 3: Demonstrate skip_first (skips first, logs rest)
    print("3. Testing 'skip_first' (skips first occurrence):\n")
    for i in range(5):
        logger.info(f"Skip first iteration {i}", skip_first=True, feature="skip_first")
        time.sleep(0.1)
    print("   ✓ First iteration skipped, rest logged\n")
    
    # Step 4: Demonstrate throttle (rate limiting)
    print("4. Testing 'throttle_duration_sec' (rate limiting to 0.5s):\n")
    for i in range(10):
        logger.info(
            f"Throttled message {i}",
            throttle_duration_sec=0.5,
            feature="throttle"
        )
        time.sleep(0.1)  # Try to log every 100ms, but throttled to 500ms
    print("   ✓ Messages throttled to 1 per 0.5 seconds\n")
    
    # Step 5: Combine with context
    print("5. Combining throttle with context injection:\n")
    for i in range(10):
        logger.info(
            "High-frequency control loop",
            throttle_duration_sec=1.0,
            iteration=i,
            cpu_percent=45.2 + i,
            feature="throttle_with_context"
        )
        time.sleep(0.2)
    print("   ✓ Context included with throttled messages\n")
    
    # Step 6: Multiple call sites
    print("6. Multiple call sites (tracked independently):\n")
    for i in range(3):
        logger.info("Call site A", once=True, site="A")
        logger.info("Call site B", once=True, site="B")
        time.sleep(0.1)
    print("   ✓ Each call site tracked independently\n")
    
    # Summary
    print("=" * 70)
    print("SUCCESS! ROS 2-style logging features work:")
    print("  ✓ once: Logs only the first time")
    print("  ✓ skip_first: Skips first, logs rest")
    print("  ✓ throttle_duration_sec: Rate limits messages")
    print("  ✓ Call sites tracked independently")
    print("  ✓ Works with context injection")
    print("\nUse cases:")
    print("  - once: One-time initialization messages")
    print("  - skip_first: Calibration/warmup phases")
    print("  - throttle: High-frequency control loops")
    print("=" * 70)

