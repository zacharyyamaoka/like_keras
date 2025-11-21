#!/usr/bin/env python3

"""
Multi-Backend Example

Demonstrates logging to multiple backends simultaneously.
Shows how messages are dispatched to all attached backends.
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import Logger

    # PYTHON
    import tempfile
    import os

    print("=" * 60)
    print("BAM Logger - Multi-Backend Example")
    print("=" * 60)

    # Step 1: Create logger with context
    print("\n1. Creating logger with context...")
    logger = Logger(run_id="multi_backend_test", component="main")
    print(f"   Context: {logger.get_context()}")

    # Step 2: Attach multiple Python backends
    print("\n2. Attaching multiple backends...")

    # Console backend (INFO level)
    logger.get_python_backend(name="console", level="INFO")
    print("   ✓ Console backend attached (INFO level)")

    # Debug backend (DEBUG level)
    logger.get_python_backend(name="debug", level="DEBUG")
    print("   ✓ Debug backend attached (DEBUG level)")

    # File backend
    temp_file = os.path.join(tempfile.gettempdir(), "bam_logger_test.log")
    logger.get_python_backend(name="file", level="WARNING", to_file=temp_file)
    print(f"   ✓ File backend attached (WARNING level, file={temp_file})")

    print(f"\n   Active backends: {list(logger.backends.keys())}")

    # Step 3: Log at various levels
    print("\n3. Logging messages (observe different backends filtering):\n")

    logger.debug("Debug message - only debug backend should show this")
    logger.info("Info message - console and debug backends show this")
    logger.warning("Warning message - all backends show this")
    logger.error("Error message - all backends show this", error_code="TEST_ERROR")

    # Step 4: Log with rich context
    print("\n4. Logging with rich context:\n")
    logger.info(
        "Complex operation",
        operation="grasp_planning",
        candidates=25,
        best_score=0.87,
        duration_ms=156,
    )

    logger.warning(
        "Performance degradation", cpu_percent=85.2, memory_mb=2048, latency_ms=350
    )

    # Step 5: Show file contents
    print("\n5. File backend output (WARNING and above):")
    print("   " + "-" * 56)
    if os.path.exists(temp_file):
        with open(temp_file, "r") as f:
            for line in f:
                print("   " + line.rstrip())
    print("   " + "-" * 56)

    # Cleanup
    if os.path.exists(temp_file):
        os.remove(temp_file)
        print(f"\n   ✓ Cleaned up temp file: {temp_file}")

    # Summary
    print("\n" + "=" * 60)
    print("SUCCESS! Multi-backend logging works:")
    print("  - Three backends attached simultaneously")
    print("  - Each backend can have different levels")
    print("  - Messages dispatched to all backends")
    print("  - File output works correctly")
    print("=" * 60)
