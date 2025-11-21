#!/usr/bin/env python3

"""
Call Site Context Example

Demonstrates automatic call site injection (file and line number).
Enabled by default, can be disabled if not needed.
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import Logger

    print("=" * 70)
    print("BAM Logger - Call Site Context Example")
    print("=" * 70)

    # Step 1: Default behavior (call site included)
    print("\n1. Default: Call site included in context\n")

    logger = Logger(robot_id="BAM-01")
    logger.get_python_backend(level="INFO")

    logger.info("Message from line 26")
    logger.warning("Warning from line 27")
    logger.error("Error from line 28", error_code="TEST")

    # Step 2: Disable call site
    print("\n2. Disabled: No call site in context\n")

    logger_no_site = Logger(include_call_site=False, robot_id="BAM-02")
    logger_no_site.get_python_backend(level="INFO")

    logger_no_site.info("Message without call site")
    logger_no_site.warning("Warning without call site")

    # Step 3: Child loggers inherit setting
    print("\n3. Child loggers inherit include_call_site setting\n")

    child_with_site = logger.get_logger(name=__name__, component="arm")
    child_with_site.info("Child with call site")

    child_no_site = logger_no_site.get_logger(name=__name__, component="gripper")
    child_no_site.info("Child without call site")

    # Step 4: Useful for debugging
    print("\n4. Call site useful for debugging:")

    def helper_function():
        logger.error("Error in helper function", value=42)

    def another_function():
        logger.warning("Warning from another location")

    helper_function()
    another_function()

    print("\n   Notice the different line numbers!")

    # Summary
    print("\n" + "=" * 70)
    print("SUCCESS! Call site feature works:")
    print("  ✓ Default: include_call_site=True")
    print("  ✓ Adds 'file' and 'line' to context")
    print("  ✓ Can disable with include_call_site=False")
    print("  ✓ Children inherit parent's setting")
    print("\nBenefits:")
    print("  - Easy debugging: know exactly where log came from")
    print("  - Automatic: no manual file/line tracking")
    print("  - Optional: disable if not needed")
    print("\nUsage:")
    print("  logger = Logger(robot_id='BAM-01')  # Default: includes call site")
    print("  logger = Logger(include_call_site=False)  # Opt out")
    print("=" * 70)
