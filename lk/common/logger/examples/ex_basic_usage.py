#!/usr/bin/env python3

"""
    Basic Logger Usage Example
    
    Demonstrates simple logging at various levels with a Python backend.
    Shows the clean import pattern: from bam.common.logger import Logger
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import Logger
    
    print("=" * 60)
    print("BAM Logger - Basic Usage Example")
    print("=" * 60)
    
    # Step 1: Create logger
    print("\n1. Creating logger...")
    logger = Logger()
    print("   ✓ Logger created")
    
    # Step 2: Attach Python backend
    print("\n2. Attaching Python logging backend...")
    logger.get_python_backend(name="basic_example", level="DEBUG")
    print("   ✓ Backend attached")
    
    # Step 3: Log at various levels
    print("\n3. Logging messages at various levels:\n")
    
    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")
    logger.critical("This is a critical message")
    
    # Summary
    print("\n" + "=" * 60)
    print("SUCCESS! Basic logging works:")
    print("  - Logger created and backend attached")
    print("  - Messages logged at all levels")
    print("  - Clean import pattern: from bam.common.logger import Logger")
    print("=" * 60)

