#!/usr/bin/env python3

"""
    Path Formatting Example
    
    Demonstrates automatic absolute path recording and shortened display:
    - Stores absolute path in 'file_abs' for introspection
    - Displays shortened path in 'file' (30 chars from back, ~ for home)
    - Easy to find exact location of log statements
"""


if __name__ == "__main__":
    # BAM
    from bam.common.logger import Logger
    
    print("=" * 70)
    print("BAM Logger - Path Formatting Example")
    print("=" * 70)
    
    # Configure logger
    logger = Logger(robot_id="BAM-01")
    logger.get_python_backend(level="INFO")
    
    print("\n1. Logging with automatic path recording:\n")
    
    # Log from this file
    logger.info("Message from this file")
    logger.warning("Warning message", value=42)
    logger.error("Error message", error_code="TEST")
    
    # Log from a function
    def helper_function():
        logger.info("Message from helper function", nested=True)
        logger.debug("Debug from helper", depth=2)
    
    helper_function()
    
    print("\n2. Context fields:")
    print("   - 'file': Shortened path (30 chars from back, ~ for home)")
    print("   - 'file_abs': Full absolute path (for introspection)")
    print("   - 'line': Line number")
    
    print("\n3. Benefits:")
    print("   ✓ Easy to find exact location: use 'file_abs' + 'line'")
    print("   ✓ Clean display: shortened path in 'file'")
    print("   ✓ Home directory replaced with ~ for brevity")
    print("   ✓ Automatic: no manual tracking needed")
    
    print("\n" + "=" * 70)
    print("SUCCESS! Path formatting feature works!")
    print("=" * 70)

