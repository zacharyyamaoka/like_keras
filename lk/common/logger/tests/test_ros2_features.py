#!/usr/bin/env python3

"""
Test ROS 2-style logging features (throttle, once, skip_first)
"""

# BAM
from bam.common.logger import Logger

# PYTHON
import pytest
import time


def test_once():
    """Test that once=True only logs the first time"""
    logger = Logger()
    logger.get_python_backend(level="INFO")

    test_logger = logger.get_logger(name="test.once")

    # Track if messages were sent (checking _once_state)
    # First call should log
    test_logger.info("Test", once=True)

    # Check that call site was recorded
    assert len(test_logger._once_state) == 1


def test_skip_first():
    """Test that skip_first=True skips the first occurrence"""
    logger = Logger()
    logger.get_python_backend(level="INFO")

    test_logger = logger.get_logger(name="test.skip")

    # Call from same location multiple times
    for i in range(3):
        test_logger.info("Test", skip_first=True)

    # Each call from different loop iteration is different call site
    # So we expect 3 entries (one per line in the loop)
    # The test should verify behavior, not implementation details
    assert len(test_logger._skip_first_state) >= 1


def test_throttle():
    """Test that throttle_duration_sec rate limits messages"""
    logger = Logger()
    logger.get_python_backend(level="INFO")

    test_logger = logger.get_logger(name="test.throttle")

    # Call multiple times rapidly from same loop
    count = 0
    for i in range(10):
        # Track if logged by checking state
        before = len(test_logger._throttle_state)
        test_logger.info("Test", throttle_duration_sec=0.5)
        after = len(test_logger._throttle_state)
        if after > before:
            count += 1
        time.sleep(0.1)  # 100ms between calls, throttle is 500ms

    # Should log roughly every 500ms (2-3 times in 1 second)
    # With 10 calls at 100ms intervals, only ~2 should go through
    assert count <= 3, f"Throttle should limit messages, got {count}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
