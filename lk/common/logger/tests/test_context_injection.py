#!/usr/bin/env python3

"""
Test context injection and merging
"""

# BAM
from bam_logger import Logger, configure, get_logger

# PYTHON
import pytest


def test_global_context_injection():
    """Test that global context is automatically injected"""
    logger = Logger(robot_id="TEST-01", run_id="exp_001")
    logger.get_python_backend(level="INFO")

    # Global context should be in merged context
    child = logger.get_logger(name="test")
    merged = child._get_merged_context()

    assert merged["robot_id"] == "TEST-01"
    assert merged["run_id"] == "exp_001"


def test_context_merging():
    """Test that global and per-call context merge correctly"""
    logger = Logger(robot_id="TEST-01")
    logger.get_python_backend(level="INFO")

    child = logger.get_logger(name="test_merge", component="arm")
    merged = child._get_merged_context()

    # Should have both parent and child context
    assert merged["robot_id"] == "TEST-01"
    assert merged["name"] == "test_merge"
    assert merged["component"] == "arm"


def test_context_update():
    """Test updating context on a logger"""
    logger = Logger(robot_id="TEST-01")
    logger.get_python_backend(level="INFO")

    child = logger.get_logger(name="test")

    # Update context
    child.update_context(phase="execution", trial=1)

    merged = child._get_merged_context()
    assert merged["phase"] == "execution"
    assert merged["trial"] == 1


def test_context_inheritance():
    """Test that nested children inherit all parent context"""
    configure(py=True, robot_id="TEST-01", run_id="exp_001")

    parent = get_logger(name="parent", component="arm")
    child = parent.get_logger(name="child", subsystem="joints")

    merged = child._get_merged_context()

    assert merged["robot_id"] == "TEST-01"
    assert merged["run_id"] == "exp_001"
    assert merged["component"] == "arm"
    assert merged["subsystem"] == "joints"
    assert merged["name"] == "parent.child"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
