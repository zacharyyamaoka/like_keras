#!/usr/bin/env python3

"""
Test logger registry behavior
"""

# BAM
from bam_logger import get_logger, get_logger_registry, configure

# PYTHON
import pytest


def test_same_name_returns_same_instance():
    """Test that get_logger with same name returns same instance (like logging.getLogger)"""
    # Reset for clean test
    configure(py=True, test_id="registry_test")

    logger1 = get_logger(name="test.logger1")
    logger2 = get_logger(name="test.logger1")

    assert logger1 is logger2, "Same name should return same instance"


def test_hierarchical_naming():
    """Test hierarchical logger naming with dot notation"""
    configure(py=True)

    root = get_logger(name="root")
    child = root.get_logger(name="child")
    grandchild = child.get_logger(name="grandchild")

    # Check context has hierarchical names
    assert child.global_context["name"] == "root.child"
    assert grandchild.global_context["name"] == "root.child.grandchild"


def test_registry_tracking():
    """Test that logger registry tracks all created loggers"""
    configure(py=True)

    # Clear by creating new config
    configure(py=True, test_id="registry_tracking")

    get_logger(name="logger1")
    get_logger(name="logger2")
    get_logger(name="logger3")

    registry = get_logger_registry()

    assert "logger1" in registry
    assert "logger2" in registry
    assert "logger3" in registry
    assert len(registry) >= 3


def test_child_inheritance():
    """Test that child loggers inherit parent's backends and context"""
    configure(py=True, robot_id="TEST-INHERIT")

    root = get_logger(name="inherit_parent", component="root_comp")
    child = root.get_logger(name="child")

    # Child should have parent's context
    merged = child._get_merged_context()
    assert merged["robot_id"] == "TEST-INHERIT"
    assert merged["component"] == "root_comp"
    assert merged["name"] == "inherit_parent.child"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
