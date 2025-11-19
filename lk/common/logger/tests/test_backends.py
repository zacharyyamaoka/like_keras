#!/usr/bin/env python3

"""
    Test backend attachment and routing
"""

# BAM
from bam_logger import Logger

# PYTHON
import pytest


def test_get_python_backend():
    """Test getting Python backend"""
    logger = Logger(robot_id="TEST-01")
    backend = logger.get_python_backend(name="test", level="INFO")
    
    assert backend is not None
    assert backend.name == "python_test"
    assert "python_test" in logger.backends


def test_multi_backend_routing():
    """Test that messages route to all backends"""
    logger = Logger(robot_id="TEST-01")
    
    # Attach multiple backends
    backend1 = logger.get_python_backend(name="backend1", level="INFO")
    backend2 = logger.get_python_backend(name="backend2", level="DEBUG")
    
    assert len(logger.backends) == 2
    assert "python_backend1" in logger.backends
    assert "python_backend2" in logger.backends


def test_backend_inheritance():
    """Test that child loggers inherit parent's backends"""
    logger = Logger(robot_id="TEST-01")
    backend = logger.get_python_backend(name="test", level="INFO")
    
    child = logger.get_logger(name="child")
    
    # Child should see parent's backends
    all_backends = child._get_all_backends()
    assert "python_test" in all_backends


if __name__ == "__main__":
    pytest.main([__file__, "-v"])


