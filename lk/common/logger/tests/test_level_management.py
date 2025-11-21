#!/usr/bin/env python3

"""
Test log level management
"""

# BAM
from bam_logger import Logger, get_logger, set_logger_level, configure

# PYTHON
import pytest
import logging


def test_set_logger_level_instance():
    """Test setting level on individual logger instance"""
    logger = Logger()
    logger.get_python_backend(name="test", level="INFO")

    child = logger.get_logger(name="test.child")

    # Set level on instance
    child.set_logger_level("DEBUG")

    # Verify backend level changed
    backends = child._get_all_backends()
    for backend in backends.values():
        if hasattr(backend, "logger"):
            assert backend.logger.level == logging.DEBUG


def test_set_logger_level_by_name():
    """Test setting level by logger name"""
    configure(py=True)

    logger = get_logger(name="test.named")

    # Set level by name
    set_logger_level(name="test.named", level="WARNING")

    # Verify level changed
    backends = logger._get_all_backends()
    for backend in backends.values():
        if hasattr(backend, "logger"):
            assert backend.logger.level == logging.WARNING


def test_set_all_loggers_level():
    """Test setting level for all loggers"""
    configure(py=True)

    logger1 = get_logger(name="test.all1")
    logger2 = get_logger(name="test.all2")

    # Set all to ERROR
    set_logger_level(level="ERROR")

    # Both should be ERROR
    for logger in [logger1, logger2]:
        backends = logger._get_all_backends()
        for backend in backends.values():
            if hasattr(backend, "logger"):
                assert backend.logger.level == logging.ERROR


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
