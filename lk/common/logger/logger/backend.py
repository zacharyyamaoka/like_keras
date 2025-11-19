#!/usr/bin/env python3

"""
    Backend Protocol - Base interface for logging backends.
    
    Defines the contract that all logging backends must implement.
    Each backend receives messages with context and handles formatting/output.
    
    Messages can be any type - strings, foxglove schemas, BAM/ROS messages, etc.
    Each backend is responsible for type conversion and formatting.
"""

# PYTHON
from typing import Protocol, Any, Optional


class Backend(Protocol):
    """Protocol defining the interface for logging backends.
    
    Backends receive messages of any type along with optional topic and log_time.
    Each backend handles type conversion and formatting appropriately.
    """
    
    @property
    def name(self) -> str:
        """Unique name identifying this backend."""
        ...
    
    def debug(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, **context) -> None:
        """Log debug message with context."""
        ...
    
    def info(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, **context) -> None:
        """Log info message with context."""
        ...
    
    def warning(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, **context) -> None:
        """Log warning message with context."""
        ...
    
    def error(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, **context) -> None:
        """Log error message with context."""
        ...
    
    def critical(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, **context) -> None:
        """Log critical message with context."""
        ...

