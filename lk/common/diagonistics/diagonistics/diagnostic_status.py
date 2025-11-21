#!/usr/bin/env python3

"""
Lightweight Diagnostic Status Classes

Standalone replacements for ROS diagnostic_updater classes.
No ROS dependencies required.
"""

# PYTHON
from typing import List, Optional
from dataclasses import dataclass, field


@dataclass
class KeyValue:
    """Simple key-value pair for diagnostic data"""

    key: str = ""
    value: str = ""


class DiagnosticStatus:
    """Diagnostic status level constants"""

    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


@dataclass
class DiagnosticStatusWrapper:
    """
    Wrapper for diagnostic status information.
    Lightweight replacement for ROS DiagnosticStatusWrapper.
    """

    level: int = DiagnosticStatus.OK
    name: str = ""
    message: str = ""
    hardware_id: str = ""
    values: List[KeyValue] = field(default_factory=list)

    def summary(self, level_or_wrapper, message: str = ""):
        """Set or merge summary from level/message or another wrapper"""
        if isinstance(level_or_wrapper, DiagnosticStatusWrapper):
            # Merge from another wrapper
            self.mergeSummary(level_or_wrapper)
        else:
            # Set level and message
            self.level = level_or_wrapper
            self.message = message

    def mergeSummary(self, other: "DiagnosticStatusWrapper"):
        """Merge another status, taking the worst level"""
        if other.level > self.level:
            self.level = other.level
            self.message = other.message

    def add(self, key: str, value: str):
        """Add a key-value pair to diagnostic data"""
        self.values.append(KeyValue(key=key, value=value))

    def clearSummary(self):
        """Clear the summary"""
        self.level = DiagnosticStatus.OK
        self.message = ""


class DiagnosticTask:
    """Base class for diagnostic tasks"""

    def __init__(self, name: str):
        self.name = name

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        """Override this to implement diagnostic check"""
        return stat
