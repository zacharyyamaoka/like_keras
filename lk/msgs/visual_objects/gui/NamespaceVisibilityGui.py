#!/usr/bin/env python3

"""
NamespaceVisibilityGui - Control visibility of visual objects by namespace.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class NamespaceVisibilityGui(VisualObject):
    """
    GUI control for toggling visibility of visual objects by namespace.

    Creates a text input for filtering and dropdown for namespace selection.
    """

    name: str = "Namespace Visibility Control"
    initial_keys: list[str] = field(
        default_factory=lambda: [
            "stage",
            "00_",
            "01_",
            "02_",
            "03_",
            "04_",
            "05_",
            "06_",
            "07_",
            "08_",
            "09_",
        ]
    )
