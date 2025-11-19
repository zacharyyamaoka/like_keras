#!/usr/bin/env python3

"""
    PointCloudGui - Interactive controls for point cloud visualization.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from dataclasses import dataclass, field
from typing import Literal


@dataclass
class PointCloudGui(VisualObject):
    """
    GUI control for point cloud visualization settings.
    
    Creates a slider to control point size and dropdown for point shape
    of all point clouds in the scene.
    """
    name: str = "Point Cloud Controls"
    size_min: float = 0.0001
    size_initial: float = 0.005
    size_max: float = 0.01
    size_step: float = 0.0001
    
    shape_options: list[str] = field(default_factory=lambda: [
        "square", "diamond", "circle", "rounded", "sparkle"
    ])
    initial_shape: str = "square"

