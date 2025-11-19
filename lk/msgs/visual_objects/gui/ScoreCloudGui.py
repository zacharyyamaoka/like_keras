#!/usr/bin/env python3

"""
    ScoreCloudGui - Interactive controls for score cloud filtering.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from dataclasses import dataclass


@dataclass
class ScoreCloudGui(VisualObject):
    """
    GUI control for filtering ScoreCloud objects by score range.
    
    Creates:
    - Multi-slider to filter points by score range
    - Dropdown to select which ScoreCloud to control
    - Histogram display of score distribution with colormap colors
    """
    name: str = "Score Cloud Control"
    min_score: float = 0.0
    max_score: float = 1.0
    step: float = 0.01
    initial_range: tuple[float, float] = (0.0, 1.0)
    
    # Histogram settings
    show_histogram: bool = True
    histogram_bins: int = 20
    
    # Performance settings
    debounce_delay: float = 0.2  # Seconds to wait after slider stops moving before redrawing

