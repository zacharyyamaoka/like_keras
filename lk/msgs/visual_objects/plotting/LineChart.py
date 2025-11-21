#!/usr/bin/env python3

"""
LineChart - Lightweight container for line chart visualization.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from dataclasses import dataclass, field
import numpy as np


@dataclass
class LineChart(VisualObject):
    """
    Lightweight container for line chart data.

    Attributes:
        x: X-axis data (optional, if None will use indices)
        y: Y-axis data (can be single series or multiple series)
        labels: Optional labels for each series
        x_label: Label for x-axis
        y_label: Label for y-axis
        title: Chart title
    """

    y: np.ndarray | list[float] = field(default_factory=list)
    x: np.ndarray | list[float] | None = None
    labels: list[str] | None = None
    x_label: str = "X"
    y_label: str = "Y"
    title: str = "Line Chart"

    @classmethod
    def from_numpy(
        cls,
        y: np.ndarray,
        x: np.ndarray | None = None,
        name: str = "line_chart",
        **kwargs
    ):
        return cls(name=name, y=y, x=x, **kwargs)
