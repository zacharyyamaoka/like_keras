#!/usr/bin/env python3

"""
Series - Generic data series for plotting.

A series represents a single line or set of lines to be plotted.
The y values can be multi-dimensional (multiple DOFs).
"""

# PYTHON
from dataclasses import dataclass, field
import numpy as np
from typing import Optional


@dataclass
class Series:
    """
    Generic data series for line plotting.

    Attributes:
        y: Y-axis data, shape (n_samples,) for single DOF or (n_samples, n_dof) for multiple DOFs
        x: X-axis data, shape (n_samples,). If None, will use indices
        label: Label for this series (used in legends)
        line_style: Matplotlib line style ('-', '--', '-.', ':', etc.)
        color: Line color (can be name, hex, rgb, etc.). If None, uses default color cycle
        marker: Matplotlib marker style ('o', 's', '^', etc.). If None, no markers
        row_layer: Optional row layer for grouping series. Series with same row_layer appear on same row.
                   If None, each series gets its own row when split_series=True.
        point_labels: Optional labels for each point, shape (n_samples,). Used for annotations on plots.
    """

    y: np.ndarray | list[float]
    x: Optional[np.ndarray | list[float]] = None
    label: str = "Series"
    line_style: str = "-"
    color: Optional[str] = None
    marker: Optional[str] = None
    row_layer: Optional[int] = None
    point_labels: Optional[list[str]] = None

    def __post_init__(self):
        """Convert lists to numpy arrays for consistency."""
        if isinstance(self.y, list):
            self.y = np.array(self.y)
        if self.x is not None and isinstance(self.x, list):
            self.x = np.array(self.x)

    @property
    def n_samples(self) -> int:
        """Number of samples in the series."""
        return len(self.y)

    @property
    def n_dof(self) -> int:
        """Number of DOFs (1 for single series, n for multi-dof)."""
        return self.y.shape[1] if self.y.ndim > 1 else 1

    @property
    def is_multi_dof(self) -> bool:
        """True if this series contains multiple DOFs."""
        return self.n_dof > 1
