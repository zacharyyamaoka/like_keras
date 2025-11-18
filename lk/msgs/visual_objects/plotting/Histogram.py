#!/usr/bin/env python3

"""
    Histogram - Lightweight container for histogram visualization.
"""

# BAM
from ..VisualObject import VisualObject
from bam.utils import BinConfig

# PYTHON
from dataclasses import dataclass, field
import numpy as np
from typing import Optional

@dataclass
class Histogram(VisualObject):
    """
    Lightweight container for histogram data.
    
    Attributes:
        values: Raw values to bin/histogram
        bin_config: Optional BinConfig for custom binning
        bin_edges: Optional explicit bin edges (overrides bin_config)
        x_label: Label for x-axis
        y_label: Label for y-axis (typically "Count")
        title: Histogram title
        color: Bar color
    """
    values: np.ndarray | list[float] = field(default_factory=list)
    bin_config: Optional[BinConfig] = None
    bin_edges: Optional[list[float]] = None
    x_label: str = "Values"
    y_label: str = "Count"
    title: str = "Histogram"
    color: str = "steelblue"
    
    @classmethod
    def from_numpy(cls, values: np.ndarray, name: str = "histogram", **kwargs):
        return cls(
            name=name,
            values=values,
            **kwargs
        )
    
    @classmethod
    def from_bin_config(cls, values: np.ndarray, bin_config: BinConfig, name: str = "histogram", **kwargs):
        return cls(
            name=name,
            values=values,
            bin_config=bin_config,
            **kwargs
        )

