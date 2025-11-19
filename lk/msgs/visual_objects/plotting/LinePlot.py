#!/usr/bin/env python3

"""
    LinePlot - Generic multi-series line plotting.
    
    Supports flexible layouts:
    - Combined: All series on same plot(s)
    - Split series: Each series gets its own row
    - Split DOF: Each DOF gets its own column
    - Split both: Grid of plots (series x DOF)
"""

# BAM
from ..VisualObject import VisualObject
from .Series import Series
from bam.msgs import NumpyTrajectory

# PYTHON
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class VerticalLine:
    """
    Vertical line marker for plots.
    
    Attributes:
        x: X-axis position for the vertical line
        label: Optional label for the line
        color: Line color (default: black)
        line_style: Matplotlib line style (default: '--' for dashed)
        alpha: Line transparency (default: 0.5)
    """
    x: float
    label: Optional[str] = None
    color: str = "black"
    line_style: str = "--"
    alpha: float = 0.5


@dataclass
class HorizontalLine:
    """
    Horizontal line marker for plots.
    
    Attributes:
        y: Y-axis position for the horizontal line
        label: Optional label for the line
        color: Line color (default: black)
        line_style: Matplotlib line style (default: '--' for dashed)
        alpha: Line transparency (default: 0.5)
    """
    y: float
    label: Optional[str] = None
    color: str = "black"
    line_style: str = "--"
    alpha: float = 0.5


@dataclass
class LinePlot(VisualObject):
    """
    Generic multi-series line plot with flexible layout options.
    
    Attributes:
        series: List of Series objects to plot
        split_series: If True, each series gets its own row
        split_dof: If True, each DOF gets its own column
        dof_names: Optional names for DOF columns (e.g., ['Joint1', 'Joint2'])
        x_label: Label for x-axis
        y_label: Label for y-axis
        title: Overall plot title
        vertical_lines: Optional list of vertical lines to draw across all subplots
        
    Layout modes:
        - split_series=False, split_dof=False: All series on same plot(s), one plot per DOF
        - split_series=True, split_dof=False: Each series on separate row, one column per DOF
        - split_series=False, split_dof=True: All series together, one column per DOF
        - split_series=True, split_dof=True: Grid layout (n_series rows x n_dof columns)
    """
    series: list[Series] = field(default_factory=list)
    split_series: bool = False
    split_dof: bool = False
    dof_names: Optional[list[str]] = None
    x_label: str = "X"
    y_label: str = "Y"
    title: Optional[str] = None
    vertical_lines: Optional[list[VerticalLine]] = None
    horizontal_lines: Optional[list[HorizontalLine]] = None
    
    @property
    def n_series(self) -> int:
        """Number of series in this plot."""
        return len(self.series)
    
    @property
    def max_dof(self) -> int:
        """Maximum number of DOFs across all series."""
        return max(s.n_dof for s in self.series) if self.series else 0
    
    def validate(self) -> bool:
        """Validate that all series have compatible dimensions."""
        if not self.series:
            return False
        
        # All series should have same number of DOFs
        n_dof = self.series[0].n_dof
        for s in self.series:
            if s.n_dof != n_dof:
                raise ValueError(f"All series must have same number of DOFs. Found {s.n_dof} vs {n_dof}")
        
        # If dof_names provided, should match number of DOFs
        if self.dof_names is not None and len(self.dof_names) != n_dof:
            raise ValueError(f"dof_names length ({len(self.dof_names)}) must match n_dof ({n_dof})")
        
        return True
    
    @classmethod
    def from_np_trajectory(
        cls,
        traj: NumpyTrajectory,
        split_series: bool = True,
        split_dof: bool = True,
        name: str = "trajectory",
        title: Optional[str] = None,
    ) -> 'LinePlot':
        """
        Create LinePlot from NumpyTrajectory.
        
        Args:
            traj: NumpyTrajectory object
            split_series: If True, each variable (q, qd, qdd, tau) gets own row
            split_dof: If True, each joint/DOF gets own column
            name: Name for the visual object
            title: Optional title for the plot
            
        Returns:
            LinePlot visual object
        """
        from bam_artist.plot_np_traj import np_trajectory_to_line_plot
        return np_trajectory_to_line_plot(
            traj=traj,
            split_series=split_series,
            split_dof=split_dof,
            name=name,
            title=title
        )
    
    @classmethod
    def from_np_trajectory_comparison(
        cls,
        traj_target: NumpyTrajectory,
        traj_measured: NumpyTrajectory,
        split_dof: bool = True,
        name: str = "comparison",
        title: Optional[str] = None,
    ) -> 'LinePlot':
        """
        Create comparison plot of target vs measured trajectories.
        
        Args:
            traj_target: Target/planned trajectory
            traj_measured: Measured/actual trajectory
            split_dof: If True, each DOF gets own column
            name: Name for the visual object
            title: Optional title for the plot
            
        Returns:
            LinePlot visual object with target (solid) and measured (dashed) series
        """
        from bam_artist.plot_np_traj import np_trajectory_comparison_to_line_plot
        return np_trajectory_comparison_to_line_plot(
            traj_target=traj_target,
            traj_measured=traj_measured,
            split_dof=split_dof,
            name=name,
            title=title
        )

