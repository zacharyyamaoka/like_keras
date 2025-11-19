#!/usr/bin/env python3

"""
    PoseSelectorGui - Interactive controls for pose selection.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from typing import Optional
from dataclasses import dataclass


@dataclass
class PoseSelectorGui(VisualObject):
    """
    GUI control for pose selection from discretized workspace.
    
    Creates 5 sliders for selecting poses:
    - x, y, z: Spatial position indices
    - view_idx: View direction index
    - rot_idx: Rotation index
    
    Can optionally integrate with ReachMap for automatic URDF updates.
    """
    name: str = "Pose Selector"
    
    # Grid dimensions
    n_x: int = 10
    n_y: int = 10
    n_z: int = 10
    n_view_idx: int = 8
    n_rot_idx: int = 8
    
    # Initial slider values
    initial_x: int = 0
    initial_y: int = 0
    initial_z: int = 0
    initial_view_idx: int = 0
    initial_rot_idx: int = 0
    
    # Optional ReachMap integration (for future enhancement)
    reach_map: Optional[any] = None
    target_urdf_name: Optional[str] = None
    show_target_frame: bool = True
    frame_name: str = "pose_selector_target"

