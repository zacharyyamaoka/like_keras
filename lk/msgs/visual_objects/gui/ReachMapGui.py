#!/usr/bin/env python3

"""
ReachMapGui - Interactive controls for ReachMap visualization and exploration.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    from bam_reach.reach_map import ReachMap


@dataclass
class ReachMapGui(VisualObject):
    """
    GUI control for ReachMap exploration.

    Creates two sliders:
    - Position index: Select which position in the workspace
    - Orientation index: Select which orientation at that position

    Displays:
    - Status text: SUCCESS, FAILURE, or COLLISION
    - Bar chart: Visual representation of IK/FK status for all orientations
      Each bar has 4 equal parts showing:
      1. IK solution exists (green=yes, red=no)
      2. IK success (green=yes, red=no)
      3. FK success (green=yes, red=no)
      4. Consistency (green=yes, red=no)

    Optionally integrates with ReachMap to:
    - Update URDF to show robot at selected pose
    - Display target frame at FK solution
    - Handle invalid poses gracefully (return to neutral)
    """

    name: str = "ReachMap Explorer"

    # ReachMap integration (required for interactive features)
    reach_map: Optional["ReachMap"] = None
    target_urdf_name: str = ""  # Visual ID of URDF to control
    neutral_q: Optional[np.ndarray] = None  # Fallback configuration when pose invalid

    # Visualization options
    show_target_frame: bool = True
    frame_name: str = "reachmap_target"
    show_status_text: bool = True
    show_bar_chart: bool = True

    # Collision mesh toggle
    show_collision_toggle: bool = True
    initial_show_collision: bool = False

    # Navigation helpers
    show_jump_inconsistent: bool = True
    show_jump_valid: bool = True

    # Initial slider values
    initial_pos_idx: int = 0
    initial_orient_idx: int = 0
