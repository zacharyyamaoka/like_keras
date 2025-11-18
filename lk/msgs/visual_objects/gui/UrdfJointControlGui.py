#!/usr/bin/env python3

"""
    UrdfJointControlGui - Interactive controls for URDF joint manipulation.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from typing import Optional
from dataclasses import dataclass, field
import numpy as np


@dataclass
class UrdfJointControlGui(VisualObject):
    """
    GUI control for URDF joint manipulation.
    
    Provides sliders for each actuated joint with bidirectional control:
    - Manual control via sliders
    - External code can update joint positions
    - Lock checkbox to prevent external updates during manual control
    """
    name: str = "URDF Joint Control"
    target_urdf_name: Optional[str] = None  # visual_id of URDF to control (None = first found)
    initial_q: Optional[np.ndarray] = None  # Initial joint config (None = use URDF's current)
    initial_lock: bool = False  # Start with external updates blocked
    show_reset_button: bool = True  # Show button to reset to initial configuration


