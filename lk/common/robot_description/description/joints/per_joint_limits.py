"""
Joint limit parameters for robot modeling.
"""

# BAM

# PYTHON
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class PerJointLimits:
    max_position: float = np.pi
    max_velocity: float = 10.0
    max_acceleration: float = 10.0
    max_jerk: float = 100.0
    max_effort: float = 0.0

    min_position: Optional[float] = None
    min_velocity: Optional[float] = None
    min_acceleration: Optional[float] = None
    min_jerk: Optional[float] = None
    min_effort: Optional[float] = None

    has_position_limits: bool = True
    has_velocity_limits: bool = True
    has_acceleration_limits: bool = True
    has_jerk_limits: bool = True
    has_effort_limits: bool = False

    def __post_init__(self) -> None:
        if self.min_position is None:
            self.min_position = -1.0 * self.max_position
        if self.min_velocity is None:
            self.min_velocity = -1.0 * self.max_velocity
        if self.min_acceleration is None:
            self.min_acceleration = -1.0 * self.max_acceleration
        if self.min_jerk is None:
            self.min_jerk = -1.0 * self.max_jerk
        if self.min_effort is None:
            self.min_effort = -1.0 * self.max_effort


if __name__ == "__main__":
    limits = PerJointLimits()
    print(limits)
