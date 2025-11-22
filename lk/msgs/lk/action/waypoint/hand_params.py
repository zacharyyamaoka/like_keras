"""
Per-waypoint hand motion parameters.

Contains hand control parameters for a single waypoint, including
optional per-waypoint kinematic limits that override global context limits.
"""

# BAM
import copy
import random

# PYTHON
from dataclasses import dataclass, field

from lk.msgs.msg_utils import lerp_value

from ..joint_limits import JointLimits


@dataclass
class HandParams:
    width: float = 0.0
    joint_angles: list[float] = field(
        default_factory=list
    )  # helpful to carry this information along

    # from https://control.ros.org/master/doc/ros2_controllers/parallel_gripper_controller/doc/userdoc.html
    # can be set by node... but also may want to override...
    goal_tolerance: float = 0.0
    allow_stalling: bool = False
    stall_velocity_threshold: float = 0.0
    stall_timeout: float = 0.0
    max_effort: float = 0.0
    max_vel: float = 0.0

    # Optional per-waypoint joint limits (overrides WaypointContext hand_limits for this waypoint)
    # Order of joints should correspond to hand_ik_sol joint order
    joint_limits: JointLimits = field(default_factory=JointLimits)

    def lerp(self, target: "HandParams", fraction: float) -> "HandParams":
        new_params = copy.deepcopy(self)
        new_params.width = lerp_value(self.width, target.width, fraction)
        new_params.max_effort = lerp_value(self.max_effort, target.max_effort, fraction)
        new_params.max_vel = lerp_value(self.max_vel, target.max_vel, fraction)

        return new_params

    def difference(self, target: "HandParams") -> dict[str, float]:
        width_diff = abs(self.width - target.width)
        max_effort_diff = abs(self.max_effort - target.max_effort)
        max_vel_diff = abs(self.max_vel - target.max_vel)
        return {
            "width": width_diff,
            "max_effort": max_effort_diff,
            "max_vel": max_vel_diff,
        }

    @classmethod
    def make_random(cls) -> "HandParams":
        hand_params = cls()
        hand_params.width = random.uniform(0.0, 0.2)
        hand_params.max_effort = random.uniform(0.0, 1000.0)
        hand_params.max_vel = random.uniform(0.0, 1.0)
        return hand_params
