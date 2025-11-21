#!/usr/bin/env python3

"""
Robotiq 2F-140 hand description.

Defines the Robotiq 2F-140 specific joint, link, and metadata configuration
while reusing the shared Robotiq base helpers.
"""

# BAM
from bam.descriptions import (
    JointDescription,
    LinkDescription,
    ROBOTIQ_DESCRIPTION_PATH,
    TAGS,
)
from .robotiq import (
    RobotiqArgs,
    Robotiq,
    RobotiqInfo,
    RobotiqUrdfInfo,
)
from bam.descriptions import TAGS

from bam.descriptions import Joints, Links

# PYTHON
from dataclasses import dataclass, field


@dataclass
class Robotiq2F140Joints(Joints):
    finger_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            name="finger_joint", initial_position=0.0
        )
    )
    right_outer_knuckle_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            initial_position=0.0,
            mimic=True,
        )
    )
    left_inner_knuckle_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            initial_position=0.0,
            mimic=True,
        )
    )
    right_inner_knuckle_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            initial_position=0.0,
            mimic=True,
        )
    )
    left_inner_finger_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            initial_position=0.0,
            mimic=True,
        )
    )
    right_inner_finger_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            initial_position=0.0,
            mimic=True,
        )
    )


@dataclass
class Robotiq2F140Links(Links):
    robotiq_base_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(
            display=True, scene_obj_collision=True, tags=[TAGS.hand_to_arm_mount]
        )
    )
    left_outer_knuckle: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    right_outer_knuckle: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    left_outer_finger: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    right_outer_finger: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    left_inner_knuckle: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    right_inner_knuckle: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    left_inner_finger: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    right_inner_finger: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    left_inner_finger_pad: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    right_inner_finger_pad: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    # tcp_tool: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True))
    tcp_world: LinkDescription = field(
        default_factory=lambda: LinkDescription(
            is_frame=True, display=True, tags=[TAGS.tcp_world]
        )
    )


@dataclass
class Robotiq2F140Info(RobotiqInfo):
    name: str = "robotiq_2f_140"
    sku: str = "2f_140"


@dataclass
class Robotiq2F140Args(RobotiqArgs):
    max_width: float = 0.140
    gripper_max_speed: float = 0.250
    gripper_max_force: float = 125.0
    gripper_closed_position: float = 0.695


@dataclass
class Robotiq2F140UrdfInfo(RobotiqUrdfInfo):
    macro_path: str = f"{ROBOTIQ_DESCRIPTION_PATH}/urdf/robotiq_2f_140_macro.urdf.xacro"
    xacro_path: str = (
        f"{ROBOTIQ_DESCRIPTION_PATH}/urdf/robotiq_2f_140_config.urdf.xacro"
    )


@dataclass
class Robotiq2F140(Robotiq):
    """Robotiq 2F-140 parallel jaw gripper."""

    info: Robotiq2F140Info = field(default_factory=Robotiq2F140Info)
    joints: Robotiq2F140Joints = field(default_factory=Robotiq2F140Joints)
    links: Robotiq2F140Links = field(default_factory=Robotiq2F140Links)

    args: Robotiq2F140Args = field(default_factory=Robotiq2F140Args)
    urdf: Robotiq2F140UrdfInfo = field(default_factory=Robotiq2F140UrdfInfo)
