#!/usr/bin/env python3

"""
    Robotiq 2F-85 hand description.

    Defines the Robotiq 2F-85 specific joint, link, and metadata configuration
    while reusing the shared Robotiq base helpers.
"""

# BAM
from bam.descriptions import (
    JointDescription, LinkDescription, JointMimic,
    ROBOTIQ_DESCRIPTION_PATH, TAGS
)
from .robotiq import (
    RobotiqArgs,
    Robotiq,
    RobotiqInfo,
    RobotiqUrdfInfo,
)
from bam.descriptions import TAGS

from bam.descriptions import Joints, Links, JointPositions

# PYTHON
from dataclasses import dataclass, field


@dataclass
class Robotiq2F85Joints(Joints):
    # Renamed to appear before mimic joints alphabetically
    finger_joint: JointDescription = field(
        default_factory=lambda: JointDescription(name="finger_joint", initial_position=0.0)
    )
    robotiq_85_right_knuckle_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            name="robotiq_85_right_knuckle_joint",
            initial_position=0.0,
            mimic=JointMimic(enabled=True),
        )
    )
    robotiq_85_left_inner_knuckle_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            name="robotiq_85_left_inner_knuckle_joint",
            initial_position=0.0,
            mimic=JointMimic(enabled=True),
        )
    )
    robotiq_85_right_inner_knuckle_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            name="robotiq_85_right_inner_knuckle_joint",
            initial_position=0.0,
            mimic=JointMimic(enabled=True),
        )
    )
    robotiq_85_left_finger_tip_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            name="robotiq_85_left_finger_tip_joint",
            initial_position=0.0,
            mimic=JointMimic(enabled=True),
        )
    )
    robotiq_85_right_finger_tip_joint: JointDescription = field(
        default_factory=lambda: JointDescription(
            name="robotiq_85_right_finger_tip_joint",
            initial_position=0.0,
            mimic=JointMimic(enabled=True),
        )
    )


@dataclass
class Robotiq2F85Links(Links):
    robotiq_85_base_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(display=True, scene_obj_collision=True, tags=[TAGS.hand_to_arm_mount])
    )
    robotiq_85_left_knuckle_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    robotiq_85_right_knuckle_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    robotiq_85_left_finger_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    robotiq_85_right_finger_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    robotiq_85_left_inner_knuckle_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    robotiq_85_right_inner_knuckle_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    robotiq_85_left_finger_tip_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    robotiq_85_right_finger_tip_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    # tcp_tool: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True))
    tcp_world: LinkDescription = field(
        default_factory=lambda: LinkDescription(is_frame=True, display=True, tags=[TAGS.tcp_world])
    )

@dataclass
class Robotiq2F85JointPositions(JointPositions):
    """Robotiq 2F-85 specific named positions with predefined joint order and positions."""
    joint_names: list[str] = field(default_factory=lambda: ["finger_joint"])
    positions: dict[str, list[float]] = field(default_factory=lambda: {
        "initial": [0.0],
        "open": [0.0],
        "closed": [0.7929],
    })
    default_positions: str = "initial"

@dataclass
class Robotiq2F85Info(RobotiqInfo):
    name: str = "robotiq_2f_85"
    sku: str = "2f_85"

@dataclass
class Robotiq2F85Args(RobotiqArgs):
    max_width: float = 0.085
    gripper_max_speed: float = 0.150
    gripper_max_force: float = 235.0
    gripper_closed_position: float = 0.7929

@dataclass
class Robotiq2F85UrdfInfo(RobotiqUrdfInfo):
    macro_path: str = f"{ROBOTIQ_DESCRIPTION_PATH}/urdf/robotiq_2f_85_macro.urdf.xacro"
    xacro_path: str = f"{ROBOTIQ_DESCRIPTION_PATH}/urdf/robotiq_2f_85_config.urdf.xacro"

@dataclass
class Robotiq2F85(Robotiq):
    """Robotiq 2F-85 parallel jaw gripper."""


    info: Robotiq2F85Info = field(default_factory= Robotiq2F85Info)
    joints: Robotiq2F85Joints = field(default_factory=Robotiq2F85Joints)
    links: Robotiq2F85Links = field(default_factory=Robotiq2F85Links)
    joint_positions: Robotiq2F85JointPositions = field(default_factory=Robotiq2F85JointPositions)

    args: Robotiq2F85Args = field(default_factory=Robotiq2F85Args)
    urdf: Robotiq2F85UrdfInfo = field(default_factory=Robotiq2F85UrdfInfo)
