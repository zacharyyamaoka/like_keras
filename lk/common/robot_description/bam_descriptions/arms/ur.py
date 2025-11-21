#!/usr/bin/env python3

# BAM
from bam.descriptions import BAM_REACH_PATH, UR_DESCRIPTION_PATH, UR_MESH_PACKAGE_PATH

from bam.descriptions import (
    JointDescription,
    UrdfInfo,
    SrdfInfo,
    RobotInfo,
    LinkDescription,
    RobotDescription,
    Joints,
    Links,
    JointPositions,
    ArmArgs,
    TAGS,
)

from .. import BamUrdfInfo

# PYTHON
import os
from typing import List, Literal
from dataclasses import dataclass, field


@dataclass
class DHParams:
    """DH parameters for closed form offset wrist kinematics like UR. Example:

    UR5e_PARAMS = {
        "d1": 0.1625,
        "a2": -0.425,
        "a3": -0.3922,
        "d4": 0.1333,
        "d5": 0.0997,
        "d6": 0.0996,
    }

    Notes:
    - d4 may be negative depending on if left or right arm.
    - If you have CAD model you can verify that L1 = a2, and L2 = a3
    """

    # Could be zero for clarity but set with reasonable defaults
    d1: float = 0.2
    a2: float = -0.5
    a3: float = -0.5
    d4: float = 0.2
    d5: float = 0.15
    d6: float = 0.15

    def to_list(self):
        return [self.d1, self.a2, self.a3, self.d4, self.d5, self.d6]


@dataclass
class IKSolPreference:
    """8 possible combinations lead to 8 solutions  2*2*2 = 8"""

    shoulder: Literal["left", "right"] = "left"
    elbow: Literal["down", "up"] = "down"
    wrist: Literal["down", "up"] = "down"


@dataclass
class URJoints(Joints):
    shoulder_pan_joint: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    shoulder_lift_joint: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=-1.5707)
    )
    elbow_joint: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    wrist_1_joint: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    wrist_2_joint: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    wrist_3_joint: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )


@dataclass
class URLinks(Links):
    base_link_ur: LinkDescription = field(
        default_factory=lambda: LinkDescription(
            display=True, is_frame=True, tags=[TAGS.base_mount, TAGS.ik_base]
        )
    )  # Frame only - Display by default
    shoulder_link: LinkDescription = field(default_factory=LinkDescription)
    upper_arm_link: LinkDescription = field(default_factory=LinkDescription)
    forearm_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )  # Enable collision
    wrist_1_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )  # Enable collision
    wrist_2_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )  # Enable collision
    wrist_3_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )  # Enable collision
    ee_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(is_frame=True)
    )  # Frame only
    tool0: LinkDescription = field(
        default_factory=lambda: LinkDescription(
            display=True, is_frame=True, tags=[TAGS.ik_tip, TAGS.arm_to_hand_mount]
        )
    )  # Frame only - Display by default (TCP)
    tool0_flip: LinkDescription = field(
        default_factory=lambda: LinkDescription(is_frame=True, tags=[TAGS.ik_tip_flip])
    )  # Frame only


@dataclass
class URUrdfInfo(BamUrdfInfo):
    macro_path: str = UR_DESCRIPTION_PATH + "/urdf/ur_macro.xacro"
    xacro_path: str = UR_DESCRIPTION_PATH + "/urdf/ur_config.urdf.xacro"
    abs_package_dirs: dict[str, str] = field(
        default_factory=lambda: {
            "ur_description": UR_MESH_PACKAGE_PATH,
        }
    )


@dataclass
class URSrdfInfo(SrdfInfo):
    xacro_path: str = (
        "/home/bam/public_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/srdf/ur.srdf.xacro"  # Optinally can set
    )


@dataclass
class URInfo(RobotInfo):
    name: str = "ur"
    type: str = "ur"
    sku: str = "urx"
    version: str = "0.0.0"
    save_dir: str = os.path.join(
        os.path.dirname(__file__),
        f"{os.path.splitext(os.path.basename(__file__))[0]}_configs",
    )


@dataclass
class URArgs(ArmArgs):

    dh: DHParams = field(default_factory=DHParams)

    ik_sol_preference: list[IKSolPreference] = field(
        default_factory=lambda: [
            IKSolPreference(shoulder="left", elbow="up", wrist="down"),  # 1
            IKSolPreference(shoulder="left", elbow="up", wrist="up"),  # 2
        ]
    )

    trajectory_topic: str = "scaled_joint_trajectory_controller/follow_joint_trajectory"
    reach_map_path: str = BAM_REACH_PATH + "/..."

    def __post_init__(self):
        super().__post_init__()


@dataclass
class URJointPositions(JointPositions):
    """UR-specific named positions with predefined joint order and positions."""

    joint_names: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
    )
    positions: dict[str, list[float]] = field(
        default_factory=lambda: {
            "initial": [0, -1.5707, 0, -1.5707, 0, 0],
            "up": [0, -1.5707, 0, -1.5707, 0, 0],
            "ready": [1.4, -1.62, 1.54, -1.2, -1.6, -0.11],
        }
    )
    default_positions: str = "initial"


@dataclass
class UR(RobotDescription):

    # Override the joints field with default_factory
    info: URInfo = field(default_factory=URInfo)

    joints: URJoints = field(default_factory=URJoints)
    links: URLinks = field(default_factory=URLinks)

    urdf: URUrdfInfo = field(default_factory=URUrdfInfo)
    srdf: URSrdfInfo = field(default_factory=URSrdfInfo)

    args: URArgs = field(default_factory=URArgs)

    joint_positions: URJointPositions = field(default_factory=URJointPositions)

    def __post_init__(self):
        super().__post_init__()

        # self.name = self.info.name # Can make it auto assign if you want...

    @classmethod
    def make_UR5e(
        cls,
        prefix=None,
        plugin="none",
        world_xyz=(0, 0, 0),
        world_rpy=(0, 0, 0),
        base_xyz=(0, 0, 0),
        base_rpy=(0, 0, 0),
        **kwargs,
    ):

        args = URArgs(
            dh=DHParams(
                d1=0.1625,
                a2=-0.425,
                a3=-0.3922,
                d4=0.1333,
                d5=0.0997,
                d6=0.0996,
            ),
            plugin=plugin,
            world_xyz=world_xyz,
            world_rpy=world_rpy,
            base_xyz=base_xyz,
            base_rpy=base_rpy,
            reach_map_path=BAM_REACH_PATH
            + "/maps/ur/ur5e/ur5e_table_0.8x1.5x0.8_0.30_180x45x360_09_aug_2025.pkl",
            **kwargs,
        )

        info = URInfo(
            prefix=prefix,
            sku="ur5e",
        )

        return cls(info=info, args=args).init_joint_positions()

    @classmethod
    def make_UR10(
        cls,
        prefix=None,
        plugin="none",
        world_xyz=(0, 0, 0),
        world_rpy=(0, 0, 0),
        base_xyz=(0, 0, 0),
        base_rpy=(0, 0, 0),
        **kwargs,
    ):
        info = URInfo(
            prefix=prefix,
            sku="ur10",
        )

        args = URArgs(
            dh=DHParams(
                d1=0.1273,
                a2=-0.612,
                a3=-0.5723,
                d4=0.163941,
                d5=0.1157,
                d6=0.0922,
            ),
            plugin=plugin,
            world_xyz=world_xyz,
            world_rpy=world_rpy,
            base_xyz=base_xyz,
            base_rpy=base_rpy,
            reach_map_path=BAM_REACH_PATH
            + "/maps/ur/ur10/ur10_table_0.8x1.5x0.8_0.30_180x45x360_09_aug_2025.pkl",
            **kwargs,
        )

        return cls(info=info, args=args).init_joint_positions()
