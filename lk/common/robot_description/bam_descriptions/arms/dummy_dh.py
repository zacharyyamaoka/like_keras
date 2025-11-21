#!/usr/bin/env python3


# BAM
from bam.descriptions import (
    BAM_MESH_PACKAGE_PATH,
    BAM_DESCRIPTIONS_PATH,
    UrdfInfo,
    RobotDescription,
    RobotInfo,
    ArmArgs,
    JointDescription,
    LinkDescription,
    Joints,
    Links,
    TAGS,
)

from .ur import DHParams

# PYTHON
from dataclasses import dataclass, field, asdict
import os

"""
Default Values similar to 

#ifdef UR30_PARAMS
const double d1 =  0.2363;
const double a2 = -0.6370;
const double a3 = -0.5037;
const double d4 =  0.2010;
const double d5 =  0.1593;
const double d6 =  0.1543;
#endif

Same defaults are set in __init__() and in the .urdf.xacro
Poses are taken at the default values, if you change the params they will no longer be valid!

Its good to have a simple offset_wrist robot, that doesn't depend on any external packages, and won't change

This is what this dummy_dh robot is

"""


@dataclass
class DummyDHJoints(Joints):
    joint_1: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    joint_2: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    joint_3: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    joint_4: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    joint_5: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )
    joint_6: JointDescription = field(
        default_factory=lambda: JointDescription(initial_position=0.0)
    )


@dataclass
class DummyDHLinks(Links):
    base_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(
            display=True, is_frame=True, tags=[TAGS.base_mount, TAGS.ik_base]
        )
    )
    link_0: LinkDescription = field(default_factory=LinkDescription)
    link_1: LinkDescription = field(default_factory=LinkDescription)
    link_2: LinkDescription = field(default_factory=LinkDescription)
    link_3: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    link_4: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    link_5: LinkDescription = field(
        default_factory=lambda: LinkDescription(scene_obj_collision=True)
    )
    ee_link: LinkDescription = field(
        default_factory=lambda: LinkDescription(is_frame=True)
    )
    tool0: LinkDescription = field(
        default_factory=lambda: LinkDescription(
            display=True, is_frame=True, tags=[TAGS.ik_tip, TAGS.arm_to_hand_mount]
        )
    )
    tool0_flip: LinkDescription = field(
        default_factory=lambda: LinkDescription(is_frame=True, tags=[TAGS.ik_tip_flip])
    )


@dataclass
class DummyDHUrdfInfo(UrdfInfo):
    xacro_path: str = BAM_DESCRIPTIONS_PATH + "/urdf/dummy_dh/dummy_dh.urdf.xacro"
    abs_package_dirs: dict[str, str] = field(
        default_factory=lambda: {
            "descriptions": BAM_MESH_PACKAGE_PATH,
        }
    )


@dataclass
class DummyDHInfo(RobotInfo):
    name: str = "dummy_dh"
    version: str = "0.0.0"
    save_dir: str = os.path.join(
        os.path.dirname(__file__),
        f"{os.path.splitext(os.path.basename(__file__))[0]}_configs",
    )


@dataclass
class DummyDHArgs(ArmArgs):
    dh: DHParams = field(default_factory=DHParams)

    reflect: int = 1


@dataclass
class DummyDH(RobotDescription):
    info: DummyDHInfo = field(default_factory=DummyDHInfo)
    # Xacro args are now directly in RobotDescription as fields
    # Cool actually, when you turn your self into a RobotDescription then these can be applied to args directly! Epic no need for config file...

    args: DummyDHArgs = field(default_factory=DummyDHArgs)

    urdf: DummyDHUrdfInfo = field(default_factory=lambda: DummyDHUrdfInfo())

    links: DummyDHLinks = field(default_factory=DummyDHLinks)
    joints: DummyDHJoints = field(default_factory=DummyDHJoints)

    def __post_init__(self):
        super().__post_init__()
        # Convert the xacro args to DH params

    @classmethod
    def from_dh_params(cls, dh_params: DHParams, reflect: int = 1):
        return cls(args=DummyDHArgs(dh=dh_params, reflect=reflect))


if __name__ == "__main__":
    rd = DummyDH.from_dh_params(
        DHParams(d1=0.2, a2=-0.5, a3=-0.5, d4=0.2, d5=0.15, d6=0.15), reflect=1
    )
    print(rd)
