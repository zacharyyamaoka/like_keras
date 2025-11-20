# BAM

from bam.descriptions import (
    UrdfInfo, RobotInfo, JointDescription, LinkDescription,
    BAM_DESCRIPTIONS_PATH, BAM_MESH_PACKAGE_PATH, HandArgs, TAGS,
    RobotDescription, Joints, Links
)

# PYTHON

from dataclasses import dataclass, field
import numpy as np
from typing import Optional
import os

@dataclass
class CrabClawUrdfInfo(UrdfInfo):
    macro_path: str = BAM_DESCRIPTIONS_PATH + "/urdf/crab_claw_v1/crab_claw_v1_macro.xacro"
    xacro_path: str = BAM_DESCRIPTIONS_PATH + "/urdf/crab_claw_v1/crab_claw_v1_config.urdf.xacro"
    abs_package_dirs: dict[str, str] = field(default_factory=lambda: {
        "descriptions": BAM_MESH_PACKAGE_PATH,
    }) 

@dataclass
class CrabClawInfo(RobotInfo):
    name: str = "crab_claw"
    type: str = "CrabClaw"
    sku: str = ""
    version: str = "0.0.0"  
    save_dir: str = os.path.join(os.path.dirname(__file__), f"{os.path.splitext(os.path.basename(__file__))[0]}_configs")


@dataclass
class ClawJoints(Joints):
    
    eef_joint_1: JointDescription = field(default_factory=lambda: JointDescription(
        name="eef_joint_1",
        initial_position=0.0,
    ))
    
    joint_virtual_finger: JointDescription = field(default_factory=lambda: JointDescription(
        name="joint_virtual_finger",
        initial_position=0.0,
        # Follows eef_joint_1 via URDF <mimic> tag
    ))
    
@dataclass
class ClawLinks(Links):

    eef_link_1: LinkDescription = field(default_factory=lambda: LinkDescription(display=True, scene_obj_collision=True, tags=[TAGS.hand_to_arm_mount]))  # Hand base - enable collision
    eef_link_2: LinkDescription = field(default_factory=lambda: LinkDescription(scene_obj_collision=True))  # Moving finger - enable collision
    virtual_finger: LinkDescription = field(default_factory=lambda: LinkDescription(scene_obj_collision=True))  # Virtual finger - enable collision
    tcp_world: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True, display=True, tags=[TAGS.tcp_world]))  # Frame only
    tcp_tool: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True, tags=[TAGS.tcp_tool]))  # Frame only
    floating_tcp_world: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True, display=True, tags=[TAGS.tcp_world_floating]))  # Frame only
    crab_claw_center: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True))  # Frame only

@dataclass
class CrabClawArgs(HandArgs):
    widths: list[float] = field(default_factory=list)
    opening_angles: list[float] = field(default_factory=list)
    poly_degree: int = 3

    disable_floating_tcp: bool = False

    # z axis of center_link should be such that a positive opening angles, increases the width
    center_link: str = "crab_claw_center"
    max_width: float = 100/1000

@dataclass
class CrabClaw(RobotDescription):

    args: CrabClawArgs = field(default_factory=CrabClawArgs)

    urdf: CrabClawUrdfInfo = field(default_factory=CrabClawUrdfInfo)

    joints: ClawJoints = field(default_factory=ClawJoints)
    links: ClawLinks = field(default_factory=ClawLinks)


    @classmethod
    def make_bam(cls, 
        prefix: Optional[str] = None,
        disable_floating_tcp: bool = False,
        **kwargs
        ):

        dataset = [
            [0, 0],
            [5, 2.5],
            [10, 5.5],
            [20, 11.5],
            [30, 17.5],
            [60, 37.5],
            [120,76.5]
            ]

        args = CrabClawArgs(
            widths = [point[0]/1000 for point in dataset], # Convert from mm to m
            opening_angles = [float(np.deg2rad(point[1])) for point in dataset], # Convert from deg to rad
            poly_degree=3,
            disable_floating_tcp=disable_floating_tcp,
            max_width=100/1000, # Convert from mm to m
        )

        return cls(
            args=args,
            info=CrabClawInfo(prefix=prefix, name="crab_claw", sku="Bam"),
            **kwargs,
        )

