#!/usr/bin/env python3
"""
    Base BAM Five-Bar Arm description class.
    
    Contains common functionality and data structures for all BAM five-bar arm variants.
"""

# BAM
from bam.descriptions import BAM_DESCRIPTIONS_PATH, BAM_CORE_PATH, BAM_MESH_PACKAGE_PATH, ArmArgs
from bam.descriptions import (
    JointDescription, RobotInfo, LinkDescription, Inertia, PerJointLimits, UrdfInfo,
    Joints, Links, RobotDescription, TAGS
)
from bam.msgs.ros_msgs import TransformStamped, PoseStamped, Point, Pose
from ..ur import DHParams, IKSolPreference, URArgs


# PYTHON
import os
import numpy as np
from dataclasses import dataclass, field, asdict
import yaml
import copy

# Helpers to convert to meters in readable way
INCH = 25.4/1000
MM = 1/1000
REV2RAD = 2*np.pi
GRAMS = 1/1000

from enum import Enum

class Plugin(Enum):
    REAL = "real"
    MOCK = "mock"
    GAZEBO = "gazebo"
    NONE = "none"

def _make_base_limits():
    return PerJointLimits(
        max_position=np.pi,
        min_position=-np.pi,
        max_velocity=1.0,
        min_velocity=-1.0,
        max_acceleration=1.0,
        min_acceleration=-1.0,
    )

@dataclass
class BamFbJoints(Joints):
    
    shoulder_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("arm_base_link", "shoulder_link"),
        limits=_make_base_limits(),
    ))
    
    l1_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("shoulder_link", "l1_link"),
        limits=_make_base_limits(),
    ))
    
    l2_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("l1_link", "l2_link"),
        limits=_make_base_limits(),
    ))
    
    wrist_1_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("l2_link", "wrist_1_link"),
        limits=_make_base_limits(),
    ))
    
    wrist_2_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("wrist_1_link", "wrist_2_link"),
        limits=_make_base_limits(),
    ))
    
    wrist_3_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("wrist_2_link", "wrist_3_link"),
        limits=_make_base_limits(),
    ))
    
    c1_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("shoulder_link", "c1_link"),
        limits=_make_base_limits(),
        # Five-bar coupler - follows primary arm kinematics (mimic defined in URDF)
    ))

    c2_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("c1_link", "c2_link"),
        limits=_make_base_limits(),
        # Five-bar coupler - follows primary arm kinematics (mimic defined in URDF)
    ))

    c2_tip_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("c2_link", "c2_tip"),
        type="fixed",
    ))

    elbow_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("l2_link", "elbow_link"),
        type="fixed",
    ))

    elbow_tip_joint: JointDescription = field(default_factory=lambda: JointDescription(
        transform=TransformStamped.from_frames("elbow_link", "elbow_tip"),
        type="fixed",
    ))


@dataclass
class BamFbLinks(Links):

    arm_base_link: LinkDescription = field(default_factory=lambda: LinkDescription(display=True, tags=[TAGS.base_mount, TAGS.ik_base]))
    shoulder_link: LinkDescription = field(default_factory=LinkDescription)
    l1_link: LinkDescription = field(default_factory=LinkDescription)
    l2_link: LinkDescription = field(default_factory=LinkDescription)
    wrist_1_link: LinkDescription = field(default_factory=LinkDescription)
    wrist_2_link: LinkDescription = field(default_factory=LinkDescription)
    wrist_3_link: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True, display=True))
    c1_link: LinkDescription = field(default_factory=LinkDescription)
    c2_link: LinkDescription = field(default_factory=LinkDescription)
    elbow_link: LinkDescription = field(default_factory=LinkDescription)
    elbow_tip: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True))
    c2_tip: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True))
    tool0: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True, display=True, tags=[TAGS.ik_tip, TAGS.arm_to_hand_mount]))
    tool0_flip: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True, display=True, tags=[TAGS.ik_tip_flip]))
    ee_link: LinkDescription = field(default_factory=lambda: LinkDescription(is_frame=True))

@dataclass
class FiveBarParams():

    L_l1: float = 0.0
    L_l2: float = 0.0
    L_c1: float = 0.0
    L_c2: float = 0.0
    L_shoulder: float = 0.0
    L_elbow: float = 0.0
    theta_elbow_offset: float = 0.0


@dataclass
class BamFbCadKinParams:

    # Visual Params for Sense Check
    shoulder_servo_radius: float = None
    shoulder_servo_length: float = None
    wrist_servo_radius: float = None
    wrist_servo_length: float = None

    # Kinematic Params measured from CAD
    # Fix Link to Origin, z+ up, y+ forward
    # Place points at the interfaces, and read off the distances

    base_to_shoulder_z: float = None

    shoulder_to_l1_z: float = None
    shoulder_to_l1_y: float = None
    l1_to_c1_z: float = None

    l1_to_l2_z: float = None
    l1_to_l2_y: float = None

    l2_to_wrist_1_y: float = None
    l2_to_wrist_1_z: float = None
    l2_to_sticker_anchor_y: float = None
    l2_to_sticker_anchor_z: float = None
    l2_to_elbow_tip_z: float = None
    l2_to_elbow_tip_y: float = None
    l2_to_elbow_tip_theta: float = None

    wrist_1_to_wrist_2_z: float = None
    wrist_1_to_wrist_2_y: float = None
    wrist_2_to_wrist_3_z: float = None
    wrist_2_to_wrist_3_y: float = None

    c1_to_c2_z: float = None
    c1_to_c2_y: float = None
    c2_to_c2_tip_y: float = None
    c2_to_c2_tip_z: float = None
    
    def __post_init__(self):
        ...

    def mm_to_m(self):
 
        for attr, value in self.__dict__.items():
            if "theta" in attr:
                continue
            if isinstance(value, (int, float)):
                setattr(self, attr, value / 1000)

        return self

    def to_five_bar_params(self) -> FiveBarParams:
        return FiveBarParams(
            L_l1=self.l1_to_l2_y,
            L_l2=self.l2_to_wrist_1_y,
            L_c1=self.c1_to_c2_y,
            L_c2=self.c2_to_c2_tip_y,
            L_shoulder=self.shoulder_to_l1_y,
            L_elbow=self.l2_to_elbow_tip_y, 
            theta_elbow_offset=self.l2_to_elbow_tip_theta)

    # Look at the dh image and also the rviz model to help with this!
    def to_dh_params(self) -> DHParams:
        return DHParams(
            d1=self.base_to_shoulder_z + self.shoulder_to_l1_z,
            a2=-1*self.l1_to_l2_y,
            a3=-1*self.l2_to_wrist_1_y,
            # I need to get base_to_wrist_2_y, I could read the transform just just add...
            # Careful with the directions.... 
            d4=-1*(self.shoulder_to_l1_y + self.l1_to_l2_z - self.l2_to_wrist_1_z - self.wrist_1_to_wrist_2_z), 
            d5=self.wrist_1_to_wrist_2_y + self.wrist_2_to_wrist_3_z,
            d6=self.wrist_2_to_wrist_3_y)

    def to_xacro_args(self) -> dict[str, str]:
        return RobotDescription.to_xacro_args(self)


@dataclass
class BamFbUrdfParams:

    # SHOULDER CONFIG
    shoulder_front_plate_offset: float = 0.1
    shoulder_C1_offset: float = 0.08
    shoulder_J2_offset: float = 0.16

    shoulder_gearbox_mass: float = -1.0
    shoulder_gearbox_length: float = 81.5 * MM
    shoulder_gearbox_diameter: float = 64 * MM
    shoulder_gearbox_front_offset: float = 19 * MM
    shoulder_gearbox_flange_diameter: float = 64 * MM * 1.2

    shoulder_motor_mass: float = -1.0
    shoulder_motor_length: float = 60 * MM
    shoulder_motor_width: float = 60 * MM
    shoulder_driver_length: float = 20 * MM

    shoulder_plate_thickness: float = 0.003
    shoulder_mount_plate_thickness: float = 0.006

    # LINKAGE CONFIG
    five_bar_offset: float = 20 * MM
    link_density: float = 2700.0
    link_wall_thickness: float = (1.0/16) * INCH

    C1_length: float = 120 * MM
    C1_width: float = 45 * MM
    C1_thickness: float = 6 * MM
    C1_to_C2_spacing: float = 3 * MM
    C1_mass: float = -1.0

    C2_length: float = 260 * MM
    C2_width: float = 30 * MM
    C2_thickness: float = 6 * MM
    C2_mass: float = -1.0

    L1_length: float = 240 * MM
    L1_width: float = 2 * INCH
    L1_mass: float = -1.0
    L1_offset: float = 6 * MM + 3 * MM
    L1_to_L2_spacing: float = (3 + 6) * MM

    L2_length: float = 0.292
    L2_width: float = 1.5 * INCH
    L2_mass: float = -1.0

    theta_elbow_offset: float = np.deg2rad(45)
    elbow_length: float = 120 * MM

    # WRIST CONFIG
    wrist_length: float = 0.05
    wrist_pipe_diameter: float = 0.04
    wrist_pipe_wall_thickness: float = 0.0015
    wrist_pipe_density: float = 2700.0
    wrist_pipe_mass: float = -1.0

    wrist_servo_length: float = 0.03
    wrist_servo_diameter: float = 0.05
    wrist_servo_mass: float = 0.01
    wrist_driver_length: float = 0.01
    wrist_to_wrist_spacing: float = 0.003
    wrist_front_offset: float = 0.03 / 2


@dataclass
class BamFbArgs(URArgs):
    disable_L1: bool = False
    disable_L2: bool = False
    disable_C1: bool = False
    disable_C2: bool = False
    disable_wrist: bool = False
    disable_sticker: bool = False

    collision_mode: str = "simple"  # "simple" or "mesh"

    # IO
    transport_path: str = field(default_factory=lambda: os.getenv("TRANSPORT_PATH") or "")
    watchdog_timeout: float = 0.01

    params: BamFbUrdfParams = field(default_factory=BamFbUrdfParams)

    five_bar: FiveBarParams = field(default_factory=FiveBarParams)

    ik_sol_preference: list[dict] = field(default_factory=lambda: [
        {"Shoulder": "Left", "Elbow": "Down", "Wrist": "Down"},  # 1
        {"Shoulder": "Left", "Elbow": "Down", "Wrist": "Up"},    # 2
    ])

    parametric: bool = False

    cad_kin_params: BamFbCadKinParams = field(default_factory=BamFbCadKinParams)
    cad_kin: bool = False

    reflect: int = 1

    def __post_init__(self):
        super().__post_init__()
        self.five_bar = self.cad_kin_params.to_five_bar_params()
        self.dh = self.cad_kin_params.to_dh_params()


@dataclass
class BamFbUrdfInfo(UrdfInfo):
    macro_path: str = os.path.join(BAM_DESCRIPTIONS_PATH, "urdf/bam_fb/bam_fb_macro.xacro")
    xacro_path: str = os.path.join(BAM_DESCRIPTIONS_PATH, "urdf/bam_fb/bam_fb_config.urdf.xacro")
    abs_package_dirs: dict[str, str] = field(default_factory=lambda: {
        "descriptions": BAM_MESH_PACKAGE_PATH,
    })

@dataclass
class BamFbInfo(RobotInfo):
    name: str = "bam_fb"
    prefix: str = ""
    type: str = "bam_fb"
    sku: str = ""
    version: str = "0.0.0"
    save_dir: str = os.path.join(os.path.dirname(__file__), f"{os.path.splitext(os.path.basename(__file__))[0]}_configs")

@dataclass
class BamFb(RobotDescription):
    """BAM Five-Bar robotic arm description."""

    joints: BamFbJoints = field(default_factory=BamFbJoints)
    links: BamFbLinks = field(default_factory=BamFbLinks)

     
    urdf: BamFbUrdfInfo = field(default_factory=lambda: BamFbUrdfInfo())
    info: BamFbInfo = field(default_factory=lambda: BamFbInfo())
    
    args: BamFbArgs = field(default_factory=BamFbArgs)

    def __post_init__(self):
        super().__post_init__()

    def set_six_dof_mode(self) -> 'BamFb':
        self.args.disable_L1 = False
        self.args.disable_L2 = False
        self.args.disable_C1 = True
        self.args.disable_C2 = True
        # self.args.disable_wrist = True
        # self.args.disable_sticker = True
        return self

    def set_shoulder_mode(self) -> 'BamFb':
        self.args.disable_L1 = True
        self.args.disable_L2 = True
        self.args.disable_C1 = True
        self.args.disable_C2 = True
        self.args.disable_wrist = True
        self.args.disable_sticker = True
        return self

    def disable_wrist(self) -> 'BamFb':
        self.args.disable_wrist = True
        return self

    def disable_sticker(self) -> 'BamFb':
        self.args.disable_sticker = True
        return self

    def set_parametric_urdf(self) -> 'BamFb':
        self.urdf.macro_path = BAM_DESCRIPTIONS_PATH + "/urdf/dummy_bam_fb/dummy_bam_fb_macro.xacro"
        self.urdf.xacro_path = BAM_DESCRIPTIONS_PATH + "/urdf/dummy_bam_fb/dummy_bam_fb_config.urdf.xacro"
        return self

    def set_cad_kin_urdf(self) -> 'BamFb':
        self.urdf.macro_path = BAM_DESCRIPTIONS_PATH + "/urdf/bam_fb/bam_fb_cad_kin_macro.xacro"
        self.urdf.xacro_path = BAM_DESCRIPTIONS_PATH + "/urdf/bam_fb/bam_fb_cad_kin_config.urdf.xacro"

        return self

    def reflect_joints(self, q: np.ndarray) -> np.ndarray:
        # account for reflection.
        
        offsets = {
            "l1_joint": np.pi/2,
            "c2_joint": np.pi/2,
        }
        if self.args.reflect == -1:
            # Update 
            name_to_joint_index = self.joint_positions.get_name_to_joint_index()
            for name, offset in offsets.items():
                q[name_to_joint_index[name]] += offset


        return q

    @classmethod
    def make_dev(cls, reflect: int = 1, **kwargs) -> 'BamFb':
        from .bam_fb_dev import BamFbDev, BamFbDevArgs
        return BamFbDev(args=BamFbDevArgs(reflect=reflect), **kwargs).init_joint_positions()

