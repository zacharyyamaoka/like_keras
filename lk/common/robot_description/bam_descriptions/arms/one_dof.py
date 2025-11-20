#!/usr/bin/env python3

# BAM
from bam.descriptions.arms.arm_description import RobotDescription
from bam.descriptions import BAM_DESCRIPTIONS_PATH, BAM_MESH_PACKAGE_PATH, BAM_CORE_BRINGUP_PATH

from bam.msgs import (
    JointDescription,
    PerJointLimits,
    RobotInfo,
    UrdfInfo,
)

# PYTHON
from dataclasses import dataclass, field
from typing import List
import os

# Allows you to test adding the interia to the URDF
MM = 1/1000
GRAMS = 1/1000

@dataclass
class OneDofUrdfInfo(UrdfInfo):
    path:str = ""
    xacro_path:str = BAM_DESCRIPTIONS_PATH + "/urdf/one_dof/one_dof.urdf.xacro"
    abs_package_dirs: dict[str, str] = field(default_factory=lambda: {
        "descriptions": BAM_MESH_PACKAGE_PATH,
    })

@dataclass
class OneDofInfo(RobotInfo):
    type: str="one_dof"
    sku: str=""
    version: str="0.0.0"
    uuid: str="00000000-0000-0000-0000-000000000000"

J1 = JointDescription(
    name="joint_1",
    can_id=0,
    hardstop_position=0.0,
    home_position=0.0,
    
    limits=PerJointLimits(
        max_velocity=1.0,
        max_acceleration=1.0,
        max_jerk=1.0,
        max_effort=1.0,

        min_velocity=-1.0,
        min_acceleration=-1.0,
        min_jerk=-1.0,
        min_effort=-1.0,

        has_velocity_limits=True,
        has_acceleration_limits=True,
        has_jerk_limits=True,
        has_effort_limits=True,
    ),
)

@dataclass  
class OneDofDescription(RobotDescription):
    # Xacro args are now directly in RobotDescription as fields
    ros2_control: bool = True
    transport_path: str = field(default_factory=lambda: os.getenv("TRANSPORT_PATH") or "")
    plugin: str = "mock"  # options: none, mock, gazebo, real
    controllers_file_path: str = BAM_CORE_BRINGUP_PATH + "/config/controllers/one_dof_controllers.yaml"
    ros_namespace: str = "bam_GPU"

    rotor_diameter: float = 0.05
    rotor_length: float = 0.1
    rotor_mass: float = 0.2
    
    gearbox_diameter: float = 0.04
    gearbox_length: float = 0.08
    gearbox_mass: float = 0.15

    arm_length: float = 600*MM
    arm_width: float = 20*MM
    arm_mass: float = 272*GRAMS

    payload_diameter: float = 0.06
    payload_thickness: float = 0.02
    payload_mass: float = 0.1
    payload_distance: float = 1.0
    mounting_angle: float = 0.0

    # Arm Description
    base_link: str = "base_link"
    ik_tip: str = "rotor"
    trajectory_topic: str = "trajectory_controller/follow_trajectory"

    # Robot Description
    info: RobotInfo = field(default_factory=lambda: OneDofInfo())
    urdf: UrdfInfo = field(default_factory=lambda: OneDofUrdfInfo())
    joints: List[JointDescription] = field(default_factory=lambda: [J1])

    def __post_init__(self):
        super().__post_init__()

    def toggle_gravity(self, gravity: bool = True):
        self.gravity = gravity

    def set_friction(self, friction: float = 0.0):
        self.friction = friction

    @classmethod
    def make_sku_350(cls):

        return cls(
            info=OneDofInfo(sku="350"),
            arm_length=350 * MM,
            arm_width=20 * MM,
            arm_mass=160 * GRAMS,
            payload_mass=0.0,
            payload_distance=0.0,
            payload_diameter=1e-6,
            payload_thickness=1e-6,
        )

    @classmethod
    def make_sku_600(cls):

        return cls(
            info=OneDofInfo(sku="600"),
            arm_length=600 * MM,
            arm_width=20 * MM,
            arm_mass=272 * GRAMS,
            payload_mass=0.0,
            payload_distance=0.0,
            payload_diameter=1e-6,
            payload_thickness=1e-6,
        )


    @classmethod
    def make_sku_1000(cls):

        return cls(
            info=OneDofInfo(sku="1000"),
            arm_length=1000 * MM,
            arm_width=20 * MM,
            arm_mass=432 * GRAMS,
            payload_mass=0.0,
            payload_distance=0.0,
            payload_diameter=1e-6,
            payload_thickness=1e-6,
        )

        


if __name__ == '__main__':

    rd = OneDofDescription.make_sku_600()
    print(OneDofDescription.get_sku_methods())
    rd = OneDofDescription.from_sku("600")

    rd.save_yaml_file(rd.description_config_file)

    from pin_utils import MeshcatClient

    with rd.get_temp_urdf_path() as urdf_path:
        meshcat = MeshcatClient.from_urdf(urdf_path, rd.urdf.abs_package_dirs)

    
    import time
    time.sleep(1)

