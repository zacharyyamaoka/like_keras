#!/usr/bin/env python3
"""
Development BAM Five-Bar Arm variant.

This is the development/prototype version of the BAM five-bar arm used for testing.
"""

# BAM
from bam.descriptions import BAM_DESCRIPTIONS_PATH, Box, Mesh, Inertia
from ..bam_fb import (
    BamFb,
    BamFbArgs,
    BamFbUrdfParams,
    BamFbInfo,
    BamFbCadKinParams,
    BamFbJoints,
    BamFbLinks,
    DHParams,
    FiveBarParams,
    MM,
    REV2RAD,
    GRAMS,
    INCH,
)
from bam.msgs import Pose, Point, TransformStamped, Transform

# PYTHON
import numpy as np
from dataclasses import dataclass, field
import copy


SPACER_OFFSET = 0


@dataclass
class BamFbDevCadKinParams(BamFbCadKinParams):

    c1_to_c2_y: float = 150

    shoulder_servo_radius: float = 86 / 2
    shoulder_servo_length: float = 160
    wrist_servo_radius: float = 53 / 2
    wrist_servo_length: float = 41

    base_to_shoulder_z: float = 19

    shoulder_to_l1_z: float = 137.5
    shoulder_to_l1_y: float = 69
    l1_to_c1_z: float = shoulder_to_l1_z - 50.5

    l1_to_l2_z: float = 9 + SPACER_OFFSET
    l1_to_l2_y: float = 240

    l2_to_wrist_1_y: float = 299.658
    l2_to_wrist_1_z: float = 45.10
    l2_to_sticker_anchor_y: float = 250
    l2_to_sticker_anchor_z: float = 6 + 38.1 / 2  # elbow thickness + pipe width/2
    l2_to_elbow_tip_z: float = 0
    l2_to_elbow_tip_y: float = 120
    l2_to_elbow_tip_theta: float = np.deg2rad(45)

    wrist_1_to_wrist_2_z: float = 60
    wrist_1_to_wrist_2_y: float = 17
    wrist_2_to_wrist_3_z: float = 60
    wrist_2_to_wrist_3_y: float = 17

    c1_to_c2_z: float = 6 + SPACER_OFFSET
    c1_to_c2_y: float = 120
    c2_to_c2_tip_y: float = 260
    c2_to_c2_tip_z: float = 0

    def __post_init__(self):
        self.mm_to_m()
        super().__post_init__()


# Copy the values in here from CAD file
@dataclass
class BamFbDevArgs(BamFbArgs):
    """Development variant arguments with specific kinematic parameters."""

    cad_kin_params: BamFbDevCadKinParams = field(default_factory=BamFbDevCadKinParams)


@dataclass
class BamFbDevLinks(BamFbLinks):
    def __post_init__(self):

        arm_base_link = self.arm_base_link
        arm_base_link.visual.geometry.filename = f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/arm_base_link.stl"
        # Simple collision - box approximation
        arm_base_link.simple_collision = copy.deepcopy(arm_base_link.collision)
        arm_base_link.simple_collision.geometry = Box(
            size=(0.090000, 0.090000, 0.158846)
        )
        arm_base_link.simple_collision.origin = Pose.from_xyzrpy(
            [0.000000, 0.000000, -0.060423], [0, 0, 0]
        )
        # Mesh collision - use visual mesh
        arm_base_link.collision.geometry = Mesh(
            filename=arm_base_link.visual.geometry.filename
        )
        arm_base_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        arm_base_link.inertial.mass = 2.462518
        arm_base_link.inertial.origin = Point(x=0.000415, y=-0.000060, z=-0.053854)
        arm_base_link.inertial.inertia = Inertia(
            ixx=4.853000e-03,
            ixy=3.730417e-06,
            ixz=5.616201e-05,
            iyy=4.910000e-03,
            iyz=-7.505672e-06,
            izz=1.526000e-03,
        )

        shoulder_link = self.shoulder_link
        shoulder_link.visual.geometry.filename = f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/shoulder_link.stl"
        shoulder_link.simple_collision = copy.deepcopy(shoulder_link.collision)
        shoulder_link.simple_collision.geometry = Box(
            size=(0.123608, 0.145836, 0.180500)
        )
        shoulder_link.simple_collision.origin = Pose.from_xyzrpy(
            [0.000000, -0.018954, 0.090250], [0, 0, 0]
        )
        shoulder_link.collision.geometry = Mesh(
            filename=shoulder_link.visual.geometry.filename
        )
        shoulder_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        shoulder_link.inertial.mass = 5.476897
        shoulder_link.inertial.origin = Point(x=0.000294, y=-0.000894, z=0.090286)
        shoulder_link.inertial.inertia = Inertia(
            ixx=2.251000e-02,
            ixy=-1.439000e-04,
            ixz=1.536863e-05,
            iyy=1.587000e-02,
            iyz=1.701000e-04,
            izz=1.174000e-02,
        )

        l1_link = self.l1_link
        l1_link.visual.geometry.filename = (
            f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/l1_link.stl"
        )
        l1_link.simple_collision = copy.deepcopy(l1_link.collision)
        l1_link.simple_collision.geometry = Box(size=(0.050800, 0.279000, 0.050800))
        l1_link.simple_collision.origin = Pose.from_xyzrpy(
            [0.000000, 0.117692, 0.034483], [0, 0, 0]
        )
        l1_link.collision.geometry = Mesh(filename=l1_link.visual.geometry.filename)
        l1_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        l1_link.inertial.mass = 0.316152
        l1_link.inertial.origin = Point(x=-0.000392, y=0.083754, z=0.032170)
        l1_link.inertial.inertia = Inertia(
            ixx=2.473000e-03,
            ixy=-1.028054e-05,
            ixz=-3.805845e-06,
            iyy=2.732000e-04,
            iyz=-6.250734e-05,
            izz=2.416000e-03,
        )

        l2_link = self.l2_link
        l2_link.visual.geometry.filename = (
            f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/l2_link.stl"
        )
        l2_link.simple_collision = copy.deepcopy(l2_link.collision)
        l2_link.simple_collision.geometry = Box(size=(0.298510, 0.038100, 0.038100))
        l2_link.simple_collision.origin = Pose.from_xyzrpy(
            [0.129811, 0.000000, 0.025036], [0, 0, 0]
        )
        l2_link.collision.geometry = Mesh(filename=l2_link.visual.geometry.filename)
        l2_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        l2_link.inertial.mass = 0.476478
        l2_link.inertial.origin = Point(x=0.233009, y=-0.000032, z=0.023521)
        l2_link.inertial.inertia = Inertia(
            ixx=1.996000e-04,
            ixy=1.061019e-06,
            ixz=5.300155e-05,
            iyy=4.761000e-03,
            iyz=-1.197886e-06,
            izz=4.748000e-03,
        )

        wrist_1_link = self.wrist_1_link
        wrist_1_link.visual.geometry.filename = f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/wrist_1_link.stl"
        wrist_1_link.simple_collision = copy.deepcopy(wrist_1_link.collision)
        wrist_1_link.simple_collision.geometry = Box(
            size=(0.055523, 0.049731, 0.090000)
        )
        wrist_1_link.simple_collision.origin = Pose.from_xyzrpy(
            [0.000000, -0.004218, 0.045000], [0, 0, 0]
        )
        wrist_1_link.collision.geometry = Mesh(
            filename=wrist_1_link.visual.geometry.filename
        )
        wrist_1_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        wrist_1_link.inertial.mass = 1.059153
        wrist_1_link.inertial.origin = Point(x=-0.000025, y=-0.005215, z=0.058851)
        wrist_1_link.inertial.inertia = Inertia(
            ixx=4.132000e-04,
            ixy=-9.281213e-07,
            ixz=3.892510e-08,
            iyy=4.227000e-04,
            iyz=5.587358e-06,
            izz=3.680000e-04,
        )

        wrist_2_link = self.wrist_2_link
        wrist_2_link.visual.geometry.filename = f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/wrist_2_link.stl"
        wrist_2_link.simple_collision = copy.deepcopy(wrist_2_link.collision)
        wrist_2_link.simple_collision.geometry = Box(
            size=(0.055523, 0.049731, 0.090000)
        )
        wrist_2_link.simple_collision.origin = Pose.from_xyzrpy(
            [0.000000, -0.004218, 0.045000], [0, 0, 0]
        )
        wrist_2_link.collision.geometry = Mesh(
            filename=wrist_2_link.visual.geometry.filename
        )
        wrist_2_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        wrist_2_link.inertial.mass = 1.059153
        wrist_2_link.inertial.origin = Point(x=-0.000025, y=-0.005215, z=0.058851)
        wrist_2_link.inertial.inertia = Inertia(
            ixx=4.132000e-04,
            ixy=-9.281213e-07,
            ixz=3.892510e-08,
            iyy=4.227000e-04,
            iyz=5.587358e-06,
            izz=3.680000e-04,
        )

        c1_link = self.c1_link
        c1_link.visual.geometry.filename = (
            f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/c1_link.stl"
        )
        c1_link.simple_collision = copy.deepcopy(c1_link.collision)
        c1_link.simple_collision.geometry = Box(size=(0.045000, 0.152500, 0.006000))
        c1_link.simple_collision.origin = Pose.from_xyzrpy(
            [0.000000, 0.042143, 0.002926], [0, 0, 0]
        )
        c1_link.collision.geometry = Mesh(filename=c1_link.visual.geometry.filename)
        c1_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        c1_link.inertial.mass = 0.046321
        c1_link.inertial.origin = Point(x=0.000000, y=0.042143, z=0.002926)
        c1_link.inertial.inertia = Inertia(
            ixx=8.754954e-05,
            ixy=0.000000e00,
            ixz=0.000000e00,
            iyy=7.091365e-06,
            iyz=-1.358111e-07,
            izz=9.436970e-05,
        )

        c2_link = self.c2_link
        c2_link.visual.geometry.filename = (
            f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/c2_link.stl"
        )
        c2_link.simple_collision = copy.deepcopy(c2_link.collision)
        c2_link.simple_collision.geometry = Box(size=(0.211366, 0.301180, 0.007500))
        c2_link.simple_collision.origin = Pose.from_xyzrpy(
            [-0.000258, 0.130000, 0.003335], [0, 0, 0]
        )
        c2_link.collision.geometry = Mesh(filename=c2_link.visual.geometry.filename)
        c2_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        c2_link.inertial.mass = 0.137577
        c2_link.inertial.origin = Point(x=-0.000258, y=0.130000, z=0.003335)
        c2_link.inertial.inertia = Inertia(
            ixx=1.291000e-03,
            ixy=2.548600e-06,
            ixz=4.837000e-17,
            iyy=6.984028e-06,
            iyz=1.801000e-17,
            izz=1.297000e-03,
        )

        elbow_link = self.elbow_link
        elbow_link.visual.geometry.filename = (
            f"file://{BAM_DESCRIPTIONS_PATH}/arms/bam_fb/bam_fb_dev/mesh/elbow_link.stl"
        )
        elbow_link.simple_collision = copy.deepcopy(elbow_link.collision)
        elbow_link.simple_collision.geometry = Box(size=(0.161083, 0.161083, 0.006000))
        elbow_link.simple_collision.origin = Pose.from_xyzrpy(
            [-0.000053, 0.048986, 0.002985], [0, 0, 0]
        )
        elbow_link.collision.geometry = Mesh(
            filename=elbow_link.visual.geometry.filename
        )
        elbow_link.collision.origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])
        elbow_link.inertial.mass = 0.065172
        elbow_link.inertial.origin = Point(x=-0.000053, y=0.048986, z=0.002985)
        elbow_link.inertial.inertia = Inertia(
            ixx=1.015000e-04,
            ixy=-2.089883e-07,
            ixz=5.040000e-11,
            iyy=5.287424e-06,
            iyz=-4.613740e-08,
            izz=1.064000e-04,
        )

        super().__post_init__()


@dataclass
class BamFbDevJoints(BamFbJoints):
    def __post_init__(self):

        shoulder_joint = self.shoulder_joint
        shoulder_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.000000, 0.019000], rpy=[0.000000, -0.000000, 3.141593]
        )
        shoulder_joint.limits.max_position = 3.14
        shoulder_joint.limits.min_position = -3.14
        shoulder_joint.limits.max_velocity = 10.0
        shoulder_joint.limits.max_acceleration = 10.0
        shoulder_joint.limits.max_effort = 10.0
        shoulder_joint.limits.max_jerk = 10.0

        l1_joint = self.l1_joint
        l1_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.069000, 0.137500], rpy=[-1.570796, 1.570796, 0.000000]
        )
        l1_joint.limits.max_position = 3.14
        l1_joint.limits.min_position = -3.14
        l1_joint.limits.max_velocity = 10.0
        l1_joint.limits.max_acceleration = 10.0
        l1_joint.limits.max_effort = 10.0
        l1_joint.limits.max_jerk = 10.0

        l2_joint = self.l2_joint
        l2_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.240000, 0.009000], rpy=[3.141593, 0.000000, 3.141593]
        )
        l2_joint.limits.max_position = 3.14
        l2_joint.limits.min_position = -3.14
        l2_joint.limits.max_velocity = 10.0
        l2_joint.limits.max_acceleration = 10.0
        l2_joint.limits.max_effort = 10.0
        l2_joint.limits.max_jerk = 10.0

        wrist_1_joint = self.wrist_1_joint
        wrist_1_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.299658, 0.045100], rpy=[0.000000, -0.000000, 1.570796]
        )
        wrist_1_joint.limits.max_position = 3.14
        wrist_1_joint.limits.min_position = -3.14
        wrist_1_joint.limits.max_velocity = 10.0
        wrist_1_joint.limits.max_acceleration = 10.0
        wrist_1_joint.limits.max_effort = 10.0
        wrist_1_joint.limits.max_jerk = 10.0

        wrist_2_joint = self.wrist_2_joint
        wrist_2_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.017000, 0.060000], rpy=[1.570796, 0.000000, 3.141593]
        )
        wrist_2_joint.limits.max_position = 3.14
        wrist_2_joint.limits.min_position = -3.14
        wrist_2_joint.limits.max_velocity = 10.0
        wrist_2_joint.limits.max_acceleration = 10.0
        wrist_2_joint.limits.max_effort = 10.0
        wrist_2_joint.limits.max_jerk = 10.0

        wrist_3_joint = self.wrist_3_joint
        wrist_3_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.017000, 0.060000], rpy=[1.570796, 0.000000, 3.141593]
        )
        wrist_3_joint.limits.max_position = 3.14
        wrist_3_joint.limits.min_position = -3.14
        wrist_3_joint.limits.max_velocity = 10.0
        wrist_3_joint.limits.max_acceleration = 10.0
        wrist_3_joint.limits.max_effort = 10.0
        wrist_3_joint.limits.max_jerk = 10.0

        c1_joint = self.c1_joint
        c1_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.069000, 0.050500], rpy=[-1.570796, 1.570796, 0.000000]
        )
        c1_joint.limits.max_position = 3.14
        c1_joint.limits.min_position = -3.14
        c1_joint.limits.max_velocity = 10.0
        c1_joint.limits.max_acceleration = 10.0
        c1_joint.limits.max_effort = 10.0
        c1_joint.limits.max_jerk = 10.0

        c2_joint = self.c2_joint
        c2_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.120000, 0.006000], rpy=[0.000000, -0.000000, 0.000000]
        )
        c2_joint.limits.max_position = 3.14
        c2_joint.limits.min_position = -3.14
        c2_joint.limits.max_velocity = 10.0
        c2_joint.limits.max_acceleration = 10.0
        c2_joint.limits.max_effort = 10.0
        c2_joint.limits.max_jerk = 10.0

        c2_tip_joint = self.c2_tip_joint
        c2_tip_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.260000, 0.000000], rpy=[0.000000, -0.000000, 0.000000]
        )

        elbow_joint = self.elbow_joint
        elbow_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[0.000000, 0.000000, 0.000000], rpy=[0.000000, 0.000000, 2.356195]
        )

        elbow_tip_joint = self.elbow_tip_joint
        elbow_tip_joint.transform.transform = Transform.from_xyzrpy(
            xyz=[-0.000000, 0.120000, -0.000000], rpy=[0.000000, 0.000000, 0.000000]
        )

        super().__post_init__()


@dataclass
class BamFbDev(BamFb):
    """BAM Five-Bar Arm with 400mm reach."""

    joints: BamFbDevJoints = field(default_factory=BamFbDevJoints)
    links: BamFbDevLinks = field(default_factory=BamFbDevLinks)

    args: BamFbDevArgs = field(default_factory=BamFbDevArgs)
    info: BamFbInfo = field(default_factory=lambda: BamFbInfo(sku="dev"))
