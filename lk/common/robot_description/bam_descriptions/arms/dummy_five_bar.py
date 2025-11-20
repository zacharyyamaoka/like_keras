#!/usr/bin/env python3


# BAM
from bam.descriptions.arms.arm_description import RobotDescription
from bam_utils import BAM_DESCRIPTIONS_PATH
from bam.descriptions import BAM_MESH_PACKAGE_PATH

# PYTHON
import numpy as np
from typing import List

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

class DummyFiveBar(RobotDescription):

    def __init__(
        self,
        dh_d1: float = 0.1565, # z distance from origin to joint 2
        j2_axis_offset: float = 69/1000, # distance from z axis to joint 2, Check y joint value in  <joint name="${prefix}joint_2" type="revolute">
        L_l1: float = 0.240,
        L_l2: float = 0.240,
        L_c1: float = 0.120,
        L_c2: float = 0.265,
        L_shoulder: float = 0.087,
        L_elbow: float = 0.120,
        theta_elbow_offset: float = np.deg2rad(45.0),
        reflect: int = 1,
        prefix: str = "",
        world_xyz=(0,0,0),
        world_rpy=(0,0,0),
        base_xyz=(0,0,0),
        base_rpy=(0,0,-np.pi/2),
        only_five_bar: bool = False, # If true, only the five bar is loaded, no other joints
        k_damping: float = 0.1,
        k_friction: float = 0.1,
        **kwargs
    ):
        # Store variables before init
        self.dh_d1 = dh_d1
        self.j2_axis_offset = j2_axis_offset

        self.L_l1 = L_l1
        self.L_l2 = L_l2
        self.L_c1 = L_c1
        self.L_c2 = L_c2
        self.L_shoulder = L_shoulder
        self.L_elbow = L_elbow
        self.theta_elbow_offset = theta_elbow_offset
        self.reflect = reflect
        self.prefix = prefix
        self.world_xyz = world_xyz
        self.world_rpy = world_rpy

        self.base_xyz = base_xyz
        self.base_rpy = base_rpy

        self.only_five_bar = only_five_bar

        self.c2_coupler_name = "C2_coupler"
        self.l2_coupler_name = "L2_coupler"

        self.k_damping = k_damping
        self.k_friction = k_friction

        super().__init__(**kwargs)

    def _init_joint_names(self):

        self.joint_names: List[str] = [
            "joint_c1",
            "joint_c2",
            "L2_coupler_joint",
            "joint_2",
            "joint_3",
        ]
        self.mimic_joint_names: List[str] = []

    def _init_five_bar(self):
        self.five_bar_dict = {
            "L_l1": self.L_l1,
            "L_l2": self.L_l2,
            "L_c1": self.L_c1,
            "L_c2": self.L_c2,
            "L_shoulder": self.L_shoulder,
            "L_elbow": self.L_elbow,
            "theta_elbow_offset": self.theta_elbow_offset,
        }

    def _init_dynamics(self):
        """ Define locking behavior and default positions.
        
            Used like:
                model = reduce_model(model, rp.lock_joints, rp.model_lock_position)
        """
        self.lock_joints: List[str] = ["L2_coupler_joint"]
        self.model_lock_position = None # use pin.neutral()


    def _init_urdf(self):

        self.xacro_args = {
            "reflect": str(self.reflect),
            "prefix": self.prefix,
            "world_xyz": ' '.join(map(str, self.world_xyz)),
            "world_rpy": ' '.join(map(str, self.world_rpy)),
            "base_xyz": ' '.join(map(str, self.base_xyz)),
            "base_rpy": ' '.join(map(str, self.base_rpy)),
            "k_damping": str(self.k_damping),
            "k_friction": str(self.k_friction),
        }

        if self.only_five_bar:
            self.urdf_path = BAM_DESCRIPTIONS_PATH + "/urdf/dummy_five_bar/five_bar.urdf.xacro"
        else:
            self.urdf_path = BAM_DESCRIPTIONS_PATH + "/urdf/dummy_five_bar/dummy_five_bar.urdf.xacro"

            self.xacro_args = {
                **self.xacro_args,
                "dh_d1": str(self.dh_d1),
                "j2_axis_offset": str(self.j2_axis_offset),
            }



        five_bar_dict_str = {k: str(v) for k, v in self.five_bar_dict.items()}

        self.xacro_args = {**self.xacro_args, **five_bar_dict_str}


        self.abs_package_dirs = {"descriptions": BAM_MESH_PACKAGE_PATH}  # where does the urdf mesh point to?

if __name__ == '__main__':

    rd = DummyFiveBar()
    print(rd)
