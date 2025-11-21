#! /usr/bin/env python3

from robot_descriptions.loaders.pinocchio import load_robot_description
from bam.utils.pin_utils.pin_robot_model import PinRobotModel
from bam.utils.pin_utils.pin_meshcat import PinMeshcat

import os

# What should I use here as a base library in case bam.descriptions not installed?
# https://github.com/robot-descriptions/robot_descriptions.py
# https://github.com/google-deepmind/mujoco_menagerie?tab=readme-ov-file#via-robot-descriptions
# Mujoco meagerie reecomdsn uses the robot_descriptions package, thats a great endorsement!
# There is also: https://github.com/Gepetto/example-robot-data (but not as popular)

# The issue with loading directly is that you cannot set the mimic flag!
robot_wrapper = load_robot_description("robotiq_2f85_description")
# print(robot_wrapper)
# print(type(robot_wrapper))

# print(robot.viewer)


# robot_model = PinRobotModel(robot_wrapper)

# robot_model.inspect()

# pin_meshcat = PinMeshcat(robot_model)
# pin_meshcat.sleep_for_load()

# It would be nice to just be able to bring it up here quickly...
# Two options I see...

import pinocchio as pin

from bam.utils import PinMeshcat
import numpy as np

from robot_descriptions import robotiq_2f85_description

description = robotiq_2f85_description
print(description.URDF_PATH)
print(os.path.dirname(description.PACKAGE_PATH))
model_full = pin.buildModelFromUrdf(description.URDF_PATH)
model_mimic_from_urdf = pin.buildModelFromUrdf(description.URDF_PATH, mimic=True)

print(f"{model_full.nq=}")
print(f"{model_mimic_from_urdf.nq=}")


robot_model = PinRobotModel.from_urdf(
    description.URDF_PATH,
    {"pkg_name": os.path.dirname(description.PACKAGE_PATH)},
    mimic=True,
    verbose=True,
)
print(robot_model.n_dof)
print(robot_model.get_q_neutral())
# robot_model.inspect()


meshcat = PinMeshcat(robot_model)
meshcat.display(np.array([0.5] * robot_model.n_dof))
meshcat.sleep_for_load()
