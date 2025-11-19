from robot_descriptions.loaders.pinocchio import load_robot_description


# What should I use here as a base library in case bam.descriptions not installed? 
# https://github.com/robot-descriptions/robot_descriptions.py
# https://github.com/google-deepmind/mujoco_menagerie?tab=readme-ov-file#via-robot-descriptions
# Mujoco meagerie reecomdsn uses the robot_descriptions package, thats a great endorsement!
# There is also: https://github.com/Gepetto/example-robot-data (but not as popular)

robot_wrapper = load_robot_description("upkie_description")
print(robot_wrapper)
print(type(robot_wrapper))

# print(robot.viewer)

from bam.utils.pin_utils.pin_robot_model import PinRobotModel
from bam.utils.pin_utils.pin_meshcat import PinMeshcat

robot_model = PinRobotModel(robot_wrapper)

robot_model.inspect()

pin_meshcat = PinMeshcat(robot_model)
pin_meshcat.sleep_for_load()

# It would be nice to just be able to bring it up here quickly...
# Two options I see...
