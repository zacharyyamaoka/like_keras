from robot_descriptions.loaders.pinocchio import load_robot_description

robot_wrapper = load_robot_description("upkie_description")
print(robot_wrapper)
print(type(robot_wrapper))

# print(robot.viewer)

from bam.utils.pin_utils.pin_robot_model import PinRobotModel
from bam.utils.pin_utils.pin_viser import PinViser

robot_model = PinRobotModel(robot_wrapper)

robot_model.inspect()

pin_viser = PinViser(robot_model)
pin_viser.add_robot_control_sliders()
pin_viser.block_until_input()

# It would be nice to just be able to bring it up here quickly...
# Two options I see...
