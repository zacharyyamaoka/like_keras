from robot_descriptions import ur10_description
from robot_descriptions._package_dirs import get_package_dirs
import os

import pinocchio as pin

description = ur10_description

print(description.URDF_PATH)

import xacrodoc

doc = xacrodoc.XacroDoc.from_file(description.URDF_PATH, resolve_packages=False)
print(doc.dom.toprettyxml(indent="  "))

for package_dir in get_package_dirs(description):

    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        description.URDF_PATH, package_dir
    )

    try:

        model, collision_model, visual_model = pin.buildModelsFromUrdf(
            description.URDF_PATH, package_dir
        )

    except Exception as e:
        print("\n")
        print(f"Failed to build model from {package_dir}")
        print(e)
        continue

    print(f"\nSuccessfully built model from {package_dir}")
    break


model, collision_model, visual_model = pin.buildModelsFromUrdf(
    description.URDF_PATH, package_dir
)


# print(os.path.dirname(description.PACKAGE_PATH))
# model_full = pin.buildModelFromUrdf(description.URDF_PATH)
# model_mimic_from_urdf = pin.buildModelFromUrdf(description.URDF_PATH, mimic=True)

# print(f"{model_full.nq=}")
# print(f"{model_mimic_from_urdf.nq=}")

# # What should I use here as a base library in case bam.descriptions not installed?
# # https://github.com/robot-descriptions/robot_descriptions.py
# # https://github.com/google-deepmind/mujoco_menagerie?tab=readme-ov-file#via-robot-descriptions
# # Mujoco meagerie reecomdsn uses the robot_descriptions package, thats a great endorsement!
# # There is also: https://github.com/Gepetto/example-robot-data (but not as popular)

# robot_wrapper = load_robot_description("upkie_description")
# print(robot_wrapper)
# print(type(robot_wrapper))

# # print(robot.viewer)

# from bam.utils.pin_utils.pin_robot_model import PinRobotModel
# from bam.utils.pin_utils.pin_meshcat import PinMeshcat

# robot_model = PinRobotModel(robot_wrapper)

# robot_model.inspect()

# pin_meshcat = PinMeshcat(robot_model)
# pin_meshcat.sleep_for_load()

# # It would be nice to just be able to bring it up here quickly...
# # Two options I see...
