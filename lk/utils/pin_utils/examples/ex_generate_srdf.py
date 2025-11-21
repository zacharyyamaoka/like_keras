from bam.utils.pin_utils import generate_srdf_from_urdf, PinRobotModel, PinMeshcat


from robot_descriptions.loaders.pinocchio import load_robot_description

from robot_descriptions._package_dirs import get_package_dirs
from robot_descriptions import ur10_description
import os

description = ur10_description

package_dirs = get_package_dirs(description)


# print(description.PACKAGE_PATH)
# print(description.URDF_PATH)


# # For UR, you need to go up alot!
# # You can also pass in all the different directories...

robot_model = PinRobotModel.from_urdf(description.URDF_PATH, package_dirs)

# robot_model.inspect()

# meshcat = PinMeshcat(robot_model)
# meshcat.sleep_for_load()

srdf_path = os.path.join(os.path.dirname(__file__), "example.srdf")

generate_srdf_from_urdf(
    description.URDF_PATH, package_dirs, srdf_path, num_samples=1000, verbose=True
)
