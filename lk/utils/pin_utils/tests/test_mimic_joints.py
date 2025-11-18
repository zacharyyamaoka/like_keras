
#! /usr/bin/env python3


from bam.utils.pin_utils.pin_robot_model import PinRobotModel
from bam.utils.pin_utils.pin_meshcat import PinMeshcat
from robot_descriptions import robotiq_2f85_description 
import os
import pinocchio as pin


def test_mimic_dof():
    description = robotiq_2f85_description

    model_full = pin.buildModelFromUrdf(description.URDF_PATH)
    model_mimic_from_urdf = pin.buildModelFromUrdf(description.URDF_PATH, mimic=True)

    assert model_full.nq == 6
    assert model_mimic_from_urdf.nq == 1



    robot_model = PinRobotModel.from_urdf(description.URDF_PATH, {"pkg_name": os.path.dirname(description.PACKAGE_PATH)}, mimic=False)
    assert robot_model.n_dof == 6


    robot_model = PinRobotModel.from_urdf(description.URDF_PATH, {"pkg_name": os.path.dirname(description.PACKAGE_PATH)}, mimic=True)
    assert robot_model.n_dof == 1


if __name__ == "__main__":
    test_mimic_dof()