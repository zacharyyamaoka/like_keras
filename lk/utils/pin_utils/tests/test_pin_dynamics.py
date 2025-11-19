#! /usr/bin/env python3

# BAM
from bam.utils.pin_utils import PinDynamics, PinViz, PinRobotModel

try:
    from bam.descriptions import (
        BAM_DESCRIPTIONS_PATH, 
        BAM_MESH_PACKAGE_PATH,
        UR_DESCRIPTION_PATH,
        UR_MESH_PACKAGE_PATH
    )

except ImportError:
    print("BAM_DESCRIPTIONS_PATH and UR_DESCRIPTION_PATH not found, make sure bam_descriptions is installed")
    
# PYTHON
import time
import pinocchio as pin
import numpy as np
import pytest

VIZ = False

@pytest.fixture(scope="module")
def ur5e_model():
    """Create UR5e robot model (shared across tests)."""
    xacro_file = UR_DESCRIPTION_PATH + "/urdf/ur.urdf.xacro"
    xacro_args = {"name": "ur5e", "ur_type": "ur5e"}
    abs_package_dirs = {
        "ur_description": UR_DESCRIPTION_PATH,
    }
    return PinRobotModel.from_xacro(xacro_file, xacro_args, abs_package_dirs, resolve_packages=True)

@pytest.fixture(scope="module")
def one_dof_base_xacro():
    """Provide base xacro configuration for one_dof robot."""
    return {
        "xacro_file": BAM_DESCRIPTIONS_PATH + "/urdf/one_dof/one_dof.urdf.xacro",
        "abs_package_dirs": {"bam_descriptions": BAM_DESCRIPTIONS_PATH},
    }

@pytest.mark.parametrize("m,l", [
    (0.0, 1.0),
    (0.5, 0.5),
    (1.0, 1.0),
])
def test_one_dof(m, l, one_dof_base_xacro):

    g = 9.81

    xacro_file = one_dof_base_xacro["xacro_file"]
    xacro_args = {
        "payload_distance": str(l),
        "rotor_mass": str(0.0),
        "gearbox_mass": str(0.0),
        "arm_mass": str(0.0), # no arm mass to avoid messing up calc
        "payload_mass": str(m),
        "mounting_angle": str(np.pi/2),
        "ros2_control": "false",  # Disable ros2_control to avoid needing bam_core_bringup package
        "controllers_file_path": "",  # Override default to avoid package lookup
    }
    abs_package_dirs = one_dof_base_xacro["abs_package_dirs"]

    pin_model = PinRobotModel.from_xacro(xacro_file, xacro_args, abs_package_dirs, resolve_packages=True)

    D = PinDynamics(pin_model)

    if VIZ:
        meshcat_client = PinViz(pin_model)   
    

    for theta in [0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, 5*np.pi/4, 3*np.pi/2, 7*np.pi/4]:
         
        q = np.array([theta])
        tau = D.rnea(q, np.array([0]), np.array([0]))
        torque = m*g*l*np.sin(theta)# T = F*r = mg*r

        diff = tau[0]-torque

        print(f"m={m}, l={l}, theta: {theta:.2f}, RNEA: {tau[0]:.3f}, Hand Calc: {torque:.3f}, diff: {diff:.3f}")
        if VIZ:
            meshcat_client.display(q)
            time.sleep(0.5)

        # assert abs(diff) < 0.1

def test_six_dof(ur5e_model):

    pin_model = ur5e_model

    D = PinDynamics(pin_model)

    if VIZ:
        meshcat_client = PinViz(pin_model)   

    # Get values by making arm completely outstretch along x axis
    q = np.array([0, 0, 0, -np.pi/2, np.pi, 0])
    data = pin_model.model.createData()
    pin.forwardKinematics(pin_model.model, data, q)
    pin.updateFramePlacements(pin_model.model, data)
    
    if VIZ:
        meshcat_client.display(q)
        time.sleep(0.5)

    payloads = {}


    for i in range(1, pin_model.model.njoints):  # Skip root joint
        joint_name = pin_model.model.names[i]
        inertia = pin_model.model.inertias[i]
        mass = inertia.mass #kg
        com = inertia.lever #m

        link_frame_id = pin_model.model.getFrameId(joint_name)
        link_pos = data.oMf[link_frame_id].translation
        
        lever_arm = abs(link_pos[0]) + abs(com[0])

        # The wrists have COM components in the y directions, but these shouldn't have an affect as not perpendicular to axis of rotation
        # Mabye that is actually where the error is coming from though.... hmmm 0.2 Nm error is not alot on 60 Nm...
        print(f"{joint_name}: ")
        print(f"    com: {com}")
        print(f"    link_origin: {link_pos}")
        print(f"    mass: {mass}")
        print(f"    lever_arm: abs({link_pos[0]:.3f}) + abs({com[0]:.3f}) = {lever_arm:.3f}")
        print(f"    torque: {mass*9.81*lever_arm:.3f} Nm")

        payloads[joint_name] = {
            "mass": mass,
            "lever_arm": lever_arm,
        }

    g = 9.81
    for theta in [0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, 5*np.pi/4, 3*np.pi/2, 7*np.pi/4]:
         
        # Make the arm fully outstretched, and turn axis 5 inwards so its more compact (all 3 wrists like a payload mass)
        q = np.array([0, -1*theta, 0, -np.pi/2, np.pi, 0])
        qd = np.array([0, 0, 0, 0, 0, 0])
        qdd = np.array([0, 0, 0, 0, 0, 0])

        tau = D.rnea(q, qd, qdd)
        torque = 0
        for joint_name, payload in payloads.items():
            torque -= payload["mass"]*g*payload["lever_arm"]*np.cos(theta) # COS as arm starts along x axis

        joint_2_idx = 1
        diff = tau[joint_2_idx]-torque # pick joint 2, as its like a pendulum
        assert abs(diff) < 0.2

        print(f"theta: {theta:.2f}, RNEA: {tau[joint_2_idx]:.3f}, Hand Calc: {torque:.3f}, diff: {diff:.3f}")

        if VIZ:
            meshcat_client.display(q)
            time.sleep(0.5)



if __name__ == "__main__":
    # test_one_dof(1.0, 1.0)
    # test_six_dof()

    pytest.main([__file__, "-v"])

