"""
    Validate that all exported bam.msgs namespaces provide usable types.
"""

# PYTHON
import pytest

# BAM
from bam.msgs.bam_msgs.action import (
    ActuatorCommand,
    ActuatorState,
    ActuatorTrajectory,
    ActuatorTrajectoryPoint,
    ArmParams,
    CartesianImpedance,
    CartesianLimits,
    HandParams,
    JointImpedance,
    JointLimits,
    JointTolerancePoint,
    MdpAction,
    MultiWaypointAction,
    NumpyTrajectory,
    PathParams,
    PathTolerance,
    WaypointAction,
    WaypointContext,
    WaypointParams,
)
from bam.msgs.bam_msgs.api import (
    ClientResponse,
    ErrorCode,
    RequestHeader,
    ResponseHeader,
)
from bam.descriptions.types import (
    Box,
    CollisionProperties,
    ConfigFileInfo,
    Cylinder,
    DropPoint,
    Geometry,
    Inertia,
    InertialProperties,
    JointCalibration,
    JointDescription,
    JointIO,
    JointMimic,
    LinkDescription,
    Material,
    Mesh,
    PerJointLimits,
    PerJointPhysics,
    PerJointState,
    PhysicalProperties,
    PointOfInterest,
    RGBA,
    RobotInfo,
    Sphere,
    SrdfInfo,
    UrdfInfo,
    VisualProperties,
)
from bam.msgs.bam_msgs.msg import Msg
from bam.msgs.bam_msgs.observation import MdpObservation
from bam.msgs.bam_msgs.state import MdpState


def test_action_types():
    actuator_command = ActuatorCommand()
    actuator_state = ActuatorState()
    actuator_trajectory = ActuatorTrajectory()
    actuator_trajectory_point = ActuatorTrajectoryPoint()
    arm_params = ArmParams()
    cartesian_impedance = CartesianImpedance()
    cartesian_limits = CartesianLimits()
    hand_params = HandParams()
    joint_impedance = JointImpedance()
    joint_limits = JointLimits()
    joint_tolerance_point = JointTolerancePoint()
    mdp_action = MdpAction()
    multi_waypoint_action = MultiWaypointAction()
    numpy_trajectory = NumpyTrajectory()
    path_params = PathParams()
    path_tolerance = PathTolerance()
    waypoint_action = WaypointAction()
    waypoint_context = WaypointContext()
    waypoint_params = WaypointParams()




def test_api_types():
    request_header = RequestHeader.create(client_id="tester", frame_id="world")
    response_header = ResponseHeader.create_success(error_msg="ok")
    client_response = ClientResponse.success(data={"status": "ok"})
    client_response_custom = ClientResponse.custom(error_code=ErrorCode.SUCCESS)

    assert request_header.client_id == "tester"
    assert response_header.success
    assert bool(client_response)
    assert client_response_custom.code == ErrorCode.SUCCESS


def test_description_types():
    # Geometry types
    box = Box(size=(1.0, 1.0, 1.0))
    assert box.length == 1.0
    cylinder = Cylinder(radius=0.5, height=1.0)
    assert cylinder.radius == 0.5
    mesh = Mesh(filename="test.stl")
    assert mesh.filename == "test.stl"
    sphere = Sphere(radius=0.5)
    assert sphere.radius == 0.5
    # Geometry is a Union type, so we can't instantiate it directly
    assert Geometry is not None
    
    # Link types
    material = Material()
    collision_properties = CollisionProperties(
    )
    inertia = Inertia()
    assert inertia is not None
    inertial_properties = InertialProperties()
    assert inertial_properties is not None
    # LinkDescription has a Union type default_factory issue, so we provide explicit values
    # VisualProperties also has Geometry Union type issue, so we provide a specific geometry type
    visual_props = VisualProperties(geometry=mesh)
    assert visual_props.geometry == mesh
    link_description = LinkDescription(
        name="test_link",
        visual=visual_props,
        collision=collision_properties,
    )
    assert link_description.name == "test_link"
    physical_properties = PhysicalProperties()
    assert physical_properties is not None
    rgba_color = RGBA.red()
    assert rgba_color is not None
    
    # Joint types - all from joints/__init__.py
    joint_calibration = JointCalibration()
    assert joint_calibration is not None
    joint_description = JointDescription()
    assert joint_description.name == ""
    joint_io = JointIO()
    assert joint_io.can_id == 0
    joint_mimic = JointMimic()
    assert joint_mimic.enabled == False
    joint_initial_state = PerJointState()
    assert joint_initial_state.position == 0.0
    joint_limits = PerJointLimits()
    assert joint_limits is not None
    joint_physics = PerJointPhysics()
    assert joint_physics is not None
    
    # Robot types
    config_file_info = ConfigFileInfo()
    assert config_file_info is not None
    robot_info = RobotInfo()
    assert robot_info is not None
    srdf_info = SrdfInfo()
    assert srdf_info is not None
    urdf_info = UrdfInfo()
    assert urdf_info is not None
    
    # Scene types
    drop_point = DropPoint()
    assert drop_point is not None
    point_of_interest = PointOfInterest()
    assert point_of_interest is not None




def test_observation_types():
    mdp_observation = MdpObservation()

    assert mdp_observation is not None


def test_state_types():
    mdp_state = MdpState()

    assert mdp_state is not None


def test_msg_type():
    msg = Msg()

    assert msg is not None


if __name__ == "__main__":

    pytest.main([__file__])

