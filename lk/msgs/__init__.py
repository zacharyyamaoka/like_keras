"""
Flat import facade for BAM message helpers.
"""

# BAM
from .lk import (
    # Action
    ActuatorCommand,
    ActuatorState,
    ActuatorTrajectory,
    ActuatorTrajectoryPoint,
    ArmParams,
    CartesianImpedance,
    CartesianLimits,
    # API
    ClientResponse,
    ErrorCode,
    HandParams,
    JointImpedance,
    JointLimits,
    JointTolerancePoint,
    MdpAction,
    MdpObservation,
    MdpState,
    # Core
    Msg,
    MultiWaypointAction,
    NumpyTrajectory,
    PathParams,
    PathTolerance,
    RequestHeader,
    ResponseHeader,
    RgbdObservation,
    RgbObservation,
    WaypointAction,
    WaypointContext,
    WaypointParams,
)

# Description types that depend on 'bam' package - commented out for standalone use
# from .lk import (
#     RGBA,
#     Box,
#     CollisionProperties,
#     ConfigFileInfo,
#     Cylinder,
#     DropPoint,
#     Geometry,
#     Inertia,
#     InertialProperties,
#     JointCalibration,
#     JointDescription,
#     JointIO,
#     JointMimic,
#     LinkDescription,
#     Material,
#     Mesh,
#     PerJointLimits,
#     PerJointPhysics,
#     PerJointState,
#     PhysicalProperties,
#     PointOfInterest,
#     RobotInfo,
#     Sphere,
#     SrdfInfo,
#     UrdfInfo,
#     VisualProperties,
# )

# Data Distributions
from .data_dist import (
    RandomFloat,
    RandomInt,
)

# ROS
from .ros import (
    # Vision
    BoundingBox2D,
    # Sensor
    CameraInfo,
    CompressedImage,
    Detection2D,
    # Builtin Interfaces
    Duration,
    Header,
    Image,
    JointState,
    # Trajectory
    JointTrajectory,
    JointTrajectoryPoint,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    # Geometry
    Point,
    Point2D,
    Polygon,
    Pose,
    Pose2D,
    PoseStamped,
    PoseType,
    PoseWithCovariance,
    Quaternion,
    RegionOfInterest,
    # Core
    RosMsg,
    Time,
    Transform,
    TransformStamped,
    Vector3,
)

# Visual Objects
# from . import visual_objects
# from .visual_objects import (
#     VisualObject,
#     VisualId,
#     CompoundVisualObject,
#     Frame,
#     UrdfFrame,
#     LineSegments,
#     Grid,
#     PointCloud,
#     Urdf,
#     Marker,
#     Arrow,
#     Path,
#     RList,
#     ScoreCloud,
#     GraspPath,
#     ParallelGripper,
#     ClawGripper,
#     GraspGeometry,
#     Conveyor,
#     NamespaceVisibilityGui,
#     PlaybackGui,
#     PointCloudGui,
#     PoseSelectorGui,
#     ReachMapGui,
#     ScoreCloudGui,
#     UrdfJointControlGui,
#     LineChart,
#     Histogram,
#     Series,
#     LinePlot,
#     VerticalLine,
# )
