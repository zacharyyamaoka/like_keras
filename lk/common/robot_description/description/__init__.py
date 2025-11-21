"""
Dataclasses describing robot components and metadata.
"""

# BAM
from .joints import (
    JointCalibration,
    JointDescription,
    JointIO,
    JointMimic,
    PerJointState,
    PerJointLimits,
    PerJointPhysics,
)
from .links import (
    CollisionProperties,
    Inertia,
    InertialProperties,
    LinkDescription,
    Material,
    PhysicalProperties,
    RGBA,
    VisualProperties,
)
from .links.geometry import (
    Box,
    Cylinder,
    Geometry,
    Mesh,
    Sphere,
)
from .scene import DropPoint, PointOfInterest
from .robot import RobotInfo, SrdfInfo, UrdfInfo, ConfigFileInfo
