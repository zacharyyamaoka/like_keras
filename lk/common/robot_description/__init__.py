"""
    Core description building blocks and utilities.
"""

# BAM
from .description_types import (
    # Joints
    JointCalibration,
    JointDescription,
    JointIO,
    JointMimic,
    PerJointState,
    PerJointLimits,
    PerJointPhysics,
    # Links
    CollisionProperties,
    Inertia,
    InertialProperties,
    LinkDescription,
    Material,
    PhysicalProperties,
    RGBA,
    VisualProperties,
    # Geometry
    Box,
    Cylinder,
    Geometry,
    Mesh,
    Sphere,
    # Scene
    DropPoint,
    PointOfInterest,
    # Robot
    RobotInfo,
    SrdfInfo,
    UrdfInfo,
    ConfigFileInfo,
)

from .robot_description import RobotDescription

# URDF conversion functionality
from . import urdf_converter

# PYTHON

