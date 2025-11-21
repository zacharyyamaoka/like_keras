#!/usr/bin/env python3

"""
Description Types Facade

This module provides a flat import interface for all description types.
Instead of importing from nested modules, users can import all types
from this single module.

This also helps break circular import dependencies.
"""

# BAM
from .description import (
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

# PYTHON
