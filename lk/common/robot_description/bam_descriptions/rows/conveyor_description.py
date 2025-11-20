#!/usr/bin/env python3

"""
    Conveyor Description
    
    Hardware description for a conveyor belt system.
    Includes geometry, joint limits, and servo properties.
"""

# BAM
from bam.descriptions import (
    RobotDescription, 
    Joints,
    Links,
    DescriptionArgs,
    PerJointLimits,
    RobotInfo,
    UrdfInfo,
    JointDescription,
    LinkDescription,
)
    
from bam.descriptions import BAM_DESCRIPTIONS_PATH, BAM_MESH_PACKAGE_PATH, TAGS
from .. import BamUrdfInfo
# PYTHON
from dataclasses import dataclass, field
import os

@dataclass
class ConveyorUrdfInfo(BamUrdfInfo):
    macro_path: str = BAM_DESCRIPTIONS_PATH + "/urdf/conveyor/conveyor_macro.xacro"
    xacro_path: str = BAM_DESCRIPTIONS_PATH + "/urdf/conveyor/conveyor_config.urdf.xacro"
    abs_package_dirs: dict[str, str] = field(default_factory=lambda: {
        "descriptions": BAM_MESH_PACKAGE_PATH,
    })


@dataclass
class ConveyorInfo(RobotInfo):
    name: str = "conveyor"
    type: str = "conveyor"
    sku: str = "v1"
    version: str = "0.0.0"
    save_dir: str = os.path.join(os.path.dirname(__file__), f"{os.path.splitext(os.path.basename(__file__))[0]}_configs")


@dataclass
class ConveyorJoints(Joints):
    """Conveyor has a fixed belt joint (motor tracked via CAN ID, not part of kinematics)."""
    belt_joint: JointDescription = field(default_factory=lambda: JointDescription(
        name="belt_joint",
        type="fixed",
        limits=PerJointLimits(
            max_velocity=10.0,  # rad/s (for reference only)
            max_effort=10.0,  # Nm (for reference only)
            has_position_limits=False,
            has_velocity_limits=True,
            has_effort_limits=True
        )
    ))


@dataclass
class ConveyorLinks(Links):
    """Conveyor links - no scene object collision needed (objects are placed ON the conveyor)."""
    surface_center: LinkDescription = field(default_factory=lambda: LinkDescription(name="surface_center", scene_obj_collision=False))  # No collision with scene objects
    center: LinkDescription = field(default_factory=lambda: LinkDescription(name="center", scene_obj_collision=False))  # No collision with scene objects
    surface_front: LinkDescription = field(default_factory=lambda: LinkDescription(name="surface_front", scene_obj_collision=False))  # No collision with scene objects
    roller_front: LinkDescription = field(default_factory=lambda: LinkDescription(name="roller_front", scene_obj_collision=False))  # No collision with scene objects
    roller_back: LinkDescription = field(default_factory=lambda: LinkDescription(name="roller_back", scene_obj_collision=False))  # No collision with scene objects

@dataclass
class ConveyorArgs(DescriptionArgs):
    # Geometry (x=width, y=length, z=height/roller diameter)
    width: float = 0.4  # meters
    length: float = 2.0  # meters
    height: float = 0.05  # meters (also roller diameter)
    
    # Railing geometry
    railing_width: float = 0.02  # meters (thickness of railing)
    railing_height_above: float = 0.05  # meters (extension above belt surface)
    railing_height_below: float = 0.0  # meters (extension below belt bottom)
    
    # Hardware limits
    max_speed: float = 1.0  # m/s
    max_acceleration: float = 0.5  # m/s^2

@dataclass
class ConveyorDescription(RobotDescription):
    """
        Hardware description for conveyor belt.
        
        Geometry: (x=width, y=length, z=height/roller_diameter)
        Frame structure:
        - surface_center: Center of top surface (PRIMARY REFERENCE)
        - center: Geometric center of conveyor box
        - surface_front: Front edge of top surface
        - roller_front: Front roller position
        - roller_back: Back roller position
    """
    

    
    info: ConveyorInfo = field(default_factory=ConveyorInfo)
    joints: ConveyorJoints = field(default_factory=ConveyorJoints)
    links: ConveyorLinks = field(default_factory=ConveyorLinks)
    urdf: ConveyorUrdfInfo = field(default_factory=ConveyorUrdfInfo)
    args: ConveyorArgs = field(default_factory=ConveyorArgs)

    def __post_init__(self):
        super().__post_init__()


    @classmethod
    def make(cls,
        width: float = 0.4,
        length: float = 2.0,
        height: float = 0.05,
        railing_width: float = 0.02,
        railing_height_above: float = 0.05,
        railing_height_below: float = 0.0,
        max_speed: float = 1.0,
        max_acceleration: float = 0.5,
    ):
        args = ConveyorArgs(
            width=width,
            length=length,
            height=height,
            railing_width=railing_width,
            railing_height_above=railing_height_above,
            railing_height_below=railing_height_below,
            max_speed=max_speed,
            max_acceleration=max_acceleration)
        return cls(args=args)

    @property
    def roller_diameter(self) -> float:
        """Roller diameter equals conveyor height."""
        return self.args.height
    
    @property
    def roller_radius(self) -> float:
        """Roller radius."""
        return self.args.height / 2.0
    
    @property
    def railing_total_height(self) -> float:
        """Total height of railing: height + height_above + height_below."""
        return self.args.height + self.args.railing_height_above + self.args.railing_height_below
    
    @property
    def railing_z_offset(self) -> float:
        """Z offset for railing center (centered with full railing height).
        
        Railing spans from:
        - Bottom: -height - railing_height_below
        - Top: +railing_height_above
        Center is at the midpoint.
        """
        bottom = -self.args.height - self.args.railing_height_below
        top = self.args.railing_height_above
        return (bottom + top) / 2.0
    
    @property
    def roller_front_y(self) -> float:
        """Y position of front roller center (outer edge at +length/2)."""
        return self.args.length / 2.0 - self.roller_radius
    
    @property
    def roller_back_y(self) -> float:
        """Y position of back roller center (outer edge at -length/2)."""
        return -self.args.length / 2.0 + self.roller_radius
    
    @property
    def belt_length(self) -> float:
        """Length of belt box (shortened to expose rollers at front/back)."""
        return self.args.length - self.args.height
