"""
Joint description dataclass capturing URDF-relevant metadata.
"""

# BAM
from bam.msgs.ros_msgs import TransformStamped, Vector3

# PYTHON
from dataclasses import dataclass, field
from typing import Literal

from .per_joint_limits import PerJointLimits
from .per_joint_physics import PerJointPhysics
from .joint_io import JointIO
from .joint_calibration import JointCalibration
from .joint_mimic import JointMimic


# Positions will be kept at the robot level, as more meaningful to look at groups of joints
@dataclass
class JointDescription:
    name: str = ""
    unprefixed_name: str = ""
    prefixes: list[str] = field(default_factory=list)
    type: Literal[
        "revolute", "continuous", "prismatic", "fixed", "floating", "planar"
    ] = "revolute"

    axis: Vector3 = field(default_factory=Vector3)

    initial_position: float = 0.0

    mimic: JointMimic = field(default_factory=JointMimic)
    calibration: JointCalibration = field(default_factory=JointCalibration)

    transform: TransformStamped = field(default_factory=TransformStamped)

    limits: PerJointLimits = field(default_factory=PerJointLimits)
    physics: PerJointPhysics = field(default_factory=PerJointPhysics)

    io: JointIO = field(default_factory=JointIO)

    tags: list[str] = field(default_factory=list)

    def __post_init__(self):
        if not self.unprefixed_name:
            self.unprefixed_name = self.name

    @property
    def parent_link(self) -> str:
        """Get parent link from transform frame_id."""
        return self.transform.header.frame_id

    @property
    def child_link(self) -> str:
        """Get child link from transform child_frame_id."""
        return self.transform.child_frame_id

    @property
    def is_fixed(self) -> bool:
        return self.type == "fixed"

    @property
    def is_mimic(self) -> bool:
        return self.mimic.is_enabled()

    @property
    def is_actuated(self) -> bool:
        return not self.is_fixed and not self.is_mimic

    def to_xml(self) -> str:
        """Generate URDF XML representation of the joint.

        Returns:
            str: URDF XML string for the joint (assumes 2-space indentation)
        """
        # Format xyz and rpy from transform
        xyz = " ".join([f"{v:.6f}" for v in self.transform.xyz])
        rpy = " ".join([f"{v:.6f}" for v in self.transform.rpy])
        parent_link = self.transform.header.frame_id
        child_link = self.transform.child_frame_id

        return (
            f'<joint name="{self.name}" type="{self.type}">\n'
            f'  <origin xyz="{xyz}" rpy="{rpy}"/>\n'
            f'  <parent link="{parent_link}"/>\n'
            f'  <child link="{child_link}"/>\n'
            f"</joint>"
        )


if __name__ == "__main__":
    joint = JointDescription(name="joint_example")
    print(joint.parent_link)
    print(joint.to_xml())
