# BAM
from .point import Point
from .quaternion import Quaternion
from .vector3 import Vector3
from .pose_msg import PoseMsg

# PYTHON
import numpy as np
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .transform import Transform

"""
For simplicity and lower dimensional rep. gym uses euler.
ROS internally uses quaternions

This class will internally store as quaternions but output like euler unless otherwise specified.
"""


@dataclass
class Pose(PoseMsg):
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)

    @property
    def pos(self) -> Point:
        return self.position

    @pos.setter
    def pos(self, value: Point):
        self.position = value

    @property
    def quat(self) -> Quaternion:
        return self.orientation

    @quat.setter
    def quat(self, value: Quaternion):
        self.orientation = value

    @classmethod
    def from_transform(cls, transform: "Transform") -> "Pose":
        return cls(
            position=transform.translation.to_point(), orientation=transform.rotation
        )

    def to_transform(self) -> "Transform":
        from .transform import Transform

        return Transform(
            translation=self.position.to_vector3(), rotation=self.orientation
        )


if __name__ == "__main__":

    pose = Pose.random()
    Pose.from_matrix()
    print(pose)
    # matrix = pose.to_matrix()|
    # pose_from_matrix = Pose.from_matrix(matrix)

    # print("Basic random pose:")
    # print(pose)
    # print(pose_from_matrix)

    # diff = pose.difference(pose_from_matrix)
    # print(f"Difference: {diff}\n")

    # # Test with limits
    # xyz_lower = [0.0, 0.0, 0.5]
    # xyz_upper = [1.0, 1.0, 1.5]
    # rpy_lower = [-0.1, -0.1, -0.1]
    # rpy_upper = [0.1, 0.1, 0.1]

    # pose_limited = Pose.random(xyz_lower=xyz_lower, xyz_upper=xyz_upper, rpy_lower=rpy_lower, rpy_upper=rpy_upper)
    # print(f"Limited random pose (xyz: {xyz_lower} to {xyz_upper}, rpy: {rpy_lower} to {rpy_upper}):")
    # print(f"  xyz: {pose_limited.xyz}")
    # print(f"  rpy: {pose_limited.rpy}")
