# BAM
from .pose_msg import PoseMsg
from .vector3 import Vector3
from .quaternion import Quaternion
from tf_transformations import xyzrpy_offset

# PYTHON
import numpy as np
from dataclasses import dataclass, field
from typing import List, TYPE_CHECKING


if TYPE_CHECKING:
    from .pose import Pose


"""
https://docs.ros2.org/foxy/api/geometry_msgs/msg/Transform.html
"""
@dataclass
class Transform(PoseMsg):
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)

    @property
    def pos(self) -> Vector3:
        return self.translation

    @pos.setter
    def pos(self, value: Vector3):
        self.translation = value

    @property
    def quat(self) -> Quaternion:
        return self.rotation

    @quat.setter
    def quat(self, value: Quaternion):
        self.rotation = value

    @classmethod
    def from_pose(cls, pose: 'Pose') -> 'Transform':
        return cls(translation=pose.position.to_vector3(), rotation=pose.orientation)

    def to_pose(self) -> 'Pose':
        from .pose import Pose
        return Pose(position=self.translation.to_point(), orientation=self.rotation)

if __name__ == "__main__":
    transform = Transform.random()
    pose = transform.to_pose()
    transform_from_pose = Transform.from_pose(pose)
    print(transform)
    print(transform_from_pose)
