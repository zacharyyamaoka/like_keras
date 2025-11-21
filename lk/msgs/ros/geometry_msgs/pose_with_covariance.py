# https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html
from dataclasses import dataclass, field
from typing import List

from .pose import Pose


@dataclass
class PoseWithCovariance:
    pose: Pose = field(default_factory=Pose)
    covariance: List[float] = field(default_factory=lambda: [0.0] * 36)

    def to_dict(self):
        return {
            "pose": self.pose.to_dict(),
            "covariance": self.covariance,
        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.pose = Pose.from_dict(d.get("pose", {}))
        obj.covariance = d.get("covariance", [0.0] * 36)
        return obj

    def __str__(self):
        return f"[ros_py_type] {self.to_dict()}"
