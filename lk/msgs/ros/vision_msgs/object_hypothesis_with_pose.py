# https://docs.ros.org/en/rolling/p/vision_msgs/msg/ObjectHypothesisWithPose.html

from ..geometry_msgs.pose_with_covariance import PoseWithCovariance
from .object_hypothesis import ObjectHypothesis


class ObjectHypothesisWithPose:
    def __init__(self):
        self.hypothesis: ObjectHypothesis = ObjectHypothesis()
        self.pose: PoseWithCovariance = PoseWithCovariance()

    def to_dict(self):
        return {
            "hypothesis": self.hypothesis.to_dict(),
            "pose": self.pose.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.hypothesis = ObjectHypothesis.from_dict(d.get("hypothesis", {}))
        obj.pose = PoseWithCovariance.from_dict(d.get("pose", {}))
        return obj
