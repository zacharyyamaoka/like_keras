# https://docs.ros.org/en/rolling/p/vision_msgs/msg/Detection2D.html

from typing import List

from ..std_msgs.header import Header
from .object_hypothesis_with_pose import ObjectHypothesisWithPose
from .bounding_box2d import BoundingBox2D

class Detection2D:
    def __init__(self):
        self.header: Header = Header()
        self.results: List[ObjectHypothesisWithPose] = []
        self.bbox: BoundingBox2D = BoundingBox2D()
        self.id: str = ""

    def to_dict(self):
        return {
            "header": self.header.to_dict(),
            "results": [r.to_dict() for r in self.results],
            "bbox": self.bbox.to_dict(),
            "id": self.id,

        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.header = Header.from_dict(d.get("header", {}))
        obj.results = [ObjectHypothesisWithPose.from_dict(r) for r in d.get("results", [])]
        obj.bbox = BoundingBox2D.from_dict(d.get("bbox", {}))
        obj.id = d.get("id", "")
        return obj