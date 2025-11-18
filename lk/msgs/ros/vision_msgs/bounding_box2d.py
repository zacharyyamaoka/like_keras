# https://docs.ros.org/en/rolling/p/vision_msgs/msg/BoundingBox2D.html
from .pose2d import Pose2D

class BoundingBox2D:
    def __init__(self):
        self.center: Pose2D = Pose2D()
        self.size_x: float = 0.0 
        self.size_y: float = 0.0 

    def to_dict(self):
        return {
            "center": self.center.to_dict(),
            "size_x": self.size_x,
            "size_y": self.size_y

        }
    
    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.center = Pose2D.from_dict(d.get("center", {}))
        obj.size_x = d.get("size_x", 0.0)
        obj.size_y = d.get("size_y", 0.0)
        return obj