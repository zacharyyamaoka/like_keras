# https://docs.ros.org/en/rolling/p/vision_msgs/msg/Pose2D.html
from .point2d import Point2D


class Pose2D:
    def __init__(self):
        self.position: Point2D = Point2D()
        self.theta: float = 0.0

    def to_dict(self):
        return {"position": self.position.to_dict(), "theta": self.theta}

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.position = Point2D.from_dict(d.get("position", {}))
        obj.theta = d.get("theta", 0.0)
        return obj
