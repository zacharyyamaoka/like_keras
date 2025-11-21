# https://docs.ros.org/en/rolling/p/vision_msgs/msg/Point2D.html


class Point2D:
    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.x = d.get("x", 0.0)
        obj.y = d.get("y", 0.0)
        return obj
