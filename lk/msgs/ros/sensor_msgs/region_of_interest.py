# https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RegionOfInterest.html


class RegionOfInterest:
    def __init__(self):
        self.x_offset: int = 0  # Leftmost pixel
        self.y_offset: int = 0  # Topmost pixel
        self.height: int = 0
        self.width: int = 0
        self.do_rectify: bool = False

    def to_dict(self):
        return {
            "x_offset": self.x_offset,
            "y_offset": self.y_offset,
            "height": self.height,
            "width": self.width,
            "do_rectify": self.do_rectify,
        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.x_offset = d.get("x_offset", 0)
        obj.y_offset = d.get("y_offset", 0)
        obj.height = d.get("height", 0)
        obj.width = d.get("width", 0)
        obj.do_rectify = d.get("do_rectify", False)
        return obj