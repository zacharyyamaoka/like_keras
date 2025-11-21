# https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html

from typing import List

from ..std_msgs.header import Header


class Image:
    def __init__(self):
        self.header: Header = Header()

        self.height: int = 0
        self.width: int = 0

        self.encoding: str = ""

        self.is_bigendian: int = 0
        self.step: int = 0
        self.data: List[int] = []

    def to_dict(self):
        return {
            "header": self.header.to_dict(),
            "height": self.height,
            "width": self.width,
            "encoding": self.encoding,
            "is_bigendian": self.is_bigendian,
            "step": self.step,
            "data": self.data,
        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.header = Header.from_dict(d.get("header", {}))

        obj.height = d.get("height", 0)
        obj.width = d.get("width", 0)

        obj.encoding = d.get("encoding", "")

        obj.is_bigendian = d.get("is_bigendian", 0)
        obj.step = d.get("step", 0)
        obj.data = d.get("data", [])
        return obj
