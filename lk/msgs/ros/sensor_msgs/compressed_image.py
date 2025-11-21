# https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html

from typing import List

from ..std_msgs.header import Header


class CompressedImage:
    def __init__(self):
        self.header: Header = Header()
        self.format: str = "None"
        self.data: List[int] = []

    def to_dict(self):
        return {
            "header": self.header.to_dict(),
            "format": self.format,
            "data": self.data,
        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.header = Header.from_dict(d.get("header", {}))
        obj.format = d.get("format", "None")
        obj.data = d.get("data", [])
        return obj
