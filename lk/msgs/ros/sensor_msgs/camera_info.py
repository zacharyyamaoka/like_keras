# https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
# https://docs.ros2.org/foxy/api/sensor_msgs/msg/CameraInfo.html Its updated in ROS2

from typing import List
from dataclasses import dataclass

from ..std_msgs.header import Header
from .region_of_interest import RegionOfInterest
from ..ros_msg import RosMsg


@dataclass
class CameraInfo(RosMsg):
    def __init__(self):
        # Image acquisition info
        self.header: Header = Header()
        self.height: int = 0
        self.width: int = 0

        # Calibration parameters
        self.distortion_model: str = ""
        self.d: List[float] = []  # Distortion coefficients
        self.k: List[float] = [0.0] * 9  # Intrinsic matrix
        self.r: List[float] = [0.0] * 9  # Rectification matrix
        self.p: List[float] = [0.0] * 12  # Projection matrix

        # Operational parameters
        self.binning_x: int = 0
        self.binning_y: int = 0
        self.roi: RegionOfInterest = RegionOfInterest()

    def to_dict(self):
        return {
            "header": self.header.to_dict(),
            "height": self.height,
            "width": self.width,
            "distortion_model": self.distortion_model,
            "d": self.d,
            "k": self.k,
            "r": self.r,
            "p": self.p,
            "binning_x": self.binning_x,
            "binning_y": self.binning_y,
            "roi": self.roi.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict):
        obj = cls()
        obj.header = Header.from_dict(d.get("header", {}))
        obj.height = d.get("height", 0)
        obj.width = d.get("width", 0)

        obj.distortion_model = d.get("distortion_model", "")
        obj.d = d.get("d", [])
        obj.k = d.get("k", [0.0] * 9)
        obj.r = d.get("r", [0.0] * 9)
        obj.p = d.get("p", [0.0] * 12)

        obj.binning_x = d.get("binning_x", 0)
        obj.binning_y = d.get("binning_y", 0)
        obj.roi = RegionOfInterest.from_dict(d.get("roi", {}))
        return obj
