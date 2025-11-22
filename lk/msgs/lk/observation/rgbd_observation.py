"""
RGBD Observation Message

Contains RGB and Depth image data with camera calibration info.
Matches ROS sensor_msgs structure for compatibility.
"""

# BAM
# PYTHON
from dataclasses import dataclass, field

from lk.msgs.ros import CameraInfo, Header, Image

from .mdp_observation import MdpObservation


@dataclass
class RgbdObservation(MdpObservation):
    header: Header = field(default_factory=Header)
    rgb_camera_info: CameraInfo = field(default_factory=CameraInfo)
    depth_camera_info: CameraInfo = field(default_factory=CameraInfo)
    rgb: Image = field(default_factory=Image)
    depth: Image = field(default_factory=Image)
