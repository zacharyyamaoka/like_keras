"""
RGB Observation Message

Contains RGB image data with camera calibration info.
Matches ROS sensor_msgs structure for compatibility.
"""

# BAM
# PYTHON
from dataclasses import dataclass, field

from lk.msgs.ros import CameraInfo, Header, Image

from .mdp_observation import MdpObservation


@dataclass
class RgbObservation(MdpObservation):
    header: Header = field(default_factory=Header)
    rgb_camera_info: CameraInfo = field(default_factory=CameraInfo)
    rgb: Image = field(default_factory=Image)
