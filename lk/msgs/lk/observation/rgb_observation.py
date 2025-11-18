"""
    RGB Observation Message
    
    Contains RGB image data with camera calibration info.
    Matches ROS sensor_msgs structure for compatibility.
"""

# BAM
from .mdp_observation import MdpObservation
from bam.msgs.ros_msgs.std_msgs import Header
from bam.msgs.ros_msgs.sensor_msgs import CameraInfo, Image

# PYTHON
from dataclasses import dataclass, field


@dataclass
class RgbObservation(MdpObservation):
    header: Header = field(default_factory=Header)
    rgb_camera_info: CameraInfo = field(default_factory=CameraInfo)
    rgb: Image = field(default_factory=Image)

