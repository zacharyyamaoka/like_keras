
# https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html
from dataclasses import dataclass, field

from ..ros_msg import RosMsg

@dataclass
class JointState(RosMsg):
    name: list[str] = field(default_factory=list)
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)

    def __post_init__(self):
        if self.position and not self.velocity:
            self.velocity = [0.0] * len(self.position)
        if self.position and not self.effort:
            self.effort = [0.0] * len(self.position)