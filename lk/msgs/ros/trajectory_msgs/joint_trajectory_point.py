from dataclasses import dataclass, field

from ..builtin_interfaces import Duration
from ..ros_msg import RosMsg

from bam.msgs.msg_utils import lerp_list

# https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html

@dataclass
class JointTrajectoryPoint(RosMsg):
    positions: list[float] = field(default_factory=list)
    velocities: list[float] = field(default_factory=list)
    accelerations: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)
    time_from_start: Duration = field(default_factory=Duration)


    def lerp(self, target: 'JointTrajectoryPoint', fraction: float) -> 'JointTrajectoryPoint':

        new_point = JointTrajectoryPoint()

        new_point.positions = lerp_list(self.positions, target.positions, fraction)
        new_point.velocities = lerp_list(self.velocities, target.velocities, fraction)
        new_point.accelerations = lerp_list(self.accelerations, target.accelerations, fraction)
        new_point.effort = lerp_list(self.effort, target.effort, fraction)
        new_point.time_from_start = lerp_list(self.time_from_start, target.time_from_start, fraction)

        return new_point