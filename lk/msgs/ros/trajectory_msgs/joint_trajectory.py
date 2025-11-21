from dataclasses import dataclass, field

from ..std_msgs import Header
from .joint_trajectory_point import JointTrajectoryPoint
from ..builtin_interfaces import Duration

# https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectory.html


@dataclass
class JointTrajectory:
    header: Header = field(default_factory=Header)
    joint_names: list[str] = field(default_factory=list)
    points: list[JointTrajectoryPoint] = field(default_factory=list)

    @classmethod
    def from_lists(
        cls,
        time_from_start: list[float],
        header: Header = None,
        joint_names: list[str] = None,
        positions: list[list[float]] = None,
        velocities: list[list[float]] = None,
        accelerations: list[list[float]] = None,
        effort: list[list[float]] = None,
    ) -> "JointTrajectory":

        points = []

        for i in range(len(time_from_start)):
            point = JointTrajectoryPoint()

            if positions:
                point.positions = positions[i]
            if velocities:
                point.velocities = velocities[i]
            if accelerations:
                point.accelerations = accelerations[i]
            if effort:
                point.effort = effort[i]
            if time_from_start:
                point.time_from_start = Duration.from_float(time_from_start[i])

            points.append(point)

        traj = cls()
        if header:
            traj.header = header
        if joint_names:
            traj.joint_names = joint_names
        traj.points = points

        return traj
