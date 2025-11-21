from dataclasses import dataclass, field

from .actuator.actuator_trajectory import ActuatorTrajectory


@dataclass
class MdpAction:

    traj: ActuatorTrajectory = field(default_factory=ActuatorTrajectory)

    ...
