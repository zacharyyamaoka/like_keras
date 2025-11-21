from .waypoint_action import WaypointAction
from ..mdp_action import MdpAction

from dataclasses import dataclass, field


@dataclass
class MultiWaypointAction(MdpAction):

    arm_name: list[str] = field(default_factory=list)
    waypoint_action: list[WaypointAction] = field(default_factory=list)
