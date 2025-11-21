from dataclasses import dataclass, field
from typing import Literal, Optional

from bam.msgs.ros_msgs import Pose
from .geometry import Geometry
from .material import Material


@dataclass
class CollisionProperties:
    name: Optional[str] = None
    origin: Pose = field(default_factory=Pose)
    geometry: Geometry = field(default_factory=Geometry)
