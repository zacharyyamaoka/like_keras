"""
Link description dataclass capturing geometry and inertial metadata.
"""

# BAM
from bam.msgs.ros_msgs import Pose

# PYTHON
from dataclasses import dataclass, field
from typing import Tuple, Optional

from .inertial_properties import InertialProperties
from .collision_properties import CollisionProperties
from .visual_properties import VisualProperties


@dataclass
class LinkDescription:
    name: str = ""
    unprefixed_name: str = None
    prefixes: list[str] = field(default_factory=list)

    visual: VisualProperties | list[VisualProperties] = field(
        default_factory=VisualProperties
    )

    collision: CollisionProperties | list[CollisionProperties] = field(
        default_factory=CollisionProperties
    )
    simple_collision: Optional[CollisionProperties | list[CollisionProperties]] = None

    inertial: InertialProperties = field(default_factory=InertialProperties)

    disable: bool = False
    is_frame: bool = False
    display: bool = False
    scene_obj_collision: bool = False

    tags: list[str] = field(default_factory=list)

    def __post_init__(self):
        if self.unprefixed_name is None:
            self.unprefixed_name = self.name

    def set_simple_collision(self):
        if self.simple_collision is not None:
            self.collision = self.simple_collision
        return self

    def to_xml(self) -> str:
        """Generate URDF XML representation of the link.

        Returns:
            str: URDF XML string for the link (assumes 2-space indentation)
        """
        return f'  <link name="{self.name}"/>'


if __name__ == "__main__":
    link = LinkDescription(name="example_link")
    print(link.inertial)
    print(link.to_xml())
