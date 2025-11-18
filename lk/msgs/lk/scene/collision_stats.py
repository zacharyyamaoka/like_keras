

from dataclasses import dataclass


@dataclass
class CollisionStats:
    is_collision: bool = False
    collision_time: float = 0.0