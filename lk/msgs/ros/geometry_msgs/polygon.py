# https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Polygon.html
# its actually Point32 but has same fields as Point
from dataclasses import dataclass, field
from typing import List

from .point import Point


@dataclass
class Polygon:
    points: List[Point] = field(default_factory=list)  # List of [x, y]

    def to_dict(self):
        return {
            "points": [p.to_dict() for p in self.points],
        }

    @classmethod
    def from_dict(cls, d: dict):
        points_data = d.get("points", [])
        points = [Point.from_dict(p) for p in points_data]
        return cls(points=points)
