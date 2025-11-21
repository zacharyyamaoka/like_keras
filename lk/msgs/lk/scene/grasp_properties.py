from dataclasses import dataclass
from typing import Literal

MM = 1 / 1000


@dataclass
class GraspProperties:
    surface_compression: float = (
        1 * MM
    )  # mm, how deep should a succesful grasp be into the surface?
    surface_clearance: float = (
        30 * MM
    )  # Between the object grasp surface and the table, how much space should we ensure there is? Ex. if doing a pinch grasp on a book, need distance underneath
    grasp_opening_axis: Literal["x", "y"] = (
        "x"  # Which axis the hand opens along (for boxes, determines which dimension is the grasp width)
    )
