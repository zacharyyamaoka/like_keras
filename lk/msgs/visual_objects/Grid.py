# BAM
from .VisualObject import VisualObject
from .RGBA import RGBA

# PYTHON
from bam.msgs import PoseStamped
from dataclasses import dataclass, field
from typing import Literal


@dataclass
class Grid(VisualObject):
    pose: PoseStamped = field(default_factory=lambda: PoseStamped())
    width: float = 10.0
    height: float = 10.0

    plane: Literal["xz", "xy", "yx", "yz", "zx", "zy"] = "xy"

    cell_color: RGBA = field(
        default_factory=lambda: RGBA(r=0.784, g=0.784, b=0.784, a=1.0)
    )
    cell_thickness: float = 1.0
    cell_size: float = 0.5

    section_color: RGBA = field(
        default_factory=lambda: RGBA(r=0.549, g=0.549, b=0.549, a=1.0)
    )
    section_thickness: float = 1.0
    section_size: float = 1.0

    infinite_grid: bool = False
    fade_distance: float = 100.0
    fade_strength: float = 1.0
    fade_from: Literal["camera", "origin"] = "camera"

    shadow_opacity: float = 0.125
