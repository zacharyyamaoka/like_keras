# BAM
from .Marker import Marker

# PYTHON
from dataclasses import dataclass, field
from typing import Literal


@dataclass
class Box(Marker):
    wireframe: bool = False
    material: Literal["standard", "toon3", "toon5"] = "standard"
    flat_shading: bool = True
    side: Literal["front", "back", "double"] = "front"
    cast_shadow: bool = True
    receive_shadow: bool | float = True
