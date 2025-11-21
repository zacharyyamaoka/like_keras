# BAM
from .Marker import Marker

# PYTHON
from dataclasses import dataclass
from typing import Literal


@dataclass
class Sphere(Marker):
    radius: float = 0.05
    subdivisions: int = 3
    wireframe: bool = False
    material: Literal["standard", "toon3", "toon5"] = "standard"
    flat_shading: bool = False
    side: Literal["front", "back", "double"] = "front"
    cast_shadow: bool = True
    receive_shadow: bool | float = True


if __name__ == "__main__":
    sphere = Sphere()
    print(sphere)
