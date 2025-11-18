# BAM
from .Marker import Marker

# PYTHON
from dataclasses import dataclass
from typing import Literal


@dataclass
class Cylinder(Marker):
    radius: float = 1.0
    height: float = 2.0
    resolution: int = 20
    split: int = 4
    wireframe: bool = False
    material: Literal['standard', 'toon3', 'toon5'] = 'standard'
    flat_shading: bool = False
    side: Literal['front', 'back', 'double'] = 'front'
    cast_shadow: bool = True
    receive_shadow: bool | float = True


if __name__ == "__main__":
    cylinder = Cylinder()
    print(cylinder)

