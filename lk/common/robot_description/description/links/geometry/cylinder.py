from dataclasses import dataclass

from .geometry import Geometry


@dataclass
class Cylinder(Geometry):
    radius: float = 0.0
    height: float = 0.0

    def __post_init__(self):
        super().__post_init__()
        if self.radius <= 0:
            raise ValueError(f"Invalid radius: {self.radius}. Radius must be positive.")
        if self.height <= 0:
            raise ValueError(f"Invalid height: {self.height}. Height must be positive.")
