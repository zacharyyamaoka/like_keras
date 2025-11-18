from dataclasses import dataclass

from .geometry import Geometry


@dataclass
class Sphere(Geometry):
    radius: float = 0.0

    def __post_init__(self):
        super().__post_init__()
        if self.radius <= 0:
            raise ValueError(f"Invalid radius: {self.radius}. Radius must be positive.")
