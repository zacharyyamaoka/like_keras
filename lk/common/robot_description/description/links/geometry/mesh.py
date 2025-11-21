from dataclasses import dataclass

from .geometry import Geometry


@dataclass
class Mesh(Geometry):
    filename: str = ""
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)

    def __post_init__(self):
        super().__post_init__()
        if any(s <= 0 for s in self.scale):
            raise ValueError(
                f"Invalid scale: {self.scale}. All scale values must be positive."
            )
