from dataclasses import dataclass

from .geometry import Geometry


@dataclass
class Box(Geometry):
    size: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self):
        super().__post_init__()
        if len(self.size) != 3:
            raise ValueError(f"Box size must have exactly 3 dimensions, got {len(self.size)}")
        if any(dim <= 0 for dim in self.size):
            raise ValueError(f"All box dimensions must be positive, got {self.size}")
    
    @property
    def length(self) -> float:
        return self.size[0]
    
    @property
    def width(self) -> float:
        return self.size[1]
    
    @property
    def height(self) -> float:
        return self.size[2]
