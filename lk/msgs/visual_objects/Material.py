# BAM

from .RGBA import RGBA

# PYTHON
from dataclasses import dataclass, field
import numpy as np

@dataclass
class Material:
    color: RGBA = field(default_factory=lambda: RGBA.grey())
    metallic: float = 0.0
    roughness: float = 1.0

    @classmethod
    def random(cls) -> 'Material':
        return cls(color=RGBA.random(), metallic=np.random.uniform(0., 1.), roughness=np.random.uniform(0., 1.))



