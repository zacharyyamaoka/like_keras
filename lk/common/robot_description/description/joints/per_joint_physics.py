"""
    Joint physics parameters for dynamics simulation.
"""

# BAM

# PYTHON
from dataclasses import dataclass


@dataclass
class PerJointPhysics:
    friction: float = 0.0
    damping: float = 0.0


if __name__ == "__main__":
    physics = PerJointPhysics()
    print(physics)

