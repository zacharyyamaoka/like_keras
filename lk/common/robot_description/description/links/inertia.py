"""
    Principal inertia tensor components for a rigid body.
"""

# BAM

# PYTHON
from dataclasses import dataclass


@dataclass
class Inertia:
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0


if __name__ == "__main__":
    inertia = Inertia()
    print(inertia)
