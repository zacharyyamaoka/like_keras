"""
    Joint initial state configuration.
"""

# BAM

# PYTHON
from dataclasses import dataclass


@dataclass
class PerJointState:
    position: float = 0.0


if __name__ == "__main__":
    initial_state = PerJointState()
    print(initial_state)

