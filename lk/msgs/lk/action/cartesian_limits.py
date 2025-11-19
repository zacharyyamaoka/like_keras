"""
    Cartesian space kinematic limits for trajectory planning.
    
    Hardware-agnostic specification of motion constraints in task space.

    Reminds me of Pilz: https://moveit.picknik.ai/humble/doc/examples/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html
"""

# PYTHON
from dataclasses import dataclass


@dataclass
class CartesianLimits:
    """Kinematic limits in Cartesian space."""
    
    # Linear motion limits (m/s, m/s², m/s³)
    max_linear_velocity: float = 1.0       # m/s
    max_linear_acceleration: float = 1.0   # m/s²
    max_linear_jerk: float = 10.0          # m/s³
    
    # Angular motion limits (rad/s, rad/s², rad/s³)
    max_angular_velocity: float = 2.0      # rad/s
    max_angular_acceleration: float = 2.0  # rad/s²
    max_angular_jerk: float = 20.0         # rad/s³
    
    # Force/torque limits (N, Nm)
    max_linear_force: float | None = None   # N (optional)
    max_angular_torque: float | None = None # Nm (optional)


if __name__ == "__main__":
    limits = CartesianLimits()
    print("CartesianLimits:")
    for key, value in vars(limits).items():
        print(f"  {key:30}: {value}")

