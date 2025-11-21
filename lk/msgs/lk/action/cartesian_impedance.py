"""
Cartesian impedance control parameters.

Based on ros2_controllers cartesian_impedance_controller configuration.
Specifies stiffness and damping for compliant motion control.

https://utiasdsl.github.io/crisp_controllers/
"""

# PYTHON
from dataclasses import dataclass


@dataclass
class CartesianImpedance:
    """Impedance parameters in Cartesian space."""

    # Position stiffness (N/m) for each axis
    k_pos_x: float = 500.0
    k_pos_y: float = 500.0
    k_pos_z: float = 500.0

    # Position damping (Ns/m) - if < 0, computed as 2*sqrt(stiffness)
    d_pos_x: float = -1.0
    d_pos_y: float = -1.0
    d_pos_z: float = -1.0

    # Orientation stiffness (Nm/rad) for each axis
    k_rot_x: float = 30.0
    k_rot_y: float = 30.0
    k_rot_z: float = 30.0

    # Orientation damping (Nms/rad) - if < 0, computed as 2*sqrt(stiffness)
    d_rot_x: float = -1.0
    d_rot_y: float = -1.0
    d_rot_z: float = -1.0

    # Error clipping limits (m, rad)
    error_clip_x: float = 0.1  # m
    error_clip_y: float = 0.1  # m
    error_clip_z: float = 0.1  # m
    error_clip_rx: float = 0.5  # rad
    error_clip_ry: float = 0.5  # rad
    error_clip_rz: float = 0.5  # rad

    # Nullspace control (optional, for redundant manipulators)
    nullspace_stiffness: float = 1.0
    nullspace_damping: float = -1.0  # if < 0, computed as 2*sqrt(stiffness)

    def get_computed_damping_pos(self, axis: str) -> float:
        """Get position damping, computing from stiffness if not explicitly set."""
        k = getattr(self, f"k_pos_{axis}")
        d = getattr(self, f"d_pos_{axis}")
        return d if d >= 0 else 2.0 * (k**0.5)

    def get_computed_damping_rot(self, axis: str) -> float:
        """Get rotation damping, computing from stiffness if not explicitly set."""
        k = getattr(self, f"k_rot_{axis}")
        d = getattr(self, f"d_rot_{axis}")
        return d if d >= 0 else 2.0 * (k**0.5)


if __name__ == "__main__":
    impedance = CartesianImpedance()
    print("CartesianImpedance:")
    for key, value in vars(impedance).items():
        print(f"  {key:30}: {value}")

    print("\nComputed damping values:")
    for axis in ["x", "y", "z"]:
        print(
            f"  d_pos_{axis} (computed): {impedance.get_computed_damping_pos(axis):.2f}"
        )
        print(
            f"  d_rot_{axis} (computed): {impedance.get_computed_damping_rot(axis):.2f}"
        )
