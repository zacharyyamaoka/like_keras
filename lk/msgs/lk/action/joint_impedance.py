"""
Joint space impedance control parameters.

Used for dynamically adjusting stiffness and damping in joint-space control.
Each joint can have its own kp_scale and kd_scale values.
Typically used with Moteus controllers to scale PD gains.
"""

# PYTHON
from dataclasses import dataclass, field


@dataclass
class JointImpedance:
    """Impedance parameters in joint space.

    Each list contains per-joint scaling factors. If empty, defaults to 1.0 for all joints.
    """

    kp_scale: list[float] = field(
        default_factory=list
    )  # per-joint position stiffness (kp) scaling factors
    kd_scale: list[float] = field(
        default_factory=list
    )  # per-joint damping (kd) scaling factors


if __name__ == "__main__":
    # Example 1: Default (empty lists)
    impedance1 = JointImpedance()
    print("JointImpedance (default):")
    for key, value in vars(impedance1).items():
        print(f"  {key:30}: {value}")

    # Example 2: 6-joint robot with different impedances
    impedance2 = JointImpedance(
        kp_scale=[1.0, 0.8, 0.9, 1.0, 0.7, 0.6],
        kd_scale=[1.0, 0.9, 0.95, 1.0, 0.8, 0.7],
    )
    print("\nJointImpedance (6 joints):")
    for key, value in vars(impedance2).items():
        print(f"  {key:30}: {value}")
