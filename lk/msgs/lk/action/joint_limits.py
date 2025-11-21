"""
Generic kinematic limits for trajectory planning.

Can be instantiated separately for arm, hand, or any actuator.
Holds per-joint velocity, acceleration, and jerk limits.
"""

# PYTHON
from dataclasses import dataclass


@dataclass
class JointLimits:
    """Generic kinematic limits (velocity, acceleration, jerk) for any actuator."""

    max_velocity: list[float] | None = None
    max_acceleration: list[float] | None = None
    max_jerk: list[float] | None = None

    min_velocity: list[float] | None = None
    min_acceleration: list[float] | None = None
    min_jerk: list[float] | None = None

    def extend(self, other: "JointLimits") -> "JointLimits":
        """
        Extend limits by appending another JointLimits instance.

        Useful for combining arm + hand limits into a single structure.
        Returns a new JointLimits instance with concatenated arrays.

        Safety: Both instances must have the same fields defined. If one has a field
        and the other doesn't, an error is raised to prevent misalignment.

        DescriptionArgs:
            other: Another JointLimits instance to append

        Returns:
            New JointLimits instance with combined limits

        Raises:
            ValueError: If one instance has a field and the other doesn't
        """

        def _concat(
            a: list[float] | None, b: list[float] | None, field_name: str
        ) -> list[float] | None:
            # Both None - skip this field
            if a is None and b is None:
                return None

            # Only one is defined - error to prevent misalignment
            if a is None or b is None:
                raise ValueError(
                    f"Cannot extend JointLimits: '{field_name}' is defined in only one instance. "
                    f"Both must be None or both must be defined to prevent misalignment."
                )

            # Both defined - concatenate
            return a + b

        return JointLimits(
            max_velocity=_concat(self.max_velocity, other.max_velocity, "max_velocity"),
            max_acceleration=_concat(
                self.max_acceleration, other.max_acceleration, "max_acceleration"
            ),
            max_jerk=_concat(self.max_jerk, other.max_jerk, "max_jerk"),
            min_velocity=_concat(self.min_velocity, other.min_velocity, "min_velocity"),
            min_acceleration=_concat(
                self.min_acceleration, other.min_acceleration, "min_acceleration"
            ),
            min_jerk=_concat(self.min_jerk, other.min_jerk, "min_jerk"),
        )


if __name__ == "__main__":
    # Example: 6-DOF arm
    arm_limits = JointLimits(
        max_velocity=[1.0, 1.0, 1.5, 1.5, 2.0, 2.0],
        max_acceleration=[2.0, 2.0, 2.5, 2.5, 3.0, 3.0],
        max_jerk=[10.0] * 6,
        min_velocity=[-1.0] * 6,
        min_acceleration=[-2.0] * 6,
    )

    # Example: 1-DOF gripper
    hand_limits = JointLimits(
        max_velocity=[0.5], max_acceleration=[1.0], max_jerk=[5.0]
    )

    print("Arm Limits (6-DOF):")
    for key, value in vars(arm_limits).items():
        print(f"  {key:30}: {value}")

    print("\nHand Limits (1-DOF):")
    for key, value in vars(hand_limits).items():
        print(f"  {key:30}: {value}")

    # Extend arm with hand limits
    combined_limits = arm_limits.extend(hand_limits)

    print("\nCombined Limits (7-DOF = 6 arm + 1 hand):")
    for key, value in vars(combined_limits).items():
        print(f"  {key:30}: {value}")
