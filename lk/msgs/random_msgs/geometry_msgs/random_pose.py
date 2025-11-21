#!/usr/bin/env python3

"""
RandomPose - Random version of geometry_msgs/Pose.

Has the exact same structure as Pose: position (Point) and orientation (Quaternion).
But uses RandomPoint and RandomQuaternion instead.
"""

# BAM
from ..random_type import RandomType
from .random_point import RandomPoint
from .random_quaternion import RandomQuaternion
from bam.msgs.ros_msgs import Pose

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class RandomPose(RandomType):
    """Random Pose with position (RandomPoint) and orientation (RandomQuaternion).

    Mirrors the exact structure of geometry_msgs/Pose.
    """

    position: RandomPoint = field(default_factory=RandomPoint)
    orientation: RandomQuaternion = field(default_factory=RandomQuaternion.identity)

    def with_seed(self, seed: int) -> "RandomPose":
        """Set seed for all components (chainable)."""
        self.position.with_seed(seed)
        self.orientation.with_seed(seed)
        return self

    def sample(self) -> Pose:
        """Sample a concrete Pose."""
        return Pose(
            position=self.position.sample(), orientation=self.orientation.sample()
        )

    def get_range(self) -> tuple:
        """Get range information for position and orientation."""
        return (self.position.get_range(), self.orientation.get_range())

    @classmethod
    def identity(cls) -> "RandomPose":
        """Create identity pose (origin, no rotation)."""
        return cls(
            position=RandomPoint.fixed(0.0, 0.0, 0.0),
            orientation=RandomQuaternion.identity(),
        )

    @classmethod
    def fixed(cls, xyz: list[float], rpy: list[float]) -> "RandomPose":
        """Create fixed Pose from xyz and rpy."""
        return cls(
            position=RandomPoint.fixed(*xyz),
            orientation=RandomQuaternion.uniform_euler(rpy, rpy),
        )

    @classmethod
    def from_xyzrpy(cls, xyz: list[float], rpy: list[float]) -> "RandomPose":
        """Create fixed Pose from xyz and rpy (alias for fixed)."""
        return cls.fixed(xyz, rpy)

    @classmethod
    def uniform(
        cls,
        xyz_lower: list[float],
        xyz_upper: list[float],
        rpy_lower: list[float],
        rpy_upper: list[float],
    ) -> "RandomPose":
        """Create with uniformly distributed xyz and rpy."""
        return cls(
            position=RandomPoint.uniform(xyz_lower, xyz_upper),
            orientation=RandomQuaternion.uniform_euler(rpy_lower, rpy_upper),
        )

    @classmethod
    def from_components(
        cls, position: RandomPoint, orientation: RandomQuaternion
    ) -> "RandomPose":
        """Create from RandomPoint and RandomQuaternion components."""
        return cls(position=position, orientation=orientation)


if __name__ == "__main__":
    import numpy as np

    print("\n[TEST] RandomPose - Chainable .with_seed()")

    # Identity
    rpose_identity = RandomPose.identity()
    print(f"Identity: {isinstance(rpose_identity, RandomType)}")
    pose = rpose_identity.sample()
    print(f"  xyz: {pose.xyz}, rpy: {np.rad2deg(pose.rpy)}")

    # Fixed
    rpose_fixed = RandomPose.fixed([0.5, 0.0, 0.1], [0.0, 0.0, 1.57])
    print(f"\nFixed: {isinstance(rpose_fixed, RandomType)}")
    pose = rpose_fixed.sample()
    print(f"  xyz: {pose.xyz}, rpy: {np.rad2deg(pose.rpy)}")

    # Uniform with chainable seed
    rpose_uniform = RandomPose.uniform(
        xyz_lower=[0.0, 0.0, 0.0],
        xyz_upper=[1.0, 1.0, 1.0],
        rpy_lower=[-0.1, -0.1, -0.1],
        rpy_upper=[0.1, 0.1, 0.1],
    ).with_seed(42)

    print(f"\nUniform (with seed): {isinstance(rpose_uniform, RandomType)}")
    for i in range(3):
        pose = rpose_uniform.sample()
        print(f"  Sample {i}:")
        print(f"    xyz: {pose.xyz}")
        print(f"    rpy (deg): {np.rad2deg(pose.rpy)}")

    # Compositional building with chainable seeds
    from ..random_float import RandomFloat

    rpose_custom = RandomPose.from_components(
        position=RandomPoint.uniform([0, 0, 0], [1, 1, 1]).with_seed(42),
        orientation=RandomQuaternion.identity(),
    )
    print(f"\nCompositional: {isinstance(rpose_custom, RandomType)}")
    pose = rpose_custom.sample()
    print(f"  xyz (random): {pose.xyz}")
    print(f"  rpy (identity): {np.rad2deg(pose.rpy)}")

    print("\n" + "=" * 70)
    print("Clean API: .with_seed() is chainable everywhere!")
    print("=" * 70)
