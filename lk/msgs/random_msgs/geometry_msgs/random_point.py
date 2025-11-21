#!/usr/bin/env python3

"""
RandomPoint - Random version of geometry_msgs/Point.

Has the exact same structure as Point but with RandomFloat fields.
"""

# BAM
from ..random_type import RandomType
from ..random_float import RandomFloat
from bam.msgs.ros_msgs import Point

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class RandomPoint(RandomType):
    """Random Point with x, y, z as RandomFloat."""

    x: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.0))
    y: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.0))
    z: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.0))

    def with_seed(self, seed: int) -> "RandomPoint":
        """Set seed for all components (chainable)."""
        self.x.with_seed(seed)
        self.y.with_seed(seed)
        self.z.with_seed(seed)
        return self

    def sample(self) -> Point:
        """Sample a concrete Point."""
        return Point(x=self.x.sample(), y=self.y.sample(), z=self.z.sample())

    def get_range(self) -> tuple:
        """Get range information for each component."""
        return (self.x.get_range(), self.y.get_range(), self.z.get_range())

    @classmethod
    def fixed(cls, x: float, y: float, z: float) -> "RandomPoint":
        """Create fixed Point (all values fixed)."""
        return cls(
            x=RandomFloat.fixed(x), y=RandomFloat.fixed(y), z=RandomFloat.fixed(z)
        )

    @classmethod
    def from_list(cls, values: list[RandomFloat]) -> "RandomPoint":
        """Create from list of RandomFloat [x, y, z]."""
        return cls(x=values[0], y=values[1], z=values[2])

    @classmethod
    def from_floats(cls, x: float, y: float, z: float) -> "RandomPoint":
        """Create from fixed float values."""
        return cls.fixed(x, y, z)

    @classmethod
    def uniform(cls, xyz_lower: list[float], xyz_upper: list[float]) -> "RandomPoint":
        """Create with uniformly distributed xyz."""
        return cls(
            x=RandomFloat.uniform(xyz_lower[0], xyz_upper[0]),
            y=RandomFloat.uniform(xyz_lower[1], xyz_upper[1]),
            z=RandomFloat.uniform(xyz_lower[2], xyz_upper[2]),
        )


if __name__ == "__main__":
    print("\n[TEST] RandomPoint - Chainable .with_seed()")

    # Fixed point
    rp_fixed = RandomPoint.fixed(1.0, 2.0, 3.0)
    print(f"Fixed: {isinstance(rp_fixed, RandomType)}")
    point = rp_fixed.sample()
    print(f"  Sampled: {point.to_list()}")

    # Uniform point with seed (chainable)
    rp_uniform = RandomPoint.uniform([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).with_seed(42)
    print(f"\nUniform (with seed): {isinstance(rp_uniform, RandomType)}")
    for i in range(3):
        point = rp_uniform.sample()
        print(f"  Sample {i}: {point.to_list()}")

    # from_list with mixed RandomFloat (chainable on individual components)
    rp_mixed = RandomPoint.from_list(
        [
            RandomFloat.fixed(0.5),
            RandomFloat.uniform(0.0, 1.0).with_seed(42),
            RandomFloat.fixed(0.0),
        ]
    )
    print(f"\nMixed (x=fixed, y=random, z=fixed): {isinstance(rp_mixed, RandomType)}")
    for i in range(3):
        point = rp_mixed.sample()
        print(f"  Sample {i}: x={point.x:.3f}, y={point.y:.3f}, z={point.z:.3f}")

    # Chaining demo
    print(f"\nChaining demo:")
    rp = RandomPoint.uniform([0, 0, 0], [1, 1, 1]).with_seed(999)
    print(f"  Created with fluent interface: {rp.sample().to_list()}")
