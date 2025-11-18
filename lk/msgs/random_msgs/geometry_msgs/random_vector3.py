#!/usr/bin/env python3

"""
    RandomVector3 - Random version of geometry_msgs/Vector3.
"""

# BAM
from ..random_type import RandomType
from ..random_float import RandomFloat
from bam.msgs.ros_msgs import Vector3

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class RandomVector3(RandomType):
    """Random Vector3 with x, y, z components."""
    
    x: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.0))
    y: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.0))
    z: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.0))
    
    def with_seed(self, seed: int) -> 'RandomVector3':
        """Set seed for all components (chainable)."""
        self.x.with_seed(seed)
        self.y.with_seed(seed + 1)
        self.z.with_seed(seed + 2)
        return self
    
    def sample(self) -> Vector3:
        """Sample a concrete Vector3."""
        return Vector3(
            x=self.x.sample(),
            y=self.y.sample(),
            z=self.z.sample()
        )
    
    def get_range(self) -> tuple:
        """Get range information for x, y, z."""
        return (
            self.x.get_range(),
            self.y.get_range(),
            self.z.get_range()
        )
    
    @classmethod
    def fixed(cls, x: float, y: float, z: float) -> 'RandomVector3':
        """Create fixed Vector3."""
        return cls(
            x=RandomFloat.fixed(x),
            y=RandomFloat.fixed(y),
            z=RandomFloat.fixed(z)
        )
    
    @classmethod
    def from_list(cls, xyz: list[float]) -> 'RandomVector3':
        """Create fixed Vector3 from list [x, y, z]."""
        return cls.fixed(xyz[0], xyz[1], xyz[2])
    
    @classmethod
    def uniform(cls, 
                lower: list[float], 
                upper: list[float]) -> 'RandomVector3':
        """Create with uniformly distributed x, y, z."""
        return cls(
            x=RandomFloat.uniform(lower[0], upper[0]),
            y=RandomFloat.uniform(lower[1], upper[1]),
            z=RandomFloat.uniform(lower[2], upper[2])
        )
    
    @classmethod
    def zero(cls) -> 'RandomVector3':
        """Create zero vector."""
        return cls.fixed(0.0, 0.0, 0.0)


if __name__ == '__main__':
    print("\n[TEST] RandomVector3")
    
    # Fixed
    rvec = RandomVector3.fixed(1.0, 2.0, 3.0)
    vec = rvec.sample()
    print(f"Fixed: {vec.to_list()}")
    
    # From list
    rvec = RandomVector3.from_list([0.5, 0.0, 0.1])
    vec = rvec.sample()
    print(f"From list: {vec.to_list()}")
    
    # Zero
    rvec = RandomVector3.zero()
    vec = rvec.sample()
    print(f"Zero: {vec.to_list()}")
    
    # Uniform
    rvec = RandomVector3.uniform([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).with_seed(42)
    print(f"\nUniform distribution:")
    for i in range(3):
        vec = rvec.sample()
        print(f"  Sample {i}: {vec.to_list()}")
    
    print("\n✓ RandomVector3 working correctly!")

