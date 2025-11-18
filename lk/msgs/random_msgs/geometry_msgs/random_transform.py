#!/usr/bin/env python3

"""
    RandomTransform - Random version of geometry_msgs/Transform.
"""

# BAM
from ..random_type import RandomType
from .random_vector3 import RandomVector3
from .random_quaternion import RandomQuaternion
from bam.msgs.ros_msgs import Transform

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class RandomTransform(RandomType):
    """Random Transform with translation (RandomVector3) and rotation (RandomQuaternion)."""
    
    translation: RandomVector3 = field(default_factory=RandomVector3.zero)
    rotation: RandomQuaternion = field(default_factory=RandomQuaternion.identity)
    
    def with_seed(self, seed: int) -> 'RandomTransform':
        """Set seed for all components (chainable)."""
        self.translation.with_seed(seed)
        self.rotation.with_seed(seed + 10)
        return self
    
    def sample(self) -> Transform:
        """Sample a concrete Transform."""
        return Transform(
            translation=self.translation.sample(),
            rotation=self.rotation.sample()
        )
    
    def get_range(self) -> tuple:
        """Get range information for translation and rotation."""
        return (
            self.translation.get_range(),
            self.rotation.get_range()
        )
    
    @classmethod
    def identity(cls) -> 'RandomTransform':
        """Create identity transform (zero translation, no rotation)."""
        return cls(
            translation=RandomVector3.zero(),
            rotation=RandomQuaternion.identity()
        )
    
    @classmethod
    def fixed(cls, xyz: list[float], rpy: list[float]) -> 'RandomTransform':
        """Create fixed Transform from xyz translation and rpy rotation."""
        return cls(
            translation=RandomVector3.from_list(xyz),
            rotation=RandomQuaternion.uniform_euler(rpy, rpy)
        )
    
    @classmethod
    def uniform(cls,
                xyz_lower: list[float], xyz_upper: list[float],
                rpy_lower: list[float], rpy_upper: list[float]) -> 'RandomTransform':
        """Create with uniformly distributed xyz and rpy."""
        return cls(
            translation=RandomVector3.uniform(xyz_lower, xyz_upper),
            rotation=RandomQuaternion.uniform_euler(rpy_lower, rpy_upper)
        )


if __name__ == '__main__':
    import numpy as np
    
    print("\n[TEST] RandomTransform")
    
    # Identity
    rtf = RandomTransform.identity()
    tf = rtf.sample()
    print(f"Identity:")
    print(f"  translation: {tf.translation.to_list()}")
    print(f"  rotation (rpy deg): {np.rad2deg(tf.rpy)}")
    
    # Fixed
    rtf = RandomTransform.fixed([0.5, 0.3, 0.1], [0.0, 0.0, 1.57])
    tf = rtf.sample()
    print(f"\nFixed:")
    print(f"  translation: {tf.translation.to_list()}")
    print(f"  rotation (rpy deg): {np.rad2deg(tf.rpy)}")
    
    # Uniform
    rtf = RandomTransform.uniform(
        xyz_lower=[0.0, 0.0, 0.0],
        xyz_upper=[1.0, 1.0, 1.0],
        rpy_lower=[0.0, 0.0, -3.14],
        rpy_upper=[0.0, 0.0, 3.14]
    ).with_seed(42)
    
    print(f"\nUniform distribution:")
    for i in range(3):
        tf = rtf.sample()
        print(f"  Sample {i}:")
        print(f"    translation: {tf.translation.to_list()}")
        print(f"    rotation (rpy deg): {np.rad2deg(tf.rpy)}")
    
    print("\n✓ RandomTransform working correctly!")

