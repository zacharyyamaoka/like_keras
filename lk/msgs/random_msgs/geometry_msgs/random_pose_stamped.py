#!/usr/bin/env python3

"""
    RandomPoseStamped - Random version of geometry_msgs/PoseStamped.
"""

# BAM
from ..random_type import RandomType
from .random_pose import RandomPose
from bam.msgs.ros_msgs import PoseStamped, Header

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class RandomPoseStamped(RandomType):
    """Random PoseStamped with frame_id and RandomPose.
    
    Mirrors the structure of geometry_msgs/PoseStamped.
    """
    
    frame_id: str = "world"
    pose: RandomPose = field(default_factory=RandomPose.identity)
    
    def with_seed(self, seed: int) -> 'RandomPoseStamped':
        """Set seed for pose component (chainable)."""
        self.pose.with_seed(seed)
        return self
    
    def sample(self) -> PoseStamped:
        """Sample a concrete PoseStamped."""
        sampled_pose = self.pose.sample()
        return PoseStamped(
            header=Header(frame_id=self.frame_id),
            pose=sampled_pose
        )
    
    def get_range(self) -> tuple:
        """Get range information for pose."""
        return self.pose.get_range()
    
    @classmethod
    def identity(cls, frame_id: str = "world") -> 'RandomPoseStamped':
        """Create identity pose (origin, no rotation)."""
        return cls(
            frame_id=frame_id,
            pose=RandomPose.identity()
        )
    
    @classmethod
    def fixed(cls, xyz: list[float], rpy: list[float], frame_id: str = "world") -> 'RandomPoseStamped':
        """Create fixed PoseStamped from xyz and rpy."""
        return cls(
            frame_id=frame_id,
            pose=RandomPose.fixed(xyz, rpy)
        )
    
    @classmethod
    def from_xyzrpy(cls, xyz: list[float], rpy: list[float], frame_id: str = "world") -> 'RandomPoseStamped':
        """Create fixed PoseStamped from xyz and rpy (alias for fixed)."""
        return cls.fixed(xyz, rpy, frame_id)
    
    @classmethod
    def uniform(cls, 
                xyz_lower: list[float], xyz_upper: list[float],
                rpy_lower: list[float], rpy_upper: list[float],
                frame_id: str = "world") -> 'RandomPoseStamped':
        """Create with uniformly distributed xyz and rpy."""
        return cls(
            frame_id=frame_id,
            pose=RandomPose.uniform(xyz_lower, xyz_upper, rpy_lower, rpy_upper)
        )


if __name__ == '__main__':
    import numpy as np
    
    print("\n[TEST] RandomPoseStamped")
    
    # Identity
    rpose_stamped = RandomPoseStamped.identity(frame_id="conveyor")
    pose_stamped = rpose_stamped.sample()
    print(f"Identity in 'conveyor' frame:")
    print(f"  frame_id: {pose_stamped.header.frame_id}")
    print(f"  xyz: {pose_stamped.xyz}, rpy: {np.rad2deg(pose_stamped.rpy)}")
    
    # Fixed
    rpose_stamped = RandomPoseStamped.fixed([0.5, 0.3, 0.1], [0.0, 0.0, 1.57], frame_id="world")
    pose_stamped = rpose_stamped.sample()
    print(f"\nFixed pose:")
    print(f"  frame_id: {pose_stamped.header.frame_id}")
    print(f"  xyz: {pose_stamped.xyz}, rpy: {np.rad2deg(pose_stamped.rpy)}")
    
    # Uniform
    rpose_stamped = RandomPoseStamped.uniform(
        xyz_lower=[0.0, 0.0, 0.0],
        xyz_upper=[1.0, 0.5, 0.2],
        rpy_lower=[0.0, 0.0, -3.14],
        rpy_upper=[0.0, 0.0, 3.14],
        frame_id="conveyor"
    ).with_seed(42)
    
    print(f"\nUniform distribution:")
    for i in range(3):
        pose_stamped = rpose_stamped.sample()
        print(f"  Sample {i}: xyz={pose_stamped.xyz}, rpy(deg)={np.rad2deg(pose_stamped.rpy)}")
    
    print("\n✓ RandomPoseStamped working correctly!")

