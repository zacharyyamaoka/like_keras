# BAM
from .point import Point
from .quaternion import Quaternion
from .vector3 import Vector3
from .pose_msg import PoseMsg

# PYTHON
import numpy as np
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from .transform import Transform

"""
For simplicity and lower dimensional rep. gym uses euler.
ROS internally uses quaternions

This class will internally store as quaternions but output like euler unless otherwise specified.
"""
@dataclass
class Pose(PoseMsg):
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)

    @property
    def pos(self) -> Point:
        return self.position

    @pos.setter
    def pos(self, value: Point):
        self.position = value

    @property
    def quat(self) -> Quaternion:
        return self.orientation

    @quat.setter
    def quat(self, value: Quaternion):
        self.orientation = value

    @classmethod
    def from_transform(cls, transform: 'Transform') -> 'Pose':
        return cls(position=transform.translation.to_point(), orientation=transform.rotation)

    def to_transform(self) -> 'Transform':
        from .transform import Transform
        return Transform(translation=self.position.to_vector3(), rotation=self.orientation)
    
    @dataclass
    class Dist:
        """Distribution specification for Pose (mirrors message structure)."""
        position: 'Point.Dist' = field(default_factory=Point.Dist)
        orientation: 'Quaternion.Dist' = field(default_factory=Quaternion.Dist.identity)
        _seed: Optional[int] = field(default=None, init=False, repr=False)
        
        def __post_init__(self):
            """Initialize RNG."""
            self.rng = np.random.default_rng(self._seed)
        
        def sample(self) -> 'Pose':
            """Sample a concrete Pose."""
            return Pose(
                position=self.position.sample(),
                orientation=self.orientation.sample()
            )
        
        def get_range(self) -> tuple[Optional[dict], Optional[dict]]:
            """Get bounds as nested dict."""
            pos_min, pos_max = self.position.get_range()
            ori_min, ori_max = self.orientation.get_range()
            
            min_dict = {'position': pos_min, 'orientation': ori_min} if pos_min and ori_min else None
            max_dict = {'position': pos_max, 'orientation': ori_max} if pos_max and ori_max else None
            return (min_dict, max_dict)
        
        def contains(self, pose: 'Pose') -> bool:
            """Check if pose is within bounds."""
            return (self.position.contains(pose.position) and
                    self.orientation.contains(pose.orientation))
        
        def seed(self, seed: Optional[int] = None) -> int:
            """Seed the RNG for all component distributions."""
            if seed is None:
                seed = np.random.randint(0, 2**31)
            self._seed = seed
            self.rng = np.random.default_rng(seed)
            # Seed component distributions
            self.position.seed(seed)
            self.orientation.seed(seed + 100)
            return seed
        
        def generate_dataset(self, n: int) -> list['Pose']:
            """Generate a dataset of n Pose samples."""
            return [self.sample() for _ in range(n)]
        
        @classmethod
        def uniform(cls, low: float, high: float) -> 'Pose.Dist':
            """Uniform distribution for position, identity for orientation."""
            return cls(
                position=Point.Dist.uniform(low, high),
                orientation=Quaternion.Dist.identity()
            )
        
        @classmethod
        def uniform_with_rotation(cls, pos_low: float, pos_high: float,
                                  rpy_lower: tuple[float, float, float] = (-np.pi, -np.pi, -np.pi),
                                  rpy_upper: tuple[float, float, float] = (np.pi, np.pi, np.pi)) -> 'Pose.Dist':
            """Uniform distribution for both position and orientation (via euler angles)."""
            return cls(
                position=Point.Dist.uniform(pos_low, pos_high),
                orientation=Quaternion.Dist.uniform_rpy(rpy_lower, rpy_upper)
            )


if __name__ == "__main__":
   
    pose = Pose.random()
    Pose.from_matrix()
    print(pose)
    # matrix = pose.to_matrix()|
    # pose_from_matrix = Pose.from_matrix(matrix)

    # print("Basic random pose:")
    # print(pose)
    # print(pose_from_matrix)

    # diff = pose.difference(pose_from_matrix)
    # print(f"Difference: {diff}\n")

    # # Test with limits
    # xyz_lower = [0.0, 0.0, 0.5]
    # xyz_upper = [1.0, 1.0, 1.5]
    # rpy_lower = [-0.1, -0.1, -0.1]
    # rpy_upper = [0.1, 0.1, 0.1]
    
    # pose_limited = Pose.random(xyz_lower=xyz_lower, xyz_upper=xyz_upper, rpy_lower=rpy_lower, rpy_upper=rpy_upper)
    # print(f"Limited random pose (xyz: {xyz_lower} to {xyz_upper}, rpy: {rpy_lower} to {rpy_upper}):")
    # print(f"  xyz: {pose_limited.xyz}")
    # print(f"  rpy: {pose_limited.rpy}")

