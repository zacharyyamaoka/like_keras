
# BAM
from ..ros_msg import RosMsg
import typing
if typing.TYPE_CHECKING:
    from .vector3 import Vector3

# PYTHON
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

# https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html

@dataclass
class Point(RosMsg):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    @classmethod
    def random(cls, xyz_lower=None, xyz_upper=None) -> 'Point':
        if xyz_lower is None:
            xyz_lower = [-1, -1, -1]
        if xyz_upper is None:
            xyz_upper = [1, 1, 1]
        return cls(
            x=np.random.uniform(xyz_lower[0], xyz_upper[0]),
            y=np.random.uniform(xyz_lower[1], xyz_upper[1]),
            z=np.random.uniform(xyz_lower[2], xyz_upper[2])
        )


    @classmethod
    def from_list(cls, list: list[float]) -> 'Point':
        return cls(x=list[0], y=list[1], z=list[2])

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    @property
    def xyz(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    @xyz.setter
    def xyz(self, value: list | tuple | np.ndarray):
        self.x, self.y, self.z = float(value[0]), float(value[1]), float(value[2])

    def to_vector3(self) -> 'Vector3':
        from .vector3 import Vector3
        return Vector3(x=self.x, y=self.y, z=self.z)

    @classmethod
    def from_xyz(cls, xyz: tuple|list) -> 'Point':
        return cls.from_list(list(xyz))

    @classmethod
    def from_matrix(cls, matrix: np.ndarray):
        return cls(x=matrix[0, 3], y=matrix[1, 3], z=matrix[2, 3])

    def to_matrix(self) -> np.ndarray:
        matrix = np.eye(4)
        matrix[:3, 3] = self.to_list()
        return matrix

    def lerp(self, target: 'Point', fraction: float) -> 'Point':
        return Point(x=self.x + (target.x - self.x) * fraction,
                     y=self.y + (target.y - self.y) * fraction,
                     z=self.z + (target.z - self.z) * fraction)

    def distance(self, target: 'Point') -> float:
        return np.linalg.norm(np.array(self.to_list()) - np.array(target.to_list()))
    
    @dataclass
    class Dist:
        """Distribution specification for Point with gymnasium Space compatibility."""
        from lk.msgs.random_msgs.float_dist import FloatDist
        from lk.msgs.random_msgs.data_dist import DataDist
        
        x: 'FloatDist' = field(default_factory=lambda: FloatDist.fixed(0.0))
        y: 'FloatDist' = field(default_factory=lambda: FloatDist.fixed(0.0))
        z: 'FloatDist' = field(default_factory=lambda: FloatDist.fixed(0.0))
        _seed: Optional[int] = field(default=None, init=False, repr=False)
        
        def __post_init__(self):
            """Initialize RNG."""
            self.rng = np.random.default_rng(self._seed)
        
        def sample(self) -> 'Point':
            """Sample a concrete Point."""
            return Point(
                x=self.x.sample(),
                y=self.y.sample(),
                z=self.z.sample()
            )
        
        def get_range(self) -> tuple[Optional[tuple], Optional[tuple]]:
            """Get (min, max) bounds as tuples."""
            x_min, x_max = self.x.get_range()
            y_min, y_max = self.y.get_range()
            z_min, z_max = self.z.get_range()
            
            min_pt = (x_min, y_min, z_min) if all(v is not None for v in [x_min, y_min, z_min]) else None
            max_pt = (x_max, y_max, z_max) if all(v is not None for v in [x_max, y_max, z_max]) else None
            return (min_pt, max_pt)
        
        def contains(self, point: 'Point') -> bool:
            """Check if point is within bounds."""
            return (self.x.contains(point.x) and 
                    self.y.contains(point.y) and 
                    self.z.contains(point.z))
        
        def seed(self, seed: Optional[int] = None) -> int:
            """Seed the RNG for all component distributions."""
            if seed is None:
                seed = np.random.randint(0, 2**31)
            self._seed = seed
            self.rng = np.random.default_rng(seed)
            # Seed component distributions
            self.x.seed(seed)
            self.y.seed(seed + 1)
            self.z.seed(seed + 2)
            return seed
        
        def generate_dataset(self, n: int) -> list['Point']:
            """Generate a dataset of n Point samples."""
            return [self.sample() for _ in range(n)]
        
        @classmethod
        def uniform(cls, low: float, high: float) -> 'Point.Dist':
            """Uniform distribution on all axes."""
            from lk.msgs.random_msgs.float_dist import FloatDist
            return cls(
                x=FloatDist.uniform(low, high),
                y=FloatDist.uniform(low, high),
                z=FloatDist.uniform(low, high)
            )
        
        @classmethod
        def normal(cls, mean: tuple[float, float, float] = (0.0, 0.0, 0.0), 
                  std: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> 'Point.Dist':
            """Normal distribution with per-axis mean and std."""
            from lk.msgs.random_msgs.float_dist import FloatDist
            return cls(
                x=FloatDist.normal(mean[0], std[0]),
                y=FloatDist.normal(mean[1], std[1]),
                z=FloatDist.normal(mean[2], std[2])
            )

    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
        self.z = float(self.z)

    # def to_dict(self):
    #     return {
    #         "x": self.x,
    #         "y": self.y,
    #         "z": self.z,
    #     }

    # @classmethod
    # def from_dict(cls, d: dict):
    #     return cls(
    #         x=float(d.get("x", 0.0)),
    #         y=float(d.get("y", 0.0)),
    #         z=float(d.get("z", 0.0)),
    #     )
    
    # @classmethod
    # def from_array(cls, array: np.ndarray | Tuple | List):
    #     return cls(*array)

