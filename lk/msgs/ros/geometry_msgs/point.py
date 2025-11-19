
# BAM
from ..ros_msg import RosMsg
import typing
if typing.TYPE_CHECKING:
    from .vector3 import Vector3

# PYTHON
from dataclasses import dataclass
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

