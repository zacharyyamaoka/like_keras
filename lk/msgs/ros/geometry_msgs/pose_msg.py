# BAM
from ..ros_msg import RosMsg
from .vector3 import Vector3
from .point import Point
from .quaternion import Quaternion
from tf_transformations import xyzrpy_offset

# PYTHON
import numpy as np
from dataclasses import dataclass, field
from typing import List, TYPE_CHECKING, Generic

if TYPE_CHECKING:
    from .transform import Transform

"""
This can work for transforms by adding property that goes from translation -> position and rotation -> orientation

Downsides, is perhaps not as clear with the classes...
- I can just add a thin wrapper though, and add the type hint.. ok make sense!

However for some of them it does make sense I think...
"""
@dataclass
class PoseMsg(RosMsg):


    # To unify both Transform and Pose we can use a generic getter/setter
    @property
    def quat(self) -> Quaternion:
        raise NotImplementedError("quat is not implemented for PoseMsg")

    @quat.setter
    def quat(self, value: Quaternion):
        raise NotImplementedError("quat is not implemented for PoseMsg")

    @property
    def pos(self) -> Point | Vector3:
        raise NotImplementedError("pos is not implemented for PoseMsg")

    @pos.setter
    def pos(self, value: Point | Vector3):
        raise NotImplementedError("pos is not implemented for PoseMsg")

    @classmethod
    def random(cls, xyz_lower=None, xyz_upper=None, rpy_lower=None, rpy_upper=None):
        klass = cls()
        klass.pos = type(klass.pos).random(xyz_lower, xyz_upper)
        klass.quat = Quaternion.random(rpy_lower, rpy_upper)
        return klass


    @classmethod
    def from_matrix(cls, matrix: np.ndarray):
        klass = cls()
        klass.pos = type(klass.pos).from_matrix(matrix)
        klass.quat = Quaternion.from_matrix(matrix)
        return klass

    def set_matrix(self, matrix: np.ndarray):
        self.pos = type(self.pos).from_matrix(matrix)
        self.quat = Quaternion.from_matrix(matrix)
        return self

    def to_matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.quat.to_matrix()[:3, :3]
        T[:3, 3] = self.pos.to_numpy()
        return T


    @classmethod
    def from_list(cls, list: list[float], euler=True):
        klass = cls()
        klass.pos = type(klass.pos).from_list(list[:3])
        klass.quat = Quaternion.from_list(list[3:], euler)
        return klass

    def set_list(self, list: list[float], euler=True):
        self.pos = type(self.pos).from_list(list[:3])
        self.quat = Quaternion.from_list(list[3:], euler)

    def to_list(self, euler=True) -> List[float]:
        return [*self.pos.to_list(), *self.quat.to_list(euler)]


    @classmethod
    def from_xyzrpy(cls, xyz: list[float], rpy: list[float]):
        return cls.from_list(list=list(xyz) + list(rpy), euler=True)

    def set_xyzrpy(self, xyz: list[float], rpy: list[float]):
        self.set_list(list=list(xyz) + list(rpy), euler=True)

    def to_xyzrpy(self) -> tuple[list[float], list[float]]:
        return self.pos.to_list(), self.quat.to_list(euler=True)

    def interpolate(self, target: 'PoseMsg', fraction: float, in_place=False):
        pos = self.pos.lerp(target.pos, fraction)
        quat = self.quat.slerp(target.quat, fraction)

        if in_place:
            self.pos = pos
            self.quat = quat
            return self

        klass = type(self)()
        klass.pos = pos
        klass.quat = quat
        return klass

    def difference(self, target: 'PoseMsg') -> tuple[float, float]:
        distance = self.pos.distance(target.pos)
        vec, theta = self.quat.axangle_diff(target.quat)
        return distance, theta

    def offset(self, xyz=[0.,0.,0.], rpy=[0.,0.,0.], local=False, in_place=False):
        matrix = xyzrpy_offset(self.to_matrix(), xyz, rpy, local=local)
        if in_place:
            return self.set_matrix(matrix)

        return type(self).from_matrix(matrix)

    def flip_z(self, in_place=False):
        return self.offset(xyz=[0.,0.,0.], rpy=[np.pi, 0., 0.], local=True, in_place=in_place)

    @property
    def xyz(self) -> np.ndarray:
        return self.pos.to_numpy()
    
    @xyz.setter
    def xyz(self, value: list | tuple | np.ndarray):
        self.pos = type(self.pos).from_list(value)

    @property
    def rpy(self) -> np.ndarray:
        return np.array(self.quat.to_list(euler=True, wxyz=False))
    
    @rpy.setter
    def rpy(self, value: list | tuple | np.ndarray):
        self.quat = Quaternion.from_list(value, euler=True)

    @property
    def wxyz(self) -> np.ndarray:
        return np.array(self.quat.to_list(euler=False, wxyz=True))

    @property
    def xyzw(self) -> np.ndarray:
        return np.array(self.quat.to_list(euler=False, wxyz=False))

    def xyzrpy_str(self) -> str:
        xyzrpy = self.to_list()
        return f"xyz: {xyzrpy[:3]}, rpy: {np.rad2deg(xyzrpy[3:])}"
    

if __name__ == "__main__":
    pass