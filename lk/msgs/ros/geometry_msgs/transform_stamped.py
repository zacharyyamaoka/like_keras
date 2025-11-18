
# BAM
from .quaternion import Quaternion
from .vector3 import Vector3
from .pose_msg import PoseMsg
from ..std_msgs.header import Header
from .transform import Transform
from .pose_stamped import PoseStamped
from tf_transformations import xyzrpy_offset

# PYTHON
import numpy as np
from typing import List, Optional, TYPE_CHECKING
from dataclasses import dataclass, field
import copy

if TYPE_CHECKING:
    from .pose_stamped import PoseStamped

"""
https://docs.ros2.org/foxy/api/geometry_msgs/msg/TransformStamped.html
"""

@dataclass
class TransformStamped(PoseMsg):
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    transform: Transform = field(default_factory=Transform)

    @property
    def pos(self) -> Vector3:
        return self.transform.translation

    @pos.setter
    def pos(self, value: Vector3):
        self.transform.translation = value

    @property
    def quat(self) -> Quaternion:
        return self.transform.rotation

    @quat.setter
    def quat(self, value: Quaternion):
        self.transform.rotation = value
            

    def set_header(self, frame_id='', child_frame_id='', header: Header = None):
        if header is None:
            header = Header(frame_id=frame_id)
        else:
            header = copy.deepcopy(header) # avoid bugs, often you pass in an object

        self.header = header
        self.child_frame_id = child_frame_id

        return self # helpful for compact chaining

    @classmethod
    def random(cls, xyz_lower=None, xyz_upper=None, rpy_lower=None, rpy_upper=None, frame_id='', child_frame_id='') -> 'TransformStamped':
        return super().random(xyz_lower, xyz_upper, rpy_lower, rpy_upper).set_header(frame_id=frame_id, child_frame_id=child_frame_id)

    @classmethod
    def from_frames(cls, frame_id: str, child_frame_id: str, header: Header = None) -> 'TransformStamped':
        return cls().set_header(frame_id=frame_id, child_frame_id=child_frame_id, header=header)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray, frame_id='', child_frame_id='', header: Header = None):
        return super().from_matrix(matrix).set_header(frame_id=frame_id, child_frame_id=child_frame_id, header=header)

    @classmethod
    def from_pose_stamped(cls, pose_stamped: 'PoseStamped', child_frame_id='') -> 'TransformStamped':
        return cls(header=pose_stamped.header, transform=pose_stamped.pose.to_transform(), child_frame_id=child_frame_id)

    def to_pose_stamped(self) -> 'PoseStamped':
        from .pose_stamped import PoseStamped
        return PoseStamped(header=self.header, pose=self.transform.to_pose())

    @classmethod
    def from_list(cls, list: List[float], euler=True, frame_id='', child_frame_id='', header: Header = None) -> 'TransformStamped':
        return super().from_list(list, euler).set_header(frame_id=frame_id, child_frame_id=child_frame_id, header=header)

    @classmethod
    def from_xyzrpy(cls, xyz: list[float], rpy: list[float], frame_id='', child_frame_id='', header: Header = None) -> 'TransformStamped':
        return cls.from_list(list=list(xyz) + list(rpy), euler=True, frame_id=frame_id, child_frame_id=child_frame_id, header=header)


    # Its important to return a new object, to avoid mutating the original object
    def interpolate(self, target: 'TransformStamped', fraction: float, in_place=False) -> 'TransformStamped':

        tf = super().interpolate(target, fraction, in_place) 

        if fraction < 0.5:
            tf.child_frame_id = self.child_frame_id
        else:
            tf.child_frame_id = target.child_frame_id

        return tf


    def offset(self, xyz=[0.,0.,0.], rpy=[0.,0.,0.], local=False, in_place=False, child_frame_id: Optional[str] = None) -> 'TransformStamped':
        tf = super().offset(xyz, rpy, local, in_place)
        if child_frame_id is not None:
            tf.child_frame_id = child_frame_id
        return tf

    def __matmul__(self, other: 'TransformStamped') -> np.ndarray:
        """Dot product (matrix multiplication) using the @ operator.
        Multiplies the underlying 4x4 transformation matrices.

        Returns:
            np.ndarray: The resulting 4x4 transformation matrix.
        """
        assert self.child_frame_id == other.header.frame_id, (
            f"child_frame_id and frame_id mismatch:\n"
            f"    T_{self.header.frame_id}_to_{self.child_frame_id} @ T_{other.header.frame_id}_to_{other.child_frame_id}\n"
        )

        T = self.to_matrix() @ other.to_matrix()
        
        return TransformStamped.from_matrix(T, header=self.header, child_frame_id=other.child_frame_id)

    def chain_multiply(self, *tfs: 'TransformStamped') -> 'TransformStamped':
        """ Chain together multiple __matmul__ operations."""
        
        if len(tfs) < 2:
            raise ValueError("At least two TransformStamped objects are required")

        T = self
        for tf in tfs:
            T = T @ tf
        
        return T






    
    # def __str__(self):
    #     return str(asdict(self)) # flat is prefered for now


if __name__ == "__main__":
    transform_stamped = TransformStamped.random()
    transform_stamped_2 = TransformStamped.random()

    print(transform_stamped)

    transform_stamped_3 = transform_stamped.interpolate(transform_stamped_2, 0.5)
    print(transform_stamped_2)

    diff = transform_stamped.difference(transform_stamped_2)
    print(diff)
