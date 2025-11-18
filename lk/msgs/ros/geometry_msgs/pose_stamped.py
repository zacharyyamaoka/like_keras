
# BAM
from .point import Point
from .quaternion import Quaternion
from ..ros_msg import RosMsg
from ..std_msgs.header import Header
from .pose_msg import PoseMsg
from .pose import Pose

from tf_transformations import xyzrpy_offset

# PYTHON
import copy
import numpy as np
from typing import List, TYPE_CHECKING, Optional
from dataclasses import dataclass, field

if TYPE_CHECKING:
    from .transform_stamped import TransformStamped


@dataclass
class PoseStamped(PoseMsg):
    header: Header = field(default_factory=Header)
    pose: Pose = field(default_factory=Pose)

    @property
    def pos(self) -> Point:
        return self.pose.position

    @pos.setter
    def pos(self, value: Point):
        self.pose.position = value

    @property
    def quat(self) -> Quaternion:
        return self.pose.orientation

    @quat.setter
    def quat(self, value: Quaternion):
        self.pose.orientation = value
        
    def set_header(self, frame_id='', header: Header = None):
        if header is None:
            header = Header(frame_id=frame_id)
        else:
            header = copy.deepcopy(header) # avoid bugs, often you pass in an object

        self.header = header

        return self # helpful for compact chaining

    @classmethod
    def from_frame(cls, frame_id: str) -> 'PoseStamped':
        return cls(header=Header(frame_id=frame_id))
        
    @classmethod
    def random(cls, xyz_lower=None, xyz_upper=None, rpy_lower=None, rpy_upper=None, frame_id='') -> 'PoseStamped':
        return super().random(xyz_lower, xyz_upper, rpy_lower, rpy_upper).set_header(frame_id=frame_id)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray, frame_id='', header: Header = None):
        return super().from_matrix(matrix).set_header(frame_id=frame_id, header=header)

    @classmethod
    def from_transform_stamped(cls, transform_stamped: 'TransformStamped') -> 'PoseStamped':
        return cls(header=transform_stamped.header, pose=transform_stamped.transform.to_pose())

    def to_transform_stamped(self, child_frame_id='') -> 'TransformStamped':
        from .transform_stamped import TransformStamped
        return TransformStamped(header=self.header, transform=self.pose.to_transform(), child_frame_id=child_frame_id)

    @classmethod
    def from_list(cls, list: List[float], euler=True, frame_id='', header: Header = None) -> 'PoseStamped':
        return super().from_list(list, euler).set_header(frame_id=frame_id, header=header)

    @classmethod
    def from_xyzrpy(cls, xyz: list[float], rpy: list[float], frame_id='', header: Header = None):
        return cls.from_list(list=list(xyz) + list(rpy), euler=True, frame_id=frame_id, header=header)
    
    @dataclass
    class Dist:
        """Distribution specification for PoseStamped (mirrors message structure)."""
        pose: 'Pose.Dist' = field(default_factory=Pose.Dist)
        # Note: header is not part of distribution - frame_id is set separately
        _seed: Optional[int] = field(default=None, init=False, repr=False)
        
        def __post_init__(self):
            """Initialize RNG."""
            self.rng = np.random.default_rng(self._seed)
        
        def sample(self, frame_id: str = '') -> 'PoseStamped':
            """Sample a concrete PoseStamped.
            
            Args:
                frame_id: Optional frame_id for the header
            """
            return PoseStamped(
                header=Header(frame_id=frame_id),
                pose=self.pose.sample()
            )
        
        def get_range(self) -> tuple[Optional[dict], Optional[dict]]:
            """Get bounds as nested dict."""
            return self.pose.get_range()
        
        def contains(self, pose_stamped: 'PoseStamped') -> bool:
            """Check if pose_stamped is within bounds."""
            return self.pose.contains(pose_stamped.pose)
        
        def seed(self, seed: Optional[int] = None) -> int:
            """Seed the RNG for all component distributions."""
            if seed is None:
                seed = np.random.randint(0, 2**31)
            self._seed = seed
            self.rng = np.random.default_rng(seed)
            # Seed component distributions
            self.pose.seed(seed)
            return seed
        
        def generate_dataset(self, n: int, frame_id: str = '') -> list['PoseStamped']:
            """Generate a dataset of n PoseStamped samples.
            
            Args:
                n: Number of samples
                frame_id: Optional frame_id for all headers
            """
            return [self.sample(frame_id=frame_id) for _ in range(n)]
        
        @classmethod
        def uniform(cls, low: float, high: float) -> 'PoseStamped.Dist':
            """Uniform distribution for position, identity for orientation."""
            return cls(
                pose=Pose.Dist.uniform(low, high)
            )
        
        @classmethod
        def uniform_with_rotation(cls, pos_low: float, pos_high: float,
                                  rpy_lower: tuple[float, float, float] = (-np.pi, -np.pi, -np.pi),
                                  rpy_upper: tuple[float, float, float] = (np.pi, np.pi, np.pi)) -> 'PoseStamped.Dist':
            """Uniform distribution for both position and orientation (via euler angles)."""
            return cls(
                pose=Pose.Dist.uniform_with_rotation(pos_low, pos_high, rpy_lower, rpy_upper)
            )




if __name__ == "__main__":
    # Test basic initialization
    pose_stamped = PoseStamped()
    pose_stamped.offset()
    PoseStamped.random()

    # print("Empty pose_stamped:")
    # print(pose_stamped)
    # print()

    # # Test random with limits
    # xyz_lower = [0.0, 0.0, 0.5]
    # xyz_upper = [1.0, 1.0, 1.5]
    # rpy_lower = [0.0, 0.0, 0.0]
    # rpy_upper = [0.2, 0.2, 0.2]
    
    # pose_stamped_limited = PoseStamped.random(
    #     frame_id='world',
    #     xyz_lower=xyz_lower,
    #     xyz_upper=xyz_upper,
    #     rpy_lower=rpy_lower,
    #     rpy_upper=rpy_upper
    # )
    # print(f"Limited random PoseStamped:")
    # print(f"  frame_id: {pose_stamped_limited.header.frame_id}")
    # print(f"  xyz: {pose_stamped_limited.xyz}")
    # print(f"  rpy: {pose_stamped_limited.rpy}")
