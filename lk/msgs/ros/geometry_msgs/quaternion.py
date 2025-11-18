from dataclasses import dataclass, field
from typing import Optional
from tf_transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    euler_from_quaternion,
    euler_from_matrix,
    quaternion_from_euler,
    random_quaternion,
    quaternion_slerp,
    quaternion_multiply,
    quaternion_inverse,
    quaternion_to_axangle,
)
import numpy as np

from ..ros_msg import RosMsg

@dataclass
class Quaternion(RosMsg):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classmethod    
    def random(cls, rpy_lower=None, rpy_upper=None) -> 'Quaternion':
        if rpy_lower is None and rpy_upper is None:
            return cls(*random_quaternion())
        
        if rpy_lower is None:
            rpy_lower = [-np.pi, -np.pi, -np.pi]
        if rpy_upper is None:
            rpy_upper = [np.pi, np.pi, np.pi]
        
        roll = np.random.uniform(rpy_lower[0], rpy_upper[0])
        pitch = np.random.uniform(rpy_lower[1], rpy_upper[1])
        yaw = np.random.uniform(rpy_lower[2], rpy_upper[2])
        return cls.from_euler(roll, pitch, yaw)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray): #4x4 matrix
        return cls(*quaternion_from_matrix(matrix))

    def to_matrix(self) -> np.ndarray:
        return quaternion_matrix([self.x, self.y, self.z, self.w])

    @classmethod
    def from_list(cls, list: list[float], euler=False, wxyz=False) -> 'Quaternion':
        if euler:
            assert len(list) == 3, "For euler, list must be length 3"
            return cls(*quaternion_from_euler(*list))
        else:
            if wxyz:
                return cls(w=list[0], x=list[1], y=list[2], z=list[3])
            else:
                return cls(x=list[0], y=list[1], z=list[2], w=list[3])

    @property
    def wxyz(self) -> list[float]:
        return self.to_list(wxyz=True)

    @property
    def xyzw(self) -> list[float]:
        return self.to_list(wxyz=False)

    @property
    def rpy(self) -> list[float]:
        return self.to_list(euler=True)
    
    @rpy.setter
    def rpy(self, value: list | tuple | np.ndarray):
        roll, pitch, yaw = float(value[0]), float(value[1]), float(value[2])
        q = quaternion_from_euler(roll, pitch, yaw)
        self.x, self.y, self.z, self.w = q[0], q[1], q[2], q[3]

    def to_list(self, euler=False, wxyz=False) -> list[float]:
        if euler:
            return list(euler_from_matrix(self.to_matrix()))
        elif wxyz:
            return [self.w, self.x, self.y, self.z]
        else:
            return [self.x, self.y, self.z, self.w]

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> 'Quaternion':
        return cls(*quaternion_from_euler(roll, pitch, yaw))

    def slerp(self, target: 'Quaternion', fraction: float) -> 'Quaternion':
        new_q = quaternion_slerp(self.to_list(), target.to_list(), fraction)
        return Quaternion(*new_q)

    def axangle_diff(self, target: 'Quaternion') -> tuple[np.ndarray, float]:

        q_diff = quaternion_multiply(target.to_list(), quaternion_inverse(self.to_list()))
        vec, theta = quaternion_to_axangle(q_diff)
        return vec, theta

    def euler_diff(self, target: 'Quaternion') -> float:
        q_diff = quaternion_multiply(target.to_list(), quaternion_inverse(self.to_list()))
        euler_error = np.array(euler_from_quaternion(q_diff))
        return euler_error

    def to_dict(self):
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "w": float(self.w),
        }
    
    @classmethod
    def from_dict(cls, d: dict):
        x = float(d.get("x", 0.0))
        y = float(d.get("y", 0.0)) 
        z = float(d.get("z", 0.0))
        w = float(d.get("w", 1.0))
        return cls(x, y, z, w)

    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
        self.z = float(self.z)
        self.w = float(self.w)
    
    @dataclass
    class Dist:
        """Distribution specification for Quaternion with gymnasium Space compatibility."""
        from lk.msgs.random_msgs.float_dist import FloatDist
        
        x: 'FloatDist' = field(default_factory=lambda: FloatDist.fixed(0.0))
        y: 'FloatDist' = field(default_factory=lambda: FloatDist.fixed(0.0))
        z: 'FloatDist' = field(default_factory=lambda: FloatDist.fixed(0.0))
        w: 'FloatDist' = field(default_factory=lambda: FloatDist.fixed(1.0))
        _seed: Optional[int] = field(default=None, init=False, repr=False)
        
        def __post_init__(self):
            """Initialize RNG."""
            self.rng = np.random.default_rng(self._seed)
        
        def sample(self) -> 'Quaternion':
            """Sample a concrete Quaternion."""
            return Quaternion(
                x=self.x.sample(),
                y=self.y.sample(),
                z=self.z.sample(),
                w=self.w.sample()
            )
        
        def get_range(self) -> tuple[Optional[tuple], Optional[tuple]]:
            """Get (min, max) bounds as tuples."""
            x_min, x_max = self.x.get_range()
            y_min, y_max = self.y.get_range()
            z_min, z_max = self.z.get_range()
            w_min, w_max = self.w.get_range()
            
            min_q = (x_min, y_min, z_min, w_min) if all(v is not None for v in [x_min, y_min, z_min, w_min]) else None
            max_q = (x_max, y_max, z_max, w_max) if all(v is not None for v in [x_max, y_max, z_max, w_max]) else None
            return (min_q, max_q)
        
        def contains(self, quat: 'Quaternion') -> bool:
            """Check if quaternion is within bounds."""
            return (self.x.contains(quat.x) and 
                    self.y.contains(quat.y) and 
                    self.z.contains(quat.z) and
                    self.w.contains(quat.w))
        
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
            self.w.seed(seed + 3)
            return seed
        
        def generate_dataset(self, n: int) -> list['Quaternion']:
            """Generate a dataset of n Quaternion samples."""
            return [self.sample() for _ in range(n)]
        
        @classmethod
        def identity(cls) -> 'Quaternion.Dist':
            """Fixed identity quaternion (no rotation)."""
            from lk.msgs.random_msgs.float_dist import FloatDist
            return cls(
                x=FloatDist.fixed(0.0),
                y=FloatDist.fixed(0.0),
                z=FloatDist.fixed(0.0),
                w=FloatDist.fixed(1.0)
            )
        
        @classmethod
        def from_euler_dist(cls, roll_dist: 'FloatDist', pitch_dist: 'FloatDist', yaw_dist: 'FloatDist') -> 'Quaternion.Dist':
            """Create Quaternion.Dist from euler angle distributions.
            
            Note: This samples the euler angles then converts, not a true quaternion distribution.
            """
            # This is a wrapper that will sample euler angles and convert
            # We'll need a custom sample function for this
            raise NotImplementedError("Euler-based quaternion distribution not yet implemented")
        
        @classmethod
        def uniform_rpy(cls, rpy_lower: tuple[float, float, float] = (-np.pi, -np.pi, -np.pi),
                       rpy_upper: tuple[float, float, float] = (np.pi, np.pi, np.pi)) -> 'Quaternion.Dist':
            """Uniform distribution over euler angles (roll, pitch, yaw).
            
            Creates a distribution that samples uniform euler angles and converts to quaternion.
            """
            # Create a special distribution that samples in euler space
            class EulerBasedQuaternionDist:
                def __init__(self, rpy_lower, rpy_upper):
                    from lk.msgs.random_msgs.float_dist import FloatDist
                    self.roll_dist = FloatDist.uniform(rpy_lower[0], rpy_upper[0])
                    self.pitch_dist = FloatDist.uniform(rpy_lower[1], rpy_upper[1])
                    self.yaw_dist = FloatDist.uniform(rpy_lower[2], rpy_upper[2])
                    self._seed = None
                    self.rng = np.random.default_rng(self._seed)
                
                def sample(self) -> 'Quaternion':
                    roll = self.roll_dist.sample()
                    pitch = self.pitch_dist.sample()
                    yaw = self.yaw_dist.sample()
                    return Quaternion.from_euler(roll, pitch, yaw)
                
                def get_range(self):
                    return (rpy_lower, rpy_upper)
                
                def contains(self, quat: 'Quaternion') -> bool:
                    rpy = quat.rpy
                    return (rpy_lower[0] <= rpy[0] <= rpy_upper[0] and
                           rpy_lower[1] <= rpy[1] <= rpy_upper[1] and
                           rpy_lower[2] <= rpy[2] <= rpy_upper[2])
                
                def seed(self, seed: Optional[int] = None) -> int:
                    if seed is None:
                        seed = np.random.randint(0, 2**31)
                    self._seed = seed
                    self.rng = np.random.default_rng(seed)
                    self.roll_dist.seed(seed)
                    self.pitch_dist.seed(seed + 1)
                    self.yaw_dist.seed(seed + 2)
                    return seed
                
                def generate_dataset(self, n: int) -> list['Quaternion']:
                    return [self.sample() for _ in range(n)]
            
            return EulerBasedQuaternionDist(rpy_lower, rpy_upper)

if __name__ == "__main__":
    q = Quaternion.random()

    # Euler
    q_list = q.to_list(euler=True)
    q_round_trip = Quaternion.from_list(q_list, euler=True)

    print(np.array(q_list))
    print(np.array(q_round_trip.to_list(euler=True)))
    assert np.allclose(np.array(q_list), np.array(q_round_trip.to_list(euler=True)), atol=1e-6)

    # Euler
    q_list = q.to_list(euler=False)
    q_round_trip = Quaternion.from_list(q_list, euler=False)

    print(np.array(q_list))
    print(np.array(q_round_trip.to_list(euler=False)))

    assert np.allclose(np.array(q_list), np.array(q_round_trip.to_list(euler=False)), atol=1e-6)

    # Test Quaternion.slerp - interpolate from identity (no rotation) to random quat
    q_identity = Quaternion(0, 0, 0, 1)
    q_rand = Quaternion.random()
    n = 5
    print(f"\nSlerp between q_identity and q_rand {q_rand.to_list()}:")
    for i in range(n + 1):
        frac = i / n
        q_interp = q_identity.slerp(q_rand, frac)
        print(f"  fraction={frac:.2f}  quat={q_interp.to_list()}")
