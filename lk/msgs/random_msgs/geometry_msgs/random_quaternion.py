#!/usr/bin/env python3

"""
    RandomQuaternion - Random version of geometry_msgs/Quaternion.
    
    Parameterized by axis-angle (initial view + max deviation) rather than
    per-component sampling. Supports both discrete and continuous modes.
"""

# BAM
from ..random_type import RandomType
from bam.msgs.ros_msgs import Quaternion

# PYTHON
from typing import Optional, Literal, Callable
from dataclasses import dataclass, field
import numpy as np
import math

# Import from tf_transformations (handles xyzw ordering correctly)
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_multiply,
    quaternion_about_axis,
    axis_angle_between
)



def sample_spherical_cap(initial_view: np.ndarray, deviation_angle: float, 
                        rotation_min: float, rotation_max: float,
                        rng: np.random.Generator) -> Quaternion:
    """
    Sample uniform quaternion: body z-axis within deviation_angle of initial_view,
    with uniform spin in [rotation_min, rotation_max] around body z-axis.
    
    Based on spherical cap sampling method.
    Uses tf_transformations for proper quaternion operations.
    """
    # Normalize initial view
    initial_view = initial_view / np.linalg.norm(initial_view)
    
    # Sample direction on spherical cap centered around +Z: angle θ ∈ [0, deviation_angle], φ ∈ [0, 2π)
    u = rng.random()
    cos_theta = 1 - u * (1 - np.cos(deviation_angle))  # uniform on cap
    sin_theta = np.sqrt(1 - cos_theta**2)
    phi = 2*np.pi * rng.random()
    
    # Sampled direction in local frame (cap centered around +Z)
    z_dir_local = np.array([
        sin_theta*np.cos(phi),
        sin_theta*np.sin(phi),
        cos_theta
    ])
    
    # Build quaternion that rotates +Z → initial_view (aligns cap to desired direction)
    axis_to_view, angle_to_view = axis_angle_between(np.array([0, 0, 1.0]), initial_view)
    q_to_view = quaternion_about_axis(angle_to_view, axis_to_view)
    
    # Build quaternion that rotates +Z → z_dir_local
    axis_local, angle_local = axis_angle_between(np.array([0, 0, 1.0]), z_dir_local)
    q_local = quaternion_about_axis(angle_local, axis_local)
    
    # Combine: first apply local deviation, then rotate to initial_view
    q_align = quaternion_multiply(q_to_view, q_local)
    
    # Uniform spin about body z-axis in [rotation_min, rotation_max]
    psi = rng.uniform(rotation_min, rotation_max)
    q_spin = quaternion_about_axis(psi, np.array([0, 0, 1]))
    
    # Hamilton product: q = q_align ⊗ q_spin (tf_transformations.quaternion_multiply)
    q = quaternion_multiply(q_align, q_spin)
    
    # Normalize and convert to Quaternion
    q = np.array(q)
    q = q / np.linalg.norm(q)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


@dataclass
class RandomQuaternion(RandomType):
    """Random Quaternion parameterized by axis-angle (initial view + deviation).
    
    Supports:
    - Discrete: List of quaternions from view_generator (permutable)
    - Continuous: Spherical cap sampling around initial view
    """
    
    mode: Literal["fixed", "discrete", "continuous"]
    
    # Fixed mode
    fixed_quat: Optional[Quaternion] = None
    
    # Discrete mode
    quaternion_list: Optional[list[Quaternion]] = field(default=None)
    current_index: int = field(default=0, init=False)
    
    # Continuous mode
    initial_view: Optional[np.ndarray] = None  # Axis direction
    hemisphere_angle: Optional[float] = None   # Max deviation (rad)
    rotation_min: float = 0.0                  # Min in-plane rotation (rad)
    rotation_max: float = 2*np.pi              # Max in-plane rotation (rad)
    
    # Sampling function (for continuous mode)
    sample_fn: Optional[Callable[[np.random.Generator], Quaternion]] = None
    
    # Seed
    _seed: Optional[int] = field(default=None, init=False, repr=False)
    
    def __post_init__(self):
        """Initialize based on mode."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_quat: Optional[Quaternion] = None
        
        if self.mode == "continuous" and self.sample_fn is None:
            # Create sampling function for continuous mode
            initial = np.array(self.initial_view) if self.initial_view is not None else np.array([0, 0, 1])
            hemisphere = self.hemisphere_angle if self.hemisphere_angle is not None else np.pi/2
            rot_min = self.rotation_min
            rot_max = self.rotation_max
            
            self.sample_fn = lambda rng: sample_spherical_cap(
                initial, hemisphere, rot_min, rot_max, rng
            )
    
    def with_seed(self, seed: int) -> 'RandomQuaternion':
        """Set seed (chainable)."""
        self._seed = seed
        self.rng = np.random.default_rng(seed)
        return self
    
    def sample(self) -> Quaternion:
        """Sample a concrete Quaternion."""
        if self.mode == "fixed":
            self.curr_quat = self.fixed_quat
        
        elif self.mode == "discrete":
            if self.quaternion_list is None or len(self.quaternion_list) == 0:
                raise ValueError("discrete mode requires quaternion_list")
            self.curr_quat = self.quaternion_list[self.current_index]
        
        elif self.mode == "continuous":
            if self.sample_fn is None:
                raise ValueError("continuous mode requires sample_fn")
            self.curr_quat = self.sample_fn(self.rng)
        
        return self.curr_quat
    
    def get_range(self) -> tuple:
        """Get range information."""
        if self.mode == "fixed":
            return (None, None)
        elif self.mode == "discrete":
            return (0, len(self.quaternion_list) - 1)
        elif self.mode == "continuous":
            return (f"hemisphere_angle={np.rad2deg(self.hemisphere_angle):.1f}deg",
                   f"rotation=[{np.rad2deg(self.rotation_min):.1f}, {np.rad2deg(self.rotation_max):.1f}]deg")
    
    def permute(self) -> tuple[Quaternion, bool]:
        """Permute to next discrete value (only for discrete mode)."""
        if self.mode != "discrete":
            raise NotImplementedError("permute() only supported for discrete mode")
        
        if self.quaternion_list is None:
            raise ValueError("discrete mode requires quaternion_list")
        
        self.current_index = (self.current_index + 1) % len(self.quaternion_list)
        quat = self.sample()
        
        done = (self.current_index == 0)
        return quat, done
    
    @property
    def permutation_total(self) -> int:
        """Total number of permutations (only for discrete mode)."""
        if self.mode != "discrete":
            raise NotImplementedError("permutation_total only supported for discrete mode")
        return len(self.quaternion_list) if self.quaternion_list else 0
    
    @classmethod
    def identity(cls) -> 'RandomQuaternion':
        """Create identity quaternion (no rotation)."""
        return cls(
            mode="fixed",
            fixed_quat=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    
    @classmethod
    def fixed(cls, x: float, y: float, z: float, w: float) -> 'RandomQuaternion':
        """Create fixed Quaternion."""
        return cls(
            mode="fixed",
            fixed_quat=Quaternion(x=x, y=y, z=z, w=w)
        )
    
    @classmethod
    def from_matrix(cls, R: np.ndarray) -> 'RandomQuaternion':
        """Create fixed quaternion from rotation matrix (3x3 or 4x4)."""
        # Convert 3x3 to 4x4 if needed
        if R.shape == (3, 3):
            T = np.eye(4)
            T[:3, :3] = R
            q = quaternion_from_matrix(T)
        else:
            q = quaternion_from_matrix(R)
        return cls.fixed(q[0], q[1], q[2], q[3])
    
    @classmethod
    def discrete_from_view_generator(cls,
                                     initial_view: list[float] = [0, 0, 1],
                                     hemisphere_angle: float = np.deg2rad(90),
                                     view_step: float = np.deg2rad(45),
                                     rotation_step: float = np.deg2rad(60),
                                     start_angle: float = 0.0,
                                     max_rotation_steps: float = np.inf) -> 'RandomQuaternion':
        """Create discrete quaternions using view_generator.
        
        DescriptionArgs:
            initial_view: Base viewing direction [x, y, z]
            hemisphere_angle: Max deviation from initial_view (rad)
            view_step: Angular step for sampling directions (rad)
            rotation_step: Angular step for in-plane rotations (rad)
            start_angle: Starting rotation angle (rad)
            max_rotation_steps: Maximum number of rotation steps
            
        Returns:
            RandomQuaternion in discrete mode with list of quaternions
        """
        # Import view_generator (handle if not available)
        try:
            from bam_reach.generators.view_generators import view_generator
            
            R_list = view_generator(
                inital_view=initial_view,
                hemisphere_angle=hemisphere_angle,
                view_step=view_step,
                rotation_step=rotation_step,
                start_angle=start_angle,
                max_rotation_steps=max_rotation_steps
            )
        except (ImportError, Exception) as e:
            # Fallback: return identity only if view_generator not available
            print(f"Warning: view_generator not available ({e}), using identity quaternion")
            R_list = [np.eye(3)]
        
        # Convert rotation matrices to quaternions
        quaternion_list = []
        for R in R_list:
            # Convert 3x3 to 4x4 if needed
            if R.shape == (3, 3):
                T = np.eye(4)
                T[:3, :3] = R
                q = quaternion_from_matrix(T)
            else:
                q = quaternion_from_matrix(R)
            quaternion_list.append(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
        
        return cls(
            mode="discrete",
            quaternion_list=quaternion_list
        )
    
    @classmethod
    def discrete_from_list(cls, quaternion_list: list[Quaternion]) -> 'RandomQuaternion':
        """Create discrete quaternion from explicit list of quaternions.
        
        DescriptionArgs:
            quaternion_list: List of Quaternion objects
            
        Returns:
            RandomQuaternion in discrete mode
        """
        return cls(
            mode="discrete",
            quaternion_list=quaternion_list
        )
    
    @classmethod
    def continuous_hemisphere(cls,
                             initial_view: list[float] = [0, 0, 1],
                             hemisphere_angle: float = np.deg2rad(90),
                             rotation_min: float = 0.0,
                             rotation_max: float = 2*np.pi) -> 'RandomQuaternion':
        """Create continuous quaternion sampler using spherical cap method.
        
        Uniformly samples orientations whose body z-axis lies within 
        hemisphere_angle of initial_view, with uniform spin in [rotation_min, rotation_max].
        
        DescriptionArgs:
            initial_view: Base viewing direction [x, y, z]
            hemisphere_angle: Max deviation from initial_view (rad)
            rotation_min: Minimum in-plane rotation (rad)
            rotation_max: Maximum in-plane rotation (rad)
            
        Returns:
            RandomQuaternion in continuous mode
        """
        return cls(
            mode="continuous",
            initial_view=np.array(initial_view),
            hemisphere_angle=hemisphere_angle,
            rotation_min=rotation_min,
            rotation_max=rotation_max
        )
    
    @classmethod
    def uniform_euler(cls,
                     rpy_lower: list[float],
                     rpy_upper: list[float]) -> 'RandomQuaternion':
        """Create continuous quaternion sampler using uniform Euler angle sampling.
        
        Samples roll, pitch, yaw independently and uniformly within specified bounds,
        then converts to quaternion. Note: This is NOT uniform on SO(3) but is often
        more intuitive for small angle variations.
        
        DescriptionArgs:
            rpy_lower: Lower bounds [roll, pitch, yaw] in radians
            rpy_upper: Upper bounds [roll, pitch, yaw] in radians
            
        Returns:
            RandomQuaternion in continuous mode with custom sampling function
        """
        from ..random_float import RandomFloat
        
        # Create RandomFloat samplers for each Euler angle
        roll_sampler = RandomFloat.uniform(rpy_lower[0], rpy_upper[0])
        pitch_sampler = RandomFloat.uniform(rpy_lower[1], rpy_upper[1])
        yaw_sampler = RandomFloat.uniform(rpy_lower[2], rpy_upper[2])
        
        def sample_euler(rng: np.random.Generator) -> Quaternion:
            """Sample Euler angles and convert to quaternion."""
            # Share the RNG with RandomFloat samplers
            roll_sampler.rng = rng
            pitch_sampler.rng = rng
            yaw_sampler.rng = rng
            
            r = roll_sampler.sample()
            p = pitch_sampler.sample()
            y = yaw_sampler.sample()
            
            # Convert RPY to quaternion using tf_transformations convention
            # This uses the ZYX Euler angle convention (yaw, pitch, roll)
            cy = np.cos(y * 0.5)
            sy = np.sin(y * 0.5)
            cp = np.cos(p * 0.5)
            sp = np.sin(p * 0.5)
            cr = np.cos(r * 0.5)
            sr = np.sin(r * 0.5)
            
            # Quaternion multiplication: Rz(yaw) * Ry(pitch) * Rx(roll)
            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            
            return Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        return cls(
            mode="continuous",
            sample_fn=sample_euler
        )


if __name__ == '__main__':
    from ..random_type import RandomType
    from tf_transformations.vectors import angle_between
    import numpy as np
    
    print("\n" + "="*70)
    print("RandomQuaternion - Axis-Angle Parameterization")
    print("="*70)
    
    # Identity
    rq_identity = RandomQuaternion.identity()
    print(f"\n[1] Identity: {isinstance(rq_identity, RandomType)}")
    quat = rq_identity.sample()
    print(f"  xyzw={quat.xyzw}, rpy={np.rad2deg(quat.rpy)}")
    
    # Fixed
    rq_fixed = RandomQuaternion.fixed(0.0, 0.0, 0.707, 0.707)
    print(f"\n[2] Fixed: {isinstance(rq_fixed, RandomType)}")
    quat = rq_fixed.sample()
    print(f"  xyzw={quat.xyzw}, rpy={np.rad2deg(quat.rpy)}")
    
    # Discrete from view_generator
    print(f"\n[3] Discrete (from view_generator):")
    try:
        rq_discrete = RandomQuaternion.discrete_from_view_generator(
            initial_view=[0, 0, 1],
            hemisphere_angle=np.deg2rad(45),
            view_step=np.deg2rad(45),
            rotation_step=np.deg2rad(90)
        ).with_seed(42)
        print(f"  Total permutations: {rq_discrete.permutation_total}")
        print(f"  isinstance(RandomType): {isinstance(rq_discrete, RandomType)}")
        
        for i in range(min(5, rq_discrete.permutation_total)):
            quat, done = rq_discrete.permute()
            print(f"    [{i}] rpy={np.rad2deg(quat.rpy)}, done={done}")
    except Exception as e:
        print(f"  Skipped (view_generator not available): {e}")
    
    # Continuous hemisphere
    print(f"\n[4] Continuous Hemisphere:")
    rq_continuous = RandomQuaternion.continuous_hemisphere(
        initial_view=[0, 0, 1],
        hemisphere_angle=np.deg2rad(30),
        rotation_min=0.0,
        rotation_max=np.pi
    ).with_seed(42)
    
    print(f"  isinstance(RandomType): {isinstance(rq_continuous, RandomType)}")
    print(f"  Range: {rq_continuous.get_range()}")
    
    for i in range(5):
        quat = rq_continuous.sample()
        # Extract z-axis from quaternion rotation matrix
        R = quat.to_matrix()
        if R.shape == (4, 4):
            z_axis = R[:3, 2]
        else:
            z_axis = R[:, 2]
        angle_from_z = angle_between(z_axis, np.array([0, 0, 1]))
        print(f"    Sample {i}: rpy={np.rad2deg(quat.rpy)}, z_deviation={np.rad2deg(angle_from_z):.1f}deg")
    
    print("\n" + "="*70)
    print("Clean API: Parameterized by axis (initial_view) + angle (hemisphere_angle)!")
    print("="*70)
