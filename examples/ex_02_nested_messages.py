#!/usr/bin/env python3

"""
    Nested Message Example
    
    Demonstrates how to use nested dataclasses with Msg base class.
    This is useful for complex observation spaces or hierarchical data.
"""

# BAM
from lk.msgs.msg import Msg, Observation

# PYTHON
from dataclasses import dataclass
from typing import List, Optional
import numpy as np


# =============================================================================
# Define Nested Message Structures
# =============================================================================

@dataclass
class CameraObservation(Msg):
    """Camera sensor data."""
    image: np.ndarray
    width: int
    height: int
    timestamp: float


@dataclass
class LidarObservation(Msg):
    """Lidar sensor data."""
    points: np.ndarray
    num_points: int
    timestamp: float


@dataclass
class RobotState(Msg):
    """Robot state information."""
    position: List[float]  # [x, y, z]
    velocity: List[float]  # [vx, vy, vz]
    joint_angles: List[float]
    joint_velocities: List[float]


@dataclass
class MultiModalObservation(Msg):
    """
        Complex observation with multiple sensor modalities.
        
        This demonstrates dacite's ability to handle nested dataclasses,
        optional fields, and type conversions automatically.
    """
    camera: CameraObservation
    lidar: Optional[LidarObservation]
    robot_state: RobotState
    metadata: dict


# =============================================================================
# Example Usage
# =============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("Nested Dataclass Message Example")
    print("=" * 70)
    
    # Create nested observation directly
    print("\n1. Creating nested observation object...")
    obs = MultiModalObservation(
        camera=CameraObservation(
            image=np.random.rand(480, 640, 3),
            width=640,
            height=480,
            timestamp=1.23
        ),
        lidar=LidarObservation(
            points=np.random.rand(1000, 3),
            num_points=1000,
            timestamp=1.23
        ),
        robot_state=RobotState(
            position=[1.0, 2.0, 0.5],
            velocity=[0.1, 0.0, 0.0],
            joint_angles=[0.0, 1.57, -1.57, 0.0, 0.0, 0.0],
            joint_velocities=[0.0] * 6
        ),
        metadata={'episode': 1, 'step': 42}
    )
    print(f"✓ Created observation with camera, lidar, and robot state")
    
    # Serialize to dictionary
    print("\n2. Serializing to dictionary...")
    obs_dict = obs.to_dict()
    print(f"✓ Serialized to dict with keys: {list(obs_dict.keys())}")
    print(f"  - camera shape: {obs_dict['camera']['image'].shape}")
    print(f"  - lidar points: {obs_dict['lidar']['num_points']}")
    print(f"  - robot position: {obs_dict['robot_state']['position']}")
    
    # Deserialize from dictionary (this is where dacite shines!)
    print("\n3. Deserializing from dictionary...")
    restored = MultiModalObservation.from_dict(obs_dict)
    print(f"✓ Restored observation")
    print(f"  - camera: {restored.camera.width}x{restored.camera.height}")
    print(f"  - lidar points: {restored.lidar.num_points}")
    print(f"  - robot position: {restored.robot_state.position}")
    
    # Create from dictionary with optional field missing
    print("\n4. Creating from dict with optional field (lidar=None)...")
    partial_dict = {
        'camera': {
            'image': np.random.rand(480, 640, 3),
            'width': 640,
            'height': 480,
            'timestamp': 2.34
        },
        'lidar': None,
        'robot_state': {
            'position': [2.0, 3.0, 1.0],
            'velocity': [0.0, 0.1, 0.0],
            'joint_angles': [0.5] * 6,
            'joint_velocities': [0.0] * 6
        },
        'metadata': {'episode': 2, 'step': 10}
    }
    
    partial_obs = MultiModalObservation.from_dict(partial_dict)
    print(f"✓ Created observation without lidar")
    print(f"  - lidar is None: {partial_obs.lidar is None}")
    print(f"  - camera still works: {partial_obs.camera.width}x{partial_obs.camera.height}")
    
    # Show that nested dataclasses are properly typed
    print("\n5. Type checking...")
    assert isinstance(restored.camera, CameraObservation)
    assert isinstance(restored.lidar, LidarObservation)
    assert isinstance(restored.robot_state, RobotState)
    print(f"✓ All nested objects have correct types")
    
    print("\n" + "=" * 70)
    print("Key Benefits:")
    print("  - Automatic nested dataclass deserialization")
    print("  - Optional field handling")
    print("  - Type safety throughout")
    print("  - Clean, declarative message definitions")
    print("=" * 70)




