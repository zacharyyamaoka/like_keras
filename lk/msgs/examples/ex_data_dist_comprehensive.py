#!/usr/bin/env python3

"""
    Comprehensive examples demonstrating DataDist usage with messages.
    
    Shows how to use nested Dist classes for:
    - Sampling random message instances
    - Validating bounds
    - Generating test datasets
    - Interface specifications
"""

# BAM
from lk.msgs.random_msgs.float_dist import FloatDist
from lk.msgs.random_msgs.int_dist import IntDist
from lk.msgs.ros.geometry_msgs import Point, Quaternion, Pose, PoseStamped
import numpy as np


def example_1_basic_float_dist():
    """Example 1: Basic FloatDist usage."""
    print("\n" + "="*70)
    print("Example 1: Basic FloatDist")
    print("="*70)
    
    # Create various distributions
    fd_fixed = FloatDist.fixed(5.0)
    fd_uniform = FloatDist.uniform(-1.0, 1.0)
    fd_normal = FloatDist.normal(0.0, 0.1)
    
    # Sample from distributions
    print(f"\nFixed(5.0): {[fd_fixed.sample() for _ in range(3)]}")
    print(f"Uniform(-1, 1): {[f'{fd_uniform.sample():.3f}' for _ in range(5)]}")
    print(f"Normal(0, 0.1): {[f'{fd_normal.sample():.3f}' for _ in range(5)]}")
    
    # Check bounds
    print(f"\nUniform bounds: {fd_uniform.get_range()}")
    print(f"Contains 0.5: {fd_uniform.contains(0.5)}")
    print(f"Contains 2.0: {fd_uniform.contains(2.0)}")


def example_2_point_dist():
    """Example 2: Point.Dist for 3D points."""
    print("\n" + "="*70)
    print("Example 2: Point.Dist")
    print("="*70)
    
    # Uniform distribution on all axes
    point_dist = Point.Dist.uniform(-1.0, 1.0)
    
    print("\nSampling from Point.Dist.uniform(-1, 1):")
    for i in range(5):
        point = point_dist.sample()
        print(f"  Sample {i+1}: {point.to_list()}")
    
    # Check bounds
    print(f"\nBounds: {point_dist.get_range()}")
    
    # Validate a point
    test_point = Point(x=0.5, y=0.5, z=0.5)
    print(f"Contains {test_point.to_list()}: {point_dist.contains(test_point)}")
    
    test_point_out = Point(x=2.0, y=0.0, z=0.0)
    print(f"Contains {test_point_out.to_list()}: {point_dist.contains(test_point_out)}")
    
    # Custom per-axis distributions
    print("\n\nCustom per-axis distributions:")
    point_dist_custom = Point.Dist(
        x=FloatDist.normal(0.0, 0.1),
        y=FloatDist.uniform(-2.0, 2.0),
        z=FloatDist.fixed(1.0)
    )
    
    print("x: Normal(0, 0.1), y: Uniform(-2, 2), z: Fixed(1.0)")
    for i in range(5):
        point = point_dist_custom.sample()
        print(f"  Sample {i+1}: [{point.x:.3f}, {point.y:.3f}, {point.z:.3f}]")


def example_3_pose_stamped_dist():
    """Example 3: PoseStamped.Dist for robot poses."""
    print("\n" + "="*70)
    print("Example 3: PoseStamped.Dist")
    print("="*70)
    
    # Simple uniform distribution for position only
    pose_dist = PoseStamped.Dist.uniform(-1.0, 1.0)
    
    print("\nSampling from PoseStamped.Dist.uniform(-1, 1):")
    for i in range(3):
        pose = pose_dist.sample(frame_id='world')
        print(f"  Sample {i+1}:")
        print(f"    Position: {pose.pose.position.to_list()}")
        print(f"    Orientation (rpy): {[f'{x:.3f}' for x in pose.pose.orientation.rpy]}")
    
    # Uniform distribution with rotation
    print("\n\nWith rotation:")
    pose_dist_rot = PoseStamped.Dist.uniform_with_rotation(
        pos_low=-1.0, pos_high=1.0,
        rpy_lower=(-0.1, -0.1, -np.pi),
        rpy_upper=(0.1, 0.1, np.pi)
    )
    
    print("Position: Uniform(-1, 1), RPY: Limited range")
    for i in range(3):
        pose = pose_dist_rot.sample(frame_id='world')
        print(f"  Sample {i+1}:")
        print(f"    Position: {[f'{x:.3f}' for x in pose.pose.position.to_list()]}")
        print(f"    RPY: {[f'{x:.3f}' for x in pose.pose.orientation.rpy]}")


def example_4_dataset_generation():
    """Example 4: Generating test datasets."""
    print("\n" + "="*70)
    print("Example 4: Generate Test Datasets")
    print("="*70)
    
    # Generate dataset of points
    point_dist = Point.Dist.uniform(-1.0, 1.0)
    point_dist.seed(42)  # For reproducibility
    
    dataset = point_dist.generate_dataset(10)
    
    print("\nGenerated 10 point samples:")
    for i, point in enumerate(dataset):
        print(f"  {i+1}: {[f'{x:.3f}' for x in point.to_list()]}")
    
    # Generate dataset of poses
    pose_dist = PoseStamped.Dist.uniform(-2.0, 2.0)
    pose_dist.seed(42)
    
    pose_dataset = pose_dist.generate_dataset(5, frame_id='world')
    
    print("\nGenerated 5 pose samples:")
    for i, pose in enumerate(pose_dataset):
        print(f"  {i+1}: xyz={[f'{x:.2f}' for x in pose.pose.position.to_list()]}")


def example_5_interface_specification():
    """Example 5: Using Dist as interface specification."""
    print("\n" + "="*70)
    print("Example 5: Interface Specification (like gymnasium Spaces)")
    print("="*70)
    
    # Define an input specification
    def define_input(name: str, msg_type_or_dist):
        """Mock function showing how to use type OR dist specification."""
        if isinstance(msg_type_or_dist, type):
            print(f"\nInput '{name}': {msg_type_or_dist.__name__} (no validation)")
        else:
            print(f"\nInput '{name}': {type(msg_type_or_dist).__name__} with bounds")
            bounds = msg_type_or_dist.get_range()
            print(f"  Bounds: {bounds}")
    
    # Option 1: Just the type (permissive)
    define_input("target_pose", PoseStamped)
    
    # Option 2: With distribution bounds (validated)
    define_input("target_pose", PoseStamped.Dist.uniform(-1.0, 1.0))
    
    # Option 3: Custom bounds per field
    custom_spec = Point.Dist(
        x=FloatDist.uniform(0.0, 1.0),
        y=FloatDist.uniform(0.0, 1.0),
        z=FloatDist.uniform(0.5, 1.5)
    )
    define_input("workspace_point", custom_spec)
    
    # Validate inputs against spec
    print("\n\nValidation:")
    spec = Point.Dist.uniform(0.0, 1.0)
    
    valid_point = Point(x=0.5, y=0.5, z=0.5)
    invalid_point = Point(x=2.0, y=0.0, z=0.0)
    
    print(f"Spec: Point.Dist.uniform(0, 1)")
    print(f"  {valid_point.to_list()} is valid: {spec.contains(valid_point)}")
    print(f"  {invalid_point.to_list()} is valid: {spec.contains(invalid_point)}")


def example_6_seeding_reproducibility():
    """Example 6: Reproducible sampling with seeds."""
    print("\n" + "="*70)
    print("Example 6: Seeding for Reproducibility")
    print("="*70)
    
    # Create two identical distributions
    dist1 = Point.Dist.uniform(-1.0, 1.0)
    dist2 = Point.Dist.uniform(-1.0, 1.0)
    
    # Seed them identically
    dist1.seed(42)
    dist2.seed(42)
    
    print("\nTwo distributions seeded with 42:")
    print("Dist 1 samples:")
    for i in range(3):
        point = dist1.sample()
        print(f"  {[f'{x:.3f}' for x in point.to_list()]}")
    
    print("\nDist 2 samples (should match):")
    for i in range(3):
        point = dist2.sample()
        print(f"  {[f'{x:.3f}' for x in point.to_list()]}")


if __name__ == '__main__':
    print("\n" + "="*70)
    print("DataDist Examples - Gymnasium-like Space API for Messages")
    print("="*70)
    
    example_1_basic_float_dist()
    example_2_point_dist()
    example_3_pose_stamped_dist()
    example_4_dataset_generation()
    example_5_interface_specification()
    example_6_seeding_reproducibility()
    
    print("\n" + "="*70)
    print("All examples completed!")
    print("="*70)
    print("\nKey takeaways:")
    print("  - Use Msg.Dist classes to define data distributions")
    print("  - Dist classes inherit from DataDist (gymnasium Space-like)")
    print("  - Use type OR Dist for interface specifications")
    print("  - Dist provides: sample(), contains(), get_range(), seed()")
    print("  - Generate test datasets with generate_dataset()")
    print("="*70 + "\n")




