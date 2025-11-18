#!/usr/bin/env python3

"""
    Test PinCollision obstacle functions.
    
    Tests box, cylinder, and mesh obstacle addition with collision detection.
"""

# BAM
from bam.descriptions import UR
from bam.utils.pin_utils import PinCollision, PinRobotModel

# PYTHON
import pytest
import numpy as np
import pinocchio as pin
import os

VIZ = False
    

@pytest.fixture(scope="module")
def arm():
    """Create UR5e robot description (shared across all tests)."""
    return UR.make_UR5e()

@pytest.fixture
def pin_collision(arm) -> PinCollision:
    """Create PinCollision instance from UR5e robot."""
    collision = PinCollision.from_robot_description(arm, verbose=False)
    return collision

def test_box_obstacle_collision_detected(pin_collision: PinCollision) -> None:
    """Test that a box obstacle on top of the robot arm is detected in collision."""
    # Get neutral config (robot stretched horizontally along x-axis)
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Place a box on the robot arm (middle of forearm at x=0.4, z=0.163)
    box_pose = np.eye(4)
    box_pose[0, 3] = 0.4  # Middle of the arm
    box_pose[2, 3] = 0.163  # Height of the arm
    
    # Add box obstacle (check against all robot links)
    pin_collision.add_box_obstacle(
        name="test_box",
        size=0.3,  # 30cm cube
        pose=box_pose
    )
    
    # Check collision at neutral position
    assert pin_collision.is_collision(q_neutral), "Box should collide with robot in neutral position"

def test_box_obstacle_no_collision_when_disabled(pin_collision: PinCollision) -> None:
    """Test that disabling collision pairs prevents collision detection."""
    # Get neutral config
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Place a box on the robot arm
    box_pose = np.eye(4)
    box_pose[0, 3] = 0.4
    box_pose[2, 3] = 0.163
    
    # Add box but with empty check_link_names (no collision pairs)
    pin_collision.add_box_obstacle(
        name="test_box_disabled",
        size=0.3,
        pose=box_pose,
        check_link_names=[]  # Don't check collision with any links
    )
    
    # Should NOT detect collision since we disabled all pairs
    assert not pin_collision.is_collision(q_neutral), "Box should NOT collide when collision pairs disabled"

def test_cylinder_obstacle_collision_detected(pin_collision: PinCollision) -> None:
    """Test that a cylinder obstacle on top of the robot arm is detected in collision."""
    # Get neutral config
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Place a cylinder on the robot arm
    cylinder_pose = np.eye(4)
    cylinder_pose[0, 3] = 0.6  # Near wrist
    cylinder_pose[2, 3] = 0.163
    
    # Add cylinder obstacle
    pin_collision.add_cylinder_obstacle(
        name="test_cylinder",
        radius=0.15,  # 15cm radius
        height=0.3,   # 30cm height
        pose=cylinder_pose
    )
    
    # Check collision
    assert pin_collision.is_collision(q_neutral), "Cylinder should collide with robot in neutral position"

def test_cylinder_obstacle_no_collision_when_disabled(pin_collision: PinCollision) -> None:
    """Test that disabling collision pairs prevents cylinder collision detection."""
    # Get neutral config
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Place a cylinder on the robot arm
    cylinder_pose = np.eye(4)
    cylinder_pose[0, 3] = 0.6
    cylinder_pose[2, 3] = 0.163
    
    # Add cylinder with no collision pairs
    pin_collision.add_cylinder_obstacle(
        name="test_cylinder_disabled",
        radius=0.15,
        height=0.3,
        pose=cylinder_pose,
        check_link_names=[]
    )
    
    # Should NOT detect collision
    assert not pin_collision.is_collision(q_neutral), "Cylinder should NOT collide when collision pairs disabled"

def test_mesh_obstacle_collision_detected(pin_collision: PinCollision) -> None:
    """Test that a mesh obstacle on top of the robot arm is detected in collision."""
    # Get neutral config
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Place a mesh on the robot arm
    mesh_pose = np.eye(4)
    mesh_pose[0, 3] = 0.4  # Middle of forearm
    mesh_pose[2, 3] = 0.163
    
    # Use a UR5 link mesh which is appropriately sized
    mesh_path = os.path.join(os.path.dirname(__file__), "mesh_for_testing.stl")
    
    # Skip test if mesh file doesn't exist
    if not os.path.exists(mesh_path):
        pytest.skip(f"Mesh file not found: {mesh_path}")
    
    # Add mesh obstacle (use normal scale since it's a UR5 mesh)
    pin_collision.add_mesh_obstacle(
        name="mesh_for_testing",
        mesh_path=mesh_path,
        pose=mesh_pose,
        scale=(1.0, 1.0, 1.0)  # Full size
    )
    
    # Check collision
    assert pin_collision.is_collision(q_neutral), "Mesh should collide with robot in neutral position"

def test_mesh_obstacle_no_collision_when_disabled(pin_collision: PinCollision) -> None:
    """Test that disabling collision pairs prevents mesh collision detection."""
    # Get neutral config
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Place a mesh on the robot arm
    mesh_pose = np.eye(4)
    mesh_pose[0, 3] = 0.4
    mesh_pose[2, 3] = 0.163
    
    # Use a UR5 link mesh
    mesh_path = os.path.join(os.path.dirname(__file__), "mesh_for_testing.stl")
    
    # Skip test if mesh file doesn't exist
    if not os.path.exists(mesh_path):
        pytest.skip(f"Mesh file not found: {mesh_path}")
    
    # Add mesh with no collision pairs
    pin_collision.add_mesh_obstacle(
        name="test_mesh_disabled",
        mesh_path=mesh_path,
        pose=mesh_pose,
        scale=(1.0, 1.0, 1.0),
        check_link_names=[]
    )
    
    # Should NOT detect collision
    assert not pin_collision.is_collision(q_neutral), "Mesh should NOT collide when collision pairs disabled"

def test_all_obstacles_together(pin_collision: PinCollision) -> None:
    """Test adding box, cylinder, and mesh obstacles simultaneously."""
    # Get neutral config
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Add box
    box_pose = np.eye(4)
    box_pose[0, 3] = 0.3
    box_pose[2, 3] = 0.163
    pin_collision.add_box_obstacle("box", 0.2, box_pose)
    
    # Add cylinder
    cylinder_pose = np.eye(4)
    cylinder_pose[0, 3] = 0.5
    cylinder_pose[2, 3] = 0.163
    pin_collision.add_cylinder_obstacle("cylinder", 0.1, 0.2, cylinder_pose)
    
    # Add mesh
    mesh_path = os.path.join(os.path.dirname(__file__), "mesh_for_testing.stl")
    if os.path.exists(mesh_path):
        mesh_pose = np.eye(4)
        mesh_pose[0, 3] = 0.7
        mesh_pose[2, 3] = 0.163
        pin_collision.add_mesh_obstacle("mesh", mesh_path, mesh_pose, scale=(1.0, 1.0, 1.0))
    
    # At least one should be in collision
    assert pin_collision.is_collision(q_neutral), "At least one obstacle should collide with robot"

def test_rectangular_box_obstacle(pin_collision: PinCollision) -> None:
    """Test adding a non-cubic box (rectangular)."""
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Add rectangular box
    box_pose = np.eye(4)
    box_pose[0, 3] = 0.5
    box_pose[2, 3] = 0.163
    pin_collision.add_box_obstacle(
        name="rect_box",
        size=(0.5, 0.3, 0.2),  # Different dimensions
        pose=box_pose
    )
    
    # Check collision
    assert pin_collision.is_collision(q_neutral), "Rectangular box should collide with robot"

def test_update_obstacle_pose(pin_collision: PinCollision) -> None:
    """Test updating obstacle pose after creation."""
    q_neutral = pin_collision.robot_model.get_q_neutral()
    
    # Add box far away (no collision)
    box_pose = np.eye(4)
    box_pose[0, 3] = 5.0  # 5 meters away in x
    pin_collision.add_box_obstacle("movable_box", 0.2, box_pose)
    
    # Should not collide when far away
    assert not pin_collision.is_collision(q_neutral), "Box should not collide when far away"
    
    # Move box to collide with robot
    collision_pose = np.eye(4)
    collision_pose[0, 3] = 0.4  # On the arm
    collision_pose[2, 3] = 0.163
    pin_collision.update_obstacle_pose("movable_box", collision_pose)
    
    # Should collide now
    assert pin_collision.is_collision(q_neutral), "Box should collide after moving to robot position"

def test_duplicate_obstacle_raises_error(pin_collision: PinCollision) -> None:
    """Test that adding an obstacle with duplicate name raises an error."""
    box_pose = np.eye(4)
    
    # Add first box
    pin_collision.add_box_obstacle("duplicate", 0.2, box_pose)
    
    # Try to add another with same name - should raise error
    with pytest.raises(ValueError, match="already exists"):
        pin_collision.add_box_obstacle("duplicate", 0.3, box_pose)

def test_update_nonexistent_obstacle_raises_error(pin_collision: PinCollision) -> None:
    """Test that updating a non-existent obstacle raises an error."""
    box_pose = np.eye(4)
    
    with pytest.raises(ValueError, match="not found"):
        pin_collision.update_obstacle_pose("nonexistent", box_pose)



if __name__ == "__main__":
    
    if False:
        pass
        # BUG you need a new collision for each tests...
        # Create pin_collision fixture instance
        # arm = UR.make_UR5e()
        # collision = PinCollision.from_robot_description(arm, verbose=False)
        
        # Basic obstacle tests
        # test_box_obstacle_collision_detected(collision)
        # test_box_obstacle_no_collision_when_disabled(collision)
        # test_cylinder_obstacle_collision_detected(collision)
        # test_cylinder_obstacle_no_collision_when_disabled(collision)
        # test_mesh_obstacle_collision_detected(collision)
        # test_mesh_obstacle_no_collision_when_disabled(collision)
        # test_all_obstacles_together(collision)
        # test_rectangular_box_obstacle(collision)
        # test_update_obstacle_pose(collision)
        # test_duplicate_obstacle_raises_error(collision)
        # test_update_nonexistent_obstacle_raises_error(collision)
  
    else:
        pytest.main([__file__, "-v"])