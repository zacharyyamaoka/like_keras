#!/usr/bin/env python3

"""
    Round-trip tests using robot_descriptions package.
    
    Tests URDF conversion on real robot URDFs from robot_descriptions package.
    Validates that parsing and re-exporting produces equivalent URDF structures.
"""

# BAM
from lk.common.robot_description import RobotDescription

# PYTHON
import pytest
from pathlib import Path

try:
    import yourdfpy
    import robot_descriptions
except ImportError:
    pytest.skip("yourdfpy or robot_descriptions not installed", allow_module_level=True)


# List of robot URDF paths to test
# We'll try to get these from robot_descriptions package
def get_test_urdfs() -> list[tuple[str, str]]:
    """Get list of (name, urdf_path) tuples for testing.
    
    Returns:
        List of tuples with robot name and URDF path
    """
    test_robots = []
    
    # Try to import various robot descriptions
    try:
        from robot_descriptions import ur10_description
        if hasattr(ur10_description, 'URDF_PATH'):
            test_robots.append(("ur10", ur10_description.URDF_PATH))
    except (ImportError, AttributeError):
        pass
    
    try:
        from robot_descriptions import panda_description
        if hasattr(panda_description, 'URDF_PATH'):
            test_robots.append(("panda", panda_description.URDF_PATH))
    except (ImportError, AttributeError):
        pass
    
    try:
        from robot_descriptions import iiwa_description
        if hasattr(iiwa_description, 'URDF_PATH'):
            test_robots.append(("iiwa", iiwa_description.URDF_PATH))
    except (ImportError, AttributeError):
        pass
    
    try:
        from robot_descriptions import ur5_description
        if hasattr(ur5_description, 'URDF_PATH'):
            test_robots.append(("ur5", ur5_description.URDF_PATH))
    except (ImportError, AttributeError):
        pass
    
    try:
        from robot_descriptions import kinova_gen3_description
        if hasattr(kinova_gen3_description, 'URDF_PATH'):
            test_robots.append(("kinova_gen3", kinova_gen3_description.URDF_PATH))
    except (ImportError, AttributeError):
        pass
    
    try:
        from robot_descriptions import fetch_description
        if hasattr(fetch_description, 'URDF_PATH'):
            test_robots.append(("fetch", fetch_description.URDF_PATH))
    except (ImportError, AttributeError):
        pass
    
    return test_robots


@pytest.mark.parametrize("robot_name,urdf_path", get_test_urdfs())
def test_round_trip_real_robots(robot_name, urdf_path):
    """Test round-trip conversion on real robot URDFs.
    
    Args:
        robot_name: Name of the robot
        urdf_path: Path to URDF file
    """
    print(f"\nTesting {robot_name} from {urdf_path}")
    
    # Parse original URDF using yourdfpy directly
    try:
        original_urdf = yourdfpy.URDF.load(urdf_path, load_meshes=False, build_scene_graph=True)
    except Exception as e:
        pytest.skip(f"Could not parse {robot_name} with yourdfpy: {e}")
    
    # Load into RobotDescription
    try:
        robot_desc = RobotDescription.from_urdf_file(urdf_path, robot_name=robot_name)
    except Exception as e:
        pytest.fail(f"Failed to load {robot_name} into RobotDescription: {e}")
    
    # Verify basic structure
    assert robot_desc.info.name == robot_name
    assert len(robot_desc.links.entities) > 0, f"{robot_name} has no links"
    assert len(robot_desc.joints.entities) >= 0, f"{robot_name} has no joints"
    
    print(f"  Loaded: {len(robot_desc.links.entities)} links, {len(robot_desc.joints.entities)} joints")
    
    # Export to URDF string
    try:
        exported_urdf_str = robot_desc.to_urdf_xml()
    except Exception as e:
        pytest.fail(f"Failed to export {robot_name} to URDF: {e}")
    
    # Parse exported URDF back with yourdfpy
    try:
        exported_urdf = yourdfpy.URDF.load_from_xml_string(exported_urdf_str)
    except Exception as e:
        # Save problematic URDF for debugging
        debug_path = f"/tmp/{robot_name}_exported.urdf"
        with open(debug_path, 'w') as f:
            f.write(exported_urdf_str)
        pytest.fail(f"Failed to parse exported {robot_name} URDF with yourdfpy: {e}\nSaved to {debug_path}")
    
    # Compare structure
    assert len(original_urdf.link_map) == len(exported_urdf.link_map), \
        f"Link count mismatch: original={len(original_urdf.link_map)}, exported={len(exported_urdf.link_map)}"
    
    assert len(original_urdf.joint_map) == len(exported_urdf.joint_map), \
        f"Joint count mismatch: original={len(original_urdf.joint_map)}, exported={len(exported_urdf.joint_map)}"
    
    # Check all links are present
    for link_name in original_urdf.link_map:
        assert link_name in exported_urdf.link_map, f"Link {link_name} missing in exported URDF"
    
    # Check all joints are present
    for joint_name in original_urdf.joint_map:
        assert joint_name in exported_urdf.joint_map, f"Joint {joint_name} missing in exported URDF"
    
    # Compare joint properties
    for joint_name in original_urdf.joint_map:
        orig_joint = original_urdf.joint_map[joint_name]
        exp_joint = exported_urdf.joint_map[joint_name]
        
        # Joint type
        assert orig_joint.type == exp_joint.type, \
            f"Joint {joint_name} type mismatch: {orig_joint.type} != {exp_joint.type}"
        
        # Parent/child links
        assert orig_joint.parent == exp_joint.parent, \
            f"Joint {joint_name} parent mismatch: {orig_joint.parent} != {exp_joint.parent}"
        assert orig_joint.child == exp_joint.child, \
            f"Joint {joint_name} child mismatch: {orig_joint.child} != {exp_joint.child}"
    
    print(f"  ✓ Round-trip successful for {robot_name}")


def test_individual_robot_ur10():
    """Individual test for UR10 robot (if available)."""
    try:
        from robot_descriptions import ur10_description
        urdf_path = ur10_description.URDF_PATH
    except (ImportError, AttributeError):
        pytest.skip("UR10 description not available")
    
    robot_desc = RobotDescription.from_urdf_file(urdf_path, robot_name="ur10")
    
    # Check UR10 specific properties
    assert "shoulder_pan_joint" in robot_desc.joints.entities
    assert "wrist_3_link" in robot_desc.links.entities
    
    # Export and re-import
    urdf_str = robot_desc.to_urdf_xml()
    robot_desc2 = RobotDescription.from_urdf_xml(urdf_str, robot_name="ur10")
    
    # Verify structure matches
    assert len(robot_desc2.links.entities) == len(robot_desc.links.entities)
    assert len(robot_desc2.joints.entities) == len(robot_desc.joints.entities)


def test_individual_robot_panda():
    """Individual test for Panda robot (if available)."""
    try:
        from robot_descriptions import panda_description
        urdf_path = panda_description.URDF_PATH
    except (ImportError, AttributeError):
        pytest.skip("Panda description not available")
    
    robot_desc = RobotDescription.from_urdf_file(urdf_path, robot_name="panda")
    
    # Check Panda specific properties (typically 7 DOF arm)
    actuated_joints = robot_desc.joints.get_actuated_joints()
    assert len(actuated_joints) >= 7, "Panda should have at least 7 actuated joints"
    
    # Export and re-import
    urdf_str = robot_desc.to_urdf_xml()
    robot_desc2 = RobotDescription.from_urdf_xml(urdf_str, robot_name="panda")
    
    # Verify structure matches
    assert len(robot_desc2.links.entities) == len(robot_desc.links.entities)
    assert len(robot_desc2.joints.entities) == len(robot_desc.joints.entities)


def test_urdf_preservation():
    """Test that specific URDF features are preserved through round-trip."""
    urdf_xml = """<?xml version="1.0"?>
<robot name="preservation_test">
  <link name="base_link">
    <inertial>
      <origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.01" ixz="0.02" iyy="0.2" iyz="0.03" izz="0.3"/>
    </inertial>
    <visual>
      <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.5" length="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.5" length="0.1"/>
      </geometry>
    </collision>
  </link>
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100.0" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>
</robot>"""
    
    # Load
    robot_desc = RobotDescription.from_urdf_xml(urdf_xml)
    
    # Check preserved properties
    base_link = robot_desc.links.entities["base_link"]
    assert abs(base_link.inertial.mass - 5.0) < 1e-5
    assert abs(base_link.inertial.origin.xyz[0] - 0.1) < 1e-5
    assert isinstance(base_link.visual.geometry, Cylinder)
    
    joint1 = robot_desc.joints.entities["joint1"]
    assert abs(joint1.limits.min_position - (-3.14159)) < 1e-5
    assert abs(joint1.limits.max_position - 3.14159) < 1e-5
    assert abs(joint1.physics.damping - 0.5) < 1e-5
    assert abs(joint1.physics.friction - 0.1) < 1e-5
    
    # Export
    exported = robot_desc.to_urdf_xml()
    
    # Re-import
    robot_desc2 = RobotDescription.from_urdf_xml(exported)
    
    # Verify properties preserved
    base_link2 = robot_desc2.links.entities["base_link"]
    assert abs(base_link2.inertial.mass - 5.0) < 1e-5
    
    joint1_2 = robot_desc2.joints.entities["joint1"]
    assert abs(joint1_2.limits.min_position - (-3.14159)) < 1e-5
    assert abs(joint1_2.physics.damping - 0.5) < 1e-5


if __name__ == "__main__":
    # Run manual tests
    print("Testing URDF round-trip with robot_descriptions package...\n")
    
    test_urdfs = get_test_urdfs()
    
    if not test_urdfs:
        print("⚠ No robot URDFs found. Install robot_descriptions package.")
        print("  pip install robot_descriptions")
    else:
        print(f"Found {len(test_urdfs)} robot URDFs to test:\n")
        for name, path in test_urdfs:
            print(f"  - {name}: {path}")
        
        print("\nRunning round-trip tests...\n")
        for name, path in test_urdfs:
            try:
                test_round_trip_real_robots(name, path)
            except Exception as e:
                print(f"  ✗ {name} failed: {e}")
    
    print("\nTesting specific preservation...")
    test_urdf_preservation()
    print("  ✓ URDF feature preservation")
    
    print("\nAll manual tests completed!")




