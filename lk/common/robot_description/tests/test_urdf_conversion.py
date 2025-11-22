#!/usr/bin/env python3

"""
Tests for URDF conversion functionality.

Tests the ability to:
- Parse URDF files into RobotDescription
- Export RobotDescription to URDF
- Round-trip conversion (URDF -> RobotDescription -> URDF)
"""

# BAM
import tempfile
from pathlib import Path

# PYTHON
import pytest
from bam.msgs.ros_msgs import TransformStamped, Vector3

from lk.common.robot_description import RobotDescription
from lk.common.robot_description.description_types import (
    Box,
    Cylinder,
    Inertia,
    InertialProperties,
    JointDescription,
    LinkDescription,
    PerJointLimits,
    Sphere,
)

try:
    import yourdfpy
except ImportError:
    pytest.skip("yourdfpy not installed", allow_module_level=True)


def test_simple_urdf_parsing():
    """Test parsing a simple URDF string."""
    urdf_xml = """<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>"""

    robot_desc = RobotDescription.from_urdf_xml(urdf_xml)

    assert robot_desc.info.name == "simple_robot"
    assert len(robot_desc.links.entities) == 2
    assert len(robot_desc.joints.entities) == 1

    # Check link properties
    base_link = robot_desc.links.entities["base_link"]
    assert base_link.name == "base_link"
    assert base_link.inertial.mass == 1.0

    # Check joint properties
    joint1 = robot_desc.joints.entities["joint1"]
    assert joint1.name == "joint1"
    assert joint1.type == "revolute"
    assert joint1.parent_link == "base_link"
    assert joint1.child_link == "link1"
    assert joint1.transform.xyz == [0.0, 0.0, 0.1]


def test_urdf_export():
    """Test exporting RobotDescription to URDF."""
    # Create a simple robot description
    base_link = LinkDescription(name="base_link")
    base_link.inertial = InertialProperties(
        mass=1.0, inertia=Inertia(ixx=0.01, iyy=0.01, izz=0.01)
    )

    link1 = LinkDescription(name="link1")

    joint1 = JointDescription(name="joint1", type="revolute")
    joint1.transform = TransformStamped()
    joint1.transform.header.frame_id = "base_link"
    joint1.transform.child_frame_id = "link1"
    joint1.transform.xyz = [0.0, 0.0, 0.1]
    joint1.transform.rpy = [0.0, 0.0, 0.0]
    joint1.axis = Vector3(x=0.0, y=0.0, z=1.0)
    joint1.limits = PerJointLimits(
        min_position=-1.57, max_position=1.57, max_effort=10.0, max_velocity=1.0
    )

    robot_desc = RobotDescription.from_entities(
        [base_link, link1, joint1], robot_info=None
    )
    robot_desc.info.name = "test_robot"

    # Export to URDF
    urdf_xml = robot_desc.to_urdf_xml()

    # Verify URDF is valid XML
    assert '<?xml version="1.0"?>' in urdf_xml
    assert '<robot name="test_robot">' in urdf_xml
    assert '<link name="base_link">' in urdf_xml
    assert '<joint name="joint1" type="revolute">' in urdf_xml
    assert "</robot>" in urdf_xml


def test_round_trip_simple():
    """Test round-trip conversion: URDF -> RobotDescription -> URDF."""
    original_urdf = """<?xml version="1.0"?>
<robot name="round_trip_test">
  <link name="base_link">
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50.0" velocity="2.0"/>
  </joint>
</robot>"""

    # Parse original URDF
    robot_desc = RobotDescription.from_urdf_xml(original_urdf)

    # Export to URDF
    exported_urdf = robot_desc.to_urdf_xml()

    # Parse exported URDF
    robot_desc2 = RobotDescription.from_urdf_xml(exported_urdf)

    # Compare key properties
    assert robot_desc2.info.name == robot_desc.info.name
    assert len(robot_desc2.links.entities) == len(robot_desc.links.entities)
    assert len(robot_desc2.joints.entities) == len(robot_desc.joints.entities)

    # Check link properties match
    for link_name in robot_desc.links.entities:
        link1 = robot_desc.links.entities[link_name]
        link2 = robot_desc2.links.entities[link_name]
        assert abs(link1.inertial.mass - link2.inertial.mass) < 1e-5

    # Check joint properties match
    for joint_name in robot_desc.joints.entities:
        joint1 = robot_desc.joints.entities[joint_name]
        joint2 = robot_desc2.joints.entities[joint_name]
        assert joint1.type == joint2.type
        assert joint1.parent_link == joint2.parent_link
        assert joint1.child_link == joint2.child_link


def test_geometry_parsing():
    """Test parsing different geometry types."""
    urdf_xml = """<?xml version="1.0"?>
<robot name="geometry_test">
  <link name="box_link">
    <visual>
      <geometry>
        <box size="1.0 2.0 3.0"/>
      </geometry>
    </visual>
  </link>
  <link name="cylinder_link">
    <visual>
      <geometry>
        <cylinder radius="0.5" length="1.0"/>
      </geometry>
    </visual>
  </link>
  <link name="sphere_link">
    <visual>
      <geometry>
        <sphere radius="0.3"/>
      </geometry>
    </visual>
  </link>
</robot>"""

    robot_desc = RobotDescription.from_urdf_xml(urdf_xml)

    # Check box geometry
    box_link = robot_desc.links.entities["box_link"]
    assert isinstance(box_link.visual.geometry, Box)
    assert box_link.visual.geometry.size == [1.0, 2.0, 3.0]

    # Check cylinder geometry
    cylinder_link = robot_desc.links.entities["cylinder_link"]
    assert isinstance(cylinder_link.visual.geometry, Cylinder)
    assert cylinder_link.visual.geometry.radius == 0.5
    assert cylinder_link.visual.geometry.length == 1.0

    # Check sphere geometry
    sphere_link = robot_desc.links.entities["sphere_link"]
    assert isinstance(sphere_link.visual.geometry, Sphere)
    assert sphere_link.visual.geometry.radius == 0.3


def test_from_urdf_file():
    """Test loading URDF from file."""
    urdf_content = """<?xml version="1.0"?>
<robot name="file_test">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="fixed">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
</robot>"""

    # Create temporary file
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        f.write(urdf_content)
        temp_path = f.name

    try:
        # Load from file
        robot_desc = RobotDescription.from_urdf_file(temp_path)

        assert robot_desc.info.name == "file_test"
        assert len(robot_desc.links.entities) == 2
        assert len(robot_desc.joints.entities) == 1
    finally:
        # Clean up
        Path(temp_path).unlink()


def test_multi_visual_collision():
    """Test links with multiple visual/collision elements."""
    urdf_xml = """<?xml version="1.0"?>
<robot name="multi_test">
  <link name="multi_link">
    <visual name="visual1">
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <visual name="visual2">
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
    </visual>
    <collision name="collision1">
      <geometry>
        <cylinder radius="0.3" length="1.0"/>
      </geometry>
    </collision>
  </link>
</robot>"""

    robot_desc = RobotDescription.from_urdf_xml(urdf_xml)

    multi_link = robot_desc.links.entities["multi_link"]

    # Check multiple visuals
    assert isinstance(multi_link.visual, list)
    assert len(multi_link.visual) == 2
    assert isinstance(multi_link.visual[0].geometry, Box)
    assert isinstance(multi_link.visual[1].geometry, Sphere)

    # Check collision (single)
    assert isinstance(multi_link.collision.geometry, Cylinder)


def test_joint_types():
    """Test different joint types."""
    urdf_xml = """<?xml version="1.0"?>
<robot name="joint_types_test">
  <link name="base"/>
  <link name="revolute_link"/>
  <link name="prismatic_link"/>
  <link name="fixed_link"/>
  <link name="continuous_link"/>
  
  <joint name="revolute_joint" type="revolute">
    <parent link="base"/>
    <child link="revolute_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
  <joint name="prismatic_joint" type="prismatic">
    <parent link="base"/>
    <child link="prismatic_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="0.5"/>
  </joint>
  
  <joint name="fixed_joint" type="fixed">
    <parent link="base"/>
    <child link="fixed_link"/>
  </joint>
  
  <joint name="continuous_joint" type="continuous">
    <parent link="base"/>
    <child link="continuous_link"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>"""

    robot_desc = RobotDescription.from_urdf_xml(urdf_xml)

    # Check revolute joint
    rev_joint = robot_desc.joints.entities["revolute_joint"]
    assert rev_joint.type == "revolute"
    assert rev_joint.axis.x == 1.0
    assert rev_joint.is_actuated
    assert not rev_joint.is_fixed

    # Check prismatic joint
    pris_joint = robot_desc.joints.entities["prismatic_joint"]
    assert pris_joint.type == "prismatic"
    assert pris_joint.axis.z == 1.0

    # Check fixed joint
    fixed_joint = robot_desc.joints.entities["fixed_joint"]
    assert fixed_joint.type == "fixed"
    assert fixed_joint.is_fixed
    assert not fixed_joint.is_actuated

    # Check continuous joint
    cont_joint = robot_desc.joints.entities["continuous_joint"]
    assert cont_joint.type == "continuous"
    assert cont_joint.axis.y == 1.0


if __name__ == "__main__":
    # Run simple smoke tests
    print("Running basic URDF conversion tests...")

    test_simple_urdf_parsing()
    print("✓ Simple URDF parsing")

    test_urdf_export()
    print("✓ URDF export")

    test_round_trip_simple()
    print("✓ Round-trip conversion")

    test_geometry_parsing()
    print("✓ Geometry parsing")

    test_from_urdf_file()
    print("✓ File loading")

    test_multi_visual_collision()
    print("✓ Multi visual/collision")

    test_joint_types()
    print("✓ Joint types")

    print("\nAll tests passed!")
