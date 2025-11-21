#!/usr/bin/env python3

"""
Example: Using URDF Import/Export

This example demonstrates how to:
1. Load a URDF file into RobotDescription
2. Inspect the robot structure
3. Make modifications
4. Export back to URDF
"""

# BAM
# Note: This example requires fixing the lk package imports
# from lk.common.robot_description import RobotDescription

# PYTHON
from pathlib import Path


def example_urdf_import():
    """Example of importing a URDF file."""

    # Create a simple URDF for demonstration
    simple_urdf = """<?xml version="1.0"?>
<robot name="example_robot">
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
    </visual>
  </link>
  
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100.0" velocity="2.0"/>
  </joint>
  
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="ee_joint" type="revolute">
    <parent link="arm_link"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>
</robot>"""

    print("=" * 70)
    print("EXAMPLE: URDF Import/Export")
    print("=" * 70)

    print("\n1. Import from URDF string")
    print("-" * 70)
    # Uncomment when lk imports work:
    # robot_desc = RobotDescription.from_urdf_xml(simple_urdf)
    # print(f"   Robot name: {robot_desc.info.name}")
    # print(f"   Links: {len(robot_desc.links.entities)}")
    # print(f"   Joints: {len(robot_desc.joints.entities)}")
    print("   [Code example]")
    print("   robot_desc = RobotDescription.from_urdf_xml(urdf_string)")
    print("   print(f'Links: {len(robot_desc.links.entities)}')")
    print("   print(f'Joints: {len(robot_desc.joints.entities)}')")

    print("\n2. Inspect robot structure")
    print("-" * 70)
    print("   [Code example]")
    print("   # List all links")
    print("   for link_name in robot_desc.links.entities:")
    print("       link = robot_desc.links.entities[link_name]")
    print("       print(f'Link: {link.name}')")
    print("   ")
    print("   # List all joints")
    print("   for joint_name in robot_desc.joints.entities:")
    print("       joint = robot_desc.joints.entities[joint_name]")
    print("       print(f'Joint: {joint.name} ({joint.type})')")
    print("       print(f'  Parent: {joint.parent_link}')")
    print("       print(f'  Child: {joint.child_link}')")

    print("\n3. Make modifications")
    print("-" * 70)
    print("   [Code example]")
    print("   # Change robot name")
    print("   robot_desc.info.name = 'modified_robot'")
    print("   ")
    print("   # Modify joint limit")
    print("   joint = robot_desc.joints.entities['arm_joint']")
    print("   joint.limits.max_velocity = 3.0")

    print("\n4. Export to URDF")
    print("-" * 70)
    print("   [Code example]")
    print("   urdf_xml = robot_desc.to_urdf_xml()")
    print("   ")
    print("   # Save to file")
    print("   with open('modified_robot.urdf', 'w') as f:")
    print("       f.write(urdf_xml)")

    print("\n5. Load real robots (with robot_descriptions)")
    print("-" * 70)
    print("   [Code example]")
    print("   from robot_descriptions import ur10_description")
    print("   ")
    print("   robot_desc = RobotDescription.from_urdf_file(")
    print("       ur10_description.URDF_PATH,")
    print("       robot_name='ur10'")
    print("   )")
    print("   ")
    print("   # Get actuated joints")
    print("   actuated = robot_desc.joints.get_actuated_joints()")
    print("   print(f'Actuated joints: {len(actuated)}')")

    print("\n" + "=" * 70)
    print("KEY METHODS")
    print("=" * 70)
    print("RobotDescription.from_urdf_xml(urdf_string, robot_name=None)")
    print("RobotDescription.from_urdf_file(urdf_path, robot_name=None)")
    print("robot_desc.to_urdf_xml()")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    example_urdf_import()
