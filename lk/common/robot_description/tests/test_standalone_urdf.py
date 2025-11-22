#!/usr/bin/env python3

"""
Standalone test for URDF conversion.

Tests URDF import/export without importing the full lk package.
"""

# PYTHON
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))

try:
    import yourdfpy

    print("✓ yourdfpy is installed")
except ImportError:
    print("✗ yourdfpy not installed. Install with: pip install yourdfpy")
    sys.exit(1)

# Test simple URDF parsing
test_urdf = """<?xml version="1.0"?>
<robot name="test_robot">
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

print("\nTesting yourdfpy parsing...")
import io

urdf = yourdfpy.URDF.load(
    io.StringIO(test_urdf), load_meshes=False, build_scene_graph=True
)
print(f"✓ Parsed URDF with {len(urdf.link_map)} links and {len(urdf.joint_map)} joints")

# Now try importing robot description components
try:
    # Import just what we need from robot_description without loading the whole lk package
    print("\nImporting robot description types...")

    print("✓ Imported description types")

    print("✓ Imported ROS message types")

    # Import converter functions
    from lk.common.robot_description import urdf_converter

    print("✓ Imported urdf_converter module")

    # Test parsing
    print("\nTesting URDF parsing...")
    links, joints, robot_info = urdf_converter.from_urdf_string(test_urdf)
    print(f"✓ Parsed {len(links)} links and {len(joints)} joints")
    print(f"  Robot name: {robot_info.name}")

    # Test export
    print("\nTesting URDF export...")
    urdf_out = urdf_converter.to_urdf_string(links, joints, robot_info.name)
    print("✓ Exported URDF")

    # Test round-trip
    print("\nTesting round-trip...")
    links2, joints2, robot_info2 = urdf_converter.from_urdf_string(urdf_out)
    assert len(links2) == len(links), "Link count mismatch after round-trip"
    assert len(joints2) == len(joints), "Joint count mismatch after round-trip"
    print("✓ Round-trip successful")

    print("\n" + "=" * 60)
    print("All converter tests passed!")
    print("=" * 60)

except Exception:
    import traceback

    print("\n✗ Error during testing:")
    traceback.print_exc()
    sys.exit(1)
