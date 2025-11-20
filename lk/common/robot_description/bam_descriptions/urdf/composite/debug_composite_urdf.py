#!/usr/bin/env python3

"""
    Debug Composite URDF Generation
    
    Saves the generated URDF to a file for inspection.
"""

# BAM
from bam_descriptions import UR, Composite, JointDescription
from bam_descriptions.hands import CrabClaw
from bam.msgs import TransformStamped

# PYTHON


print("Creating simple arm+hand composite...")
arm = UR.make_UR5e(prefix="arm_")
hand = CrabClaw.make_bam(prefix="hand_")

mount_joint = JointDescription(
    name="arm_to_hand_mount",
    type="fixed",
    transform=TransformStamped.from_xyzrpy(
        xyz=[0, 0, 0],
        rpy=[0, 0, 0],
        frame_id=arm.hand_mount,
        child_frame_id=hand.arm_mount
    )
)

composite = Composite(
    descriptions=[arm, hand],
    mounting_joints=[mount_joint],
    add_timestamp=True  # Force unique config to avoid reusing stale files
)

print(f"\nComposite created:")
print(f"  Config: {composite.config_file}")
print(f"  Macro: {composite.urdf.macro_path}")
print(f"  Xacro: {composite.urdf.xacro_path}")

print("\nGenerating URDF...")
urdf_string = composite.get_urdf_xml()

# Save to file
urdf_file = "/home/bam/bam_ws/src/bam_common/bam_descriptions/urdf/composite/debug_composite.urdf"
with open(urdf_file, 'w') as f:
    f.write(urdf_string)

print(f"Saved URDF to: {urdf_file}")
print(f"URDF length: {len(urdf_string)} characters")

# Check for common issues
print("\nURDF Validation:")
if "arm_base_link_ur" in urdf_string:
    count = urdf_string.count('<link name="arm_base_link_ur">')
    print(f"  arm_base_link_ur appears {count} time(s)")
    if count > 1:
        print("  ✗ ERROR: Duplicate link definition!")
    else:
        print("  ✓ Link is unique")

if "base_link" in urdf_string:
    count = urdf_string.count('<link name="base_link">')
    print(f"  base_link appears {count} time(s)")
    if count > 1:
        print("  ✗ ERROR: Duplicate link definition!")
    else:
        print("  ✓ Link is unique")

print(f"\nYou can inspect the URDF at: {urdf_file}")
print("To validate with urdf_parser:")
print(f"  check_urdf {urdf_file}")

