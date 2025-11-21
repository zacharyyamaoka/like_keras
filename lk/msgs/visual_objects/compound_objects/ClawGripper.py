#!/usr/bin/env python3

"""
ClawGripper - Compound visual object for claw-style grippers.

Creates a claw gripper model with rotating fingers around a servo.
Useful for visualizing servo-based grippers, claw mechanisms, and angular grasps.

Example:
    import bam.msgs.visual_objects as viz
    from bam_artist import Artist
    from bam.msgs import PoseStamped
    import numpy as np

    # Create closed claw gripper
    gripper_closed = viz.ClawGripper(
        name="claw_closed",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0, 0, 0.1], [0, 0, 0]),
        opening_angle=np.pi/6,  # 30 degrees
        box_props=viz.Box(color=viz.RGBA.red())
    )

    # Create open claw gripper
    gripper_open = viz.ClawGripper(
        name="claw_open",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, 0, 0.1], [0, 0, 0]),
        opening_angle=np.pi/3,  # 60 degrees
        box_props=viz.Box(color=viz.RGBA.green())
    )

    artist.draw(gripper_closed)
    artist.draw(gripper_open)
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from ..markers import Box, Cylinder
from ..Frame import Frame
from ..RGBA import RGBA
from ..Material import Material
from bam.msgs import PoseStamped, PoseType

# PYTHON
from dataclasses import dataclass, field
import copy
import numpy as np
from typing import Literal, Optional
from tf_transformations import xyzrpy_to_matrix, xyzrpy_offset


@dataclass
class ClawGripper(CompoundVisualObject):
    """Claw-style gripper visualization with rotating fingers.

    Creates a gripper model with:
    - Cylindrical servo/pivot (visualized as sphere)
    - Two fingers rotating around the servo center
    - Optional coordinate frames showing TCP and virtual tip

    The fingers rotate symmetrically around the opening angle.

    Note: name and T_world_to_tcp are optional - useful for creating templates
    that will be cloned and positioned later.
    """

    # Data
    T_world_to_tcp: PoseType = field(default_factory=PoseStamped)
    opening_angle: float = np.pi / 4  # Angle between fingers in radians

    # Geometry configuration
    servo_diameter: float = 0.05
    finger_length: float = 0.12
    finger_thickness: float = 0.01
    finger_width: float = 0.02
    wrist_offset: float = 0.0

    # Visual properties
    box_props: Box = field(default_factory=lambda: Box(color=RGBA.red()))

    # Gripper options
    frame_scale: float = 0.025  # Size of coordinate frame (0 to disable)
    show_frames: bool = True  # Show TCP, claw center, and virtual tip frames
    z_into_table: bool = False  # If True, fingers point down
    opening_axis: Literal["x", "y"] = "x"  # Axis along which gripper opens

    def __post_init__(self):
        super().__post_init__()
        self.build_model()

    def build_model(self):
        """Build the gripper model geometry based on current parameters."""
        super().build_model()

        # Get z direction
        z_dir = -1 if self.z_into_table else 1

        # Get base transform (with optional rotation for opening axis)
        T_world_to_tcp = self.T_world_to_tcp.to_matrix()
        if self.opening_axis == "y":
            T_world_to_tcp = xyzrpy_offset(
                T_world_to_tcp, rpy=(0, 0, np.pi / 2), local=True
            )

        # Get claw center transform (offset along finger length, rotated 90deg)
        T_claw_center = xyzrpy_offset(
            T_world_to_tcp,
            xyz=(0, 0, z_dir * self.finger_length),
            rpy=(np.pi / 2, 0, 0),
            local=True,
        )

        # Create servo (cylinder oriented along z-axis)
        servo_pose = PoseStamped.from_matrix(T_claw_center)
        servo = Cylinder(
            name=f"{self.visual_id}/servo",
            pose=servo_pose,
            radius=self.servo_diameter / 2,
            height=self.finger_width * 0.8,
            color=copy.deepcopy(self.box_props.color),
        )
        self.add(servo)

        # Create finger 1 (no rotation)
        T_finger_1 = xyzrpy_offset(
            T_claw_center,
            xyz=(self.finger_thickness / 2, -z_dir * self.finger_length / 2, 0),
            local=True,
        )
        finger_1 = self._create_box(
            f"{self.visual_id}/finger_1",
            T_finger_1,
            [self.finger_thickness, self.finger_length, self.finger_width],
        )
        self.add(finger_1)

        # Create finger 2 (rotated by opening_angle)
        T_claw_center_rot = xyzrpy_offset(
            T_claw_center, rpy=(0, 0, -z_dir * self.opening_angle), local=True
        )
        T_finger_2 = xyzrpy_offset(
            T_claw_center_rot,
            xyz=(-self.finger_thickness / 2, -z_dir * self.finger_length / 2, 0),
            local=True,
        )
        finger_2 = self._create_box(
            f"{self.visual_id}/finger_2",
            T_finger_2,
            [self.finger_thickness, self.finger_length, self.finger_width],
        )
        self.add(finger_2)

        # Add coordinate frames if enabled
        if self.frame_scale > 0.0 and self.show_frames:
            # Claw center frame
            claw_frame = Frame(
                name=f"{self.visual_id}/claw_center_frame",
                transform=PoseStamped.from_matrix(T_claw_center),
                axis_length=self.frame_scale,
            )
            self.add(claw_frame)

            # Virtual tip frame (midpoint between finger tips)
            T_claw_center_half_rot = xyzrpy_offset(
                T_claw_center, rpy=(0, 0, -z_dir * self.opening_angle / 2), local=True
            )
            T_virtual_tip = xyzrpy_offset(
                T_claw_center_half_rot,
                xyz=(0, -z_dir * self.finger_length, 0),
                rpy=(-np.pi / 2, 0, 0),
                local=True,
            )
            virtual_tip_frame = Frame(
                name=f"{self.visual_id}/virtual_tip_frame",
                transform=PoseStamped.from_matrix(T_virtual_tip),
                axis_length=self.frame_scale,
            )
            self.add(virtual_tip_frame)

            # TCP frame
            tcp_frame = Frame(
                name=f"{self.visual_id}/tcp_frame",
                transform=self.T_world_to_tcp,
                axis_length=self.frame_scale,
            )
            self.add(tcp_frame)

    def _create_box(self, name: str, transform: np.ndarray, scale: list[float]) -> Box:
        """Create a box with the specified visual properties."""
        box = copy.deepcopy(self.box_props)
        box.name = name
        box.pose = PoseStamped.from_matrix(transform)
        box.scale = scale
        return box

    @classmethod
    def make_skeleton(
        cls,
        name: str = "",
        T_world_to_tcp: Optional[PoseType] = None,
        opening_angle: float = np.pi / 4,
        thickness_scale: float = 1.0,
        box_props: Optional[Box] = None,
        frame_scale: float = 0.025,
        show_frames: bool = True,
        z_into_table: bool = False,
        opening_axis: Literal["x", "y"] = "x",
    ) -> "ClawGripper":
        """Create a thin skeleton claw gripper for lightweight visualization.

        Args:
            name: Name for the gripper (auto-generated if empty)
            T_world_to_tcp: Transform from world to gripper TCP (defaults to origin if None)
            opening_angle: Angle between fingers in radians
            thickness_scale: Scale factor for thickness (default 1.0)
            box_props: Optional Box template for visual properties
            frame_scale: Size of coordinate frame (0 to disable)
            show_frames: Show TCP, claw center, and virtual tip frames
            z_into_table: If True, fingers point down
            opening_axis: Axis along which gripper opens

        Returns:
            ClawGripper with thin skeleton geometry
        """
        # Use provided template or create default
        if box_props is None:
            box_props = Box(color=RGBA.red())

        if T_world_to_tcp is None:
            T_world_to_tcp = PoseStamped()

        return cls(
            name=name,
            T_world_to_tcp=T_world_to_tcp,
            opening_angle=opening_angle,
            servo_diameter=thickness_scale * 0.025,
            finger_length=thickness_scale * 0.08,
            finger_thickness=thickness_scale * 0.0025,
            finger_width=thickness_scale * 0.0025,
            box_props=box_props,
            frame_scale=frame_scale,
            show_frames=show_frames,
            z_into_table=z_into_table,
            opening_axis=opening_axis,
        )


if __name__ == "__main__":
    from bam.msgs import PoseStamped

    # Example 1: Closed claw gripper
    print("Example 1: Closed claw gripper (30°)")
    gripper1 = ClawGripper(
        name="claw_closed",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0, 0, 0.1], [0, 0, 0]),
        opening_angle=np.pi / 6,  # 30 degrees
        box_props=Box(color=RGBA.red()),
    )
    print(f"  Created with {len(gripper1.children)} children")
    print(f"  Opening angle: {np.degrees(gripper1.opening_angle):.1f}°")

    # Example 2: Open claw gripper
    print("\nExample 2: Open claw gripper (60°)")
    gripper2 = ClawGripper(
        name="claw_open",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, 0, 0.1], [0, 0, 0]),
        opening_angle=np.pi / 3,  # 60 degrees
        box_props=Box(color=RGBA.green()),
    )
    print(f"  Created with {len(gripper2.children)} children")
    print(f"  Opening angle: {np.degrees(gripper2.opening_angle):.1f}°")

    # Example 3: Wide open claw
    print("\nExample 3: Wide open claw (90°)")
    gripper3 = ClawGripper(
        name="claw_wide",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0, 0.2, 0.1], [0, 0, 0]),
        opening_angle=np.pi / 2,  # 90 degrees
        box_props=Box(color=RGBA.blue()),
    )
    print(f"  Opening angle: {np.degrees(gripper3.opening_angle):.1f}°")

    # Example 4: Y-axis opening claw
    print("\nExample 4: Y-axis opening claw")
    gripper4 = ClawGripper(
        name="claw_y_axis",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0, -0.2, 0.1], [0, 0, 0]),
        opening_angle=np.pi / 4,
        opening_axis="y",
        box_props=Box(color=RGBA.cyan()),
    )
    print(f"  Created with opening_axis: {gripper4.opening_axis}")

    # Example 5: Claw pointing down
    print("\nExample 5: Claw pointing down (z_into_table=True)")
    gripper5 = ClawGripper(
        name="claw_down",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, 0.2, 0.1], [0, 0, 0]),
        opening_angle=np.pi / 4,
        z_into_table=True,
        box_props=Box(color=RGBA.magenta()),
    )
    print(f"  Created with z_into_table: {gripper5.z_into_table}")

    # Example 6: Custom geometry
    print("\nExample 6: Custom geometry")
    gripper6 = ClawGripper(
        name="claw_custom",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, -0.2, 0.1], [0, 0, 0]),
        opening_angle=np.pi / 4,
        servo_diameter=0.06,
        finger_length=0.15,
        finger_thickness=0.012,
        finger_width=0.025,
        box_props=Box(color=RGBA.yellow()),
    )
    print(f"  Created with custom geometry: finger_length={gripper6.finger_length}m")

    # Example 7: No frames
    print("\nExample 7: Claw without frames")
    gripper7 = ClawGripper(
        name="claw_no_frames",
        T_world_to_tcp=PoseStamped.from_xyzrpy([-0.2, 0, 0.1], [0, 0, 0]),
        opening_angle=np.pi / 4,
        show_frames=False,
        box_props=Box(color=RGBA.orange()),
    )
    print(f"  Created with show_frames: {gripper7.show_frames}")
    print(f"  Children count: {len(gripper7.children)} (no frames)")

    print("\n✓ All examples created successfully!")
