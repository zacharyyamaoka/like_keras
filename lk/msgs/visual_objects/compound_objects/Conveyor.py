#!/usr/bin/env python3

"""
Conveyor - Compound visual object for conveyor belts.

Creates a conveyor belt visualization with configurable geometry.
Useful for visualizing pick-and-place scenarios, material handling, and automation tasks.

The origin frame is positioned at the top surface, end center of the conveyor:
- Z axis points upward
- Y axis points forward off the conveyor
- X axis points to the right

Example:
    import bam.msgs.visual_objects as viz
    from bam_artist import Artist
    from bam.msgs import TransformStamped

    # Create basic conveyor
    conveyor = viz.Conveyor(
        name="conveyor",
        T_world_to_surface_center=TransformStamped.from_xyzrpy([0.5, 0, 0], [0, 0, 0]),
        size=(0.4, 2.0, 0.05)
    )

    artist.draw(conveyor)
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from ..markers import Box
from ..Frame import Frame
from ..RGBA import RGBA
from ..Material import Material
from bam.msgs import PoseStamped, PoseType, TransformStamped

# PYTHON
from dataclasses import dataclass, field
import copy
import numpy as np
from typing import Optional
from tf_transformations import xyzrpy_to_matrix


@dataclass
class Conveyor(CompoundVisualObject):
    """Conveyor belt visualization with configurable geometry.

    Creates a conveyor model with:
    - Belt box (default black)
    - 4 coordinate frames showing key reference points

    Frame structure (all derived from T_world_to_surface_center):
    1. surface_center: Center of top surface (PRIMARY REFERENCE FRAME)
       - Simplest for positioning the conveyor

    2. center: Geometric center of conveyor box
       - Offset from surface_center: z = -height/2
       - Used for visual/collision geometry

    3. surface_front: Front edge of top surface
       - Offset from surface_center: y = length/2
       - Useful for robot positioning

    4. roller_front: Front roller position (bottom front edge)
       - Offset from surface_center: y = length/2, z = -height/2
       - Useful for tilting operations

    All frames have: Z up, Y forward off conveyor, X right

    Note: name and transforms are optional - useful for creating templates
    that will be cloned and positioned later.
    """

    # Data
    T_world_to_surface_center: TransformStamped = field(
        default_factory=TransformStamped
    )
    size: tuple[float, float, float] = (0.4, 2.0, 0.05)  # (width, length, height)

    # Visual properties
    belt_material: Material = field(
        default_factory=lambda: Material(
            color=RGBA.from_list([0.1, 0.1, 0.1, 1.0]),  # Black
            metallic=0.0,
            roughness=0.9,
        )
    )

    # Frame options
    frame_scale: float = 0.1  # Size of coordinate frame (0 to disable)

    def __post_init__(self):
        super().__post_init__()
        self.build_model()

    def build_model(self):
        """Build the conveyor model geometry based on current parameters."""
        super().build_model()

        # Get dimensions
        width, length, height = self.size

        # Derive all 4 frames from surface_center
        # 1. Center: geometric center (offset down by height/2)
        T_world_to_center = self.T_world_to_surface_center.offset(
            xyz=[0.0, 0.0, -height / 2],
            local=True,
            child_frame_id=f"{self.visual_id}_center",
        )

        # 2. Surface front: front edge of top surface (offset forward by length/2)
        T_world_to_surface_front = self.T_world_to_surface_center.offset(
            xyz=[0.0, length / 2, 0.0],
            local=True,
            child_frame_id=f"{self.visual_id}_surface_front",
        )

        # 3. Roller front: front roller position (offset forward and down)
        T_world_to_roller_front = self.T_world_to_surface_center.offset(
            xyz=[0.0, length / 2, -height / 2],
            local=True,
            child_frame_id=f"{self.visual_id}_roller_front",
        )

        # Create belt box at the center
        belt = Box(
            name=f"{self.visual_id}/belt",
            pose=T_world_to_center.to_pose_stamped(),
            scale=[width, length, height],
            color=self.belt_material.color,
        )
        self.add(belt)

        # Add coordinate frames if enabled
        if self.frame_scale > 0.0:
            # 1. Center frame
            frame_center = Frame(
                name=f"{self.visual_id}/center",
                transform=T_world_to_center,
                axis_length=self.frame_scale,
            )
            self.add(frame_center)

            # 2. Surface center frame (PRIMARY)
            frame_surface_center = Frame(
                name=f"{self.visual_id}/surface_center",
                transform=self.T_world_to_surface_center,
                axis_length=self.frame_scale
                * 1.2,  # Slightly larger to indicate primary
            )
            self.add(frame_surface_center)

            # 3. Surface front frame
            frame_surface_front = Frame(
                name=f"{self.visual_id}/surface_front",
                transform=T_world_to_surface_front,
                axis_length=self.frame_scale,
            )
            self.add(frame_surface_front)

            # 4. Roller front frame
            frame_roller_front = Frame(
                name=f"{self.visual_id}/roller_front",
                transform=T_world_to_roller_front,
                axis_length=self.frame_scale,
            )
            self.add(frame_roller_front)


if __name__ == "__main__":

    # Example 1: Standard conveyor (surface_center at origin)
    print("Example 1: Standard conveyor (surface_center at origin)")
    from bam.msgs import TransformStamped

    conveyor1 = Conveyor(
        name="standard_conveyor",
        T_world_to_surface_center=TransformStamped.from_xyzrpy([0.5, 0, 0], [0, 0, 0]),
        size=(0.4, 2.0, 0.05),
    )
    print(f"  Created with {len(conveyor1.children)} children")
    print(f"  Size: {conveyor1.size}")

    # Example 2: Custom size conveyor
    print("\nExample 2: Custom size conveyor")
    conveyor2 = Conveyor(
        name="wide_conveyor",
        T_world_to_surface_center=TransformStamped.from_xyzrpy([1.5, 0, 0], [0, 0, 0]),
        size=(0.6, 3.0, 0.08),
    )
    print(f"  Size: {conveyor2.size}")

    # Example 3: Custom color conveyor
    print("\nExample 3: Custom color conveyor (red)")
    conveyor3 = Conveyor(
        name="red_conveyor",
        T_world_to_surface_center=TransformStamped.from_xyzrpy([0, 1.0, 0], [0, 0, 0]),
        size=(0.4, 1.5, 0.05),
        belt_material=Material(color=RGBA.red(), metallic=0.0, roughness=0.9),
    )
    print(f"  Created with custom red color")

    # Example 4: Conveyor without frame
    print("\nExample 4: Conveyor without frame")
    conveyor4 = Conveyor(
        name="no_frame_conveyor",
        T_world_to_surface_center=TransformStamped.from_xyzrpy([0, -1.0, 0], [0, 0, 0]),
        size=(0.3, 1.0, 0.04),
        frame_scale=0.0,
    )
    print(f"  Created without coordinate frame")

    # Example 5: Rotated conveyor
    print("\nExample 5: Rotated conveyor (45 degrees)")
    conveyor5 = Conveyor(
        name="rotated_conveyor",
        T_world_to_surface_center=TransformStamped.from_xyzrpy(
            [1.0, 1.0, 0], [0, 0, np.pi / 4]
        ),
        size=(0.4, 2.0, 0.05),
    )
    print(f"  Created with 45° rotation")

    print("\n✓ All examples created successfully!")
