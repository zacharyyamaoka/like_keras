#!/usr/bin/env python3

"""
RList - Compound visual object for visualizing lists of rotation matrices.

Automatically creates Frame or Arrow objects for each rotation matrix.
This is useful for visualizing reachability maps, orientation samples, etc.

Example:
    import bam.msgs.visual_objects as viz
    from bam_artist import Artist

    # Create RList from rotation matrices
    R_list = [
        np.eye(3),
        np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    ]

    # Show full frames
    frames = viz.RList(
        name="orientations",
        R_list=R_list,
        origin=[0.5, 0, 0.5],
        scale=0.1,
        only_z=False
    )

    # Show just Z-axes (removes duplicates from in-plane rotations)
    arrows = viz.RList(
        name="z_axes",
        R_list=R_list,
        origin=[0.5, 0, 0.5],
        scale=0.1,
        only_z=True,
        remove_z_duplicates=True,
        arrow_props=viz.Arrow(color=viz.RGBA.blue())
    )

    artist.draw(frames)
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from ..Frame import Frame
from ..markers import Arrow
from ..RGBA import RGBA
from bam.msgs import TransformStamped, PoseStamped

# PYTHON
from dataclasses import dataclass, field
import copy
import numpy as np


@dataclass
class RList(CompoundVisualObject):
    """Visual representation of rotation matrices as frames or Z-axis arrows."""

    # Data
    R_list: list = field(default_factory=list)
    origin: list = field(default_factory=lambda: [0.0, 0.0, 0.0])

    # Display options
    scale: float = 0.1
    only_z: bool = False
    color_by_order: bool = False
    remove_z_duplicates: bool = False  # In plane rotations lead to duplicates

    # Viz properties
    frame_props: Frame = field(default_factory=lambda: Frame())
    arrow_props: Arrow = field(default_factory=lambda: Arrow())

    def __post_init__(self):
        super().__post_init__()
        self.build_model()

    def build_model(self):
        """Build the RList model by creating Frame or Arrow objects."""
        super().build_model()

        # Normalize input to list of 3x3 matrices
        R_list = self._normalize_R_list(self.R_list)

        if len(R_list) == 0:
            return

        origin = np.array(self.origin)
        num_frames = len(R_list)

        print(f"Creating RList with {len(R_list)} children")

        # Track unique Z-axes if removing duplicates
        unique_z_axes = []
        z_eps = 1e-6

        for i, R in enumerate(R_list):
            # Compute color gradient if requested
            color = None
            if self.color_by_order and num_frames > 1:
                t = i / (num_frames - 1)
                color = RGBA(r=t, g=t, b=1.0, a=1.0)

            # Build 4x4 transform matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = origin

            if self.only_z:
                # Check for duplicate Z-axes if flag is set
                z_axis = R[:, 2]
                if self.remove_z_duplicates:
                    is_duplicate = False
                    for existing_z in unique_z_axes:
                        if np.linalg.norm(z_axis - existing_z) < z_eps:
                            is_duplicate = True
                            break

                    if is_duplicate:
                        continue

                    unique_z_axes.append(z_axis.copy())

                # Create arrow pointing in Z direction
                pose = PoseStamped.from_matrix(T)

                arrow = copy.deepcopy(self.arrow_props)
                arrow.name = f"{self.visual_id}/arrow_{i}"
                arrow.pose = pose
                arrow.length = self.scale

                if color is not None:
                    arrow.color = color

                self.add(arrow)
            else:
                # Create full coordinate frame
                transform = TransformStamped.from_matrix(T)

                frame = copy.deepcopy(self.frame_props)
                frame.name = f"{self.visual_id}/frame_{i}"
                frame.transform = transform
                frame.axis_length = self.scale

                self.add(frame)

    def _normalize_R_list(self, R_list):
        """Normalize input to list of 3x3 matrices."""
        if isinstance(R_list, np.ndarray):
            if R_list.ndim == 2:
                return [R_list]
            elif R_list.ndim == 3:
                return list(R_list)
        elif not isinstance(R_list, list):
            return [R_list]
        return R_list


if __name__ == "__main__":
    # BAM
    from bam_reach.generators.view_generators import view_generator
    from bam_artist import Artist

    # Generate rotation matrices using view_generator
    print("Generating views...")
    R_list = view_generator(
        inital_view=[0, 0, 1],
        hemisphere_angle=np.deg2rad(90),
        view_step=np.deg2rad(45),
        rotation_step=np.deg2rad(90),
        max_rotation_steps=4,
    )
    print(f"Generated {len(R_list)} rotation matrices")

    # Create artist for visualization
    artist = Artist()

    # Test 1: Full frames
    frames = RList(
        name="view_frames",
        R_list=R_list,
        origin=[0.0, 0.0, 0.5],
        scale=0.1,
        only_z=False,
    )
    print(f"\nFrames: Created {len(frames.children)} frame children")

    # Test 2: Just Z-axes with color gradient
    arrows = RList(
        name="view_arrows",
        R_list=R_list,
        origin=[0.5, 0.0, 0.5],
        scale=0.15,
        only_z=True,
        color_by_order=True,
        arrow_props=Arrow(color=RGBA.blue()),
    )
    print(f"Arrows: Created {len(arrows.children)} arrow children")

    # Test 3: Dense hemisphere with in-plane rotations (will have duplicates)
    R_dense = view_generator(
        inital_view=[0, 0, -1],
        hemisphere_angle=np.deg2rad(45),
        view_step=np.deg2rad(15),
        rotation_step=np.deg2rad(60),
    )
    dense_arrows = RList(
        name="dense_views",
        R_list=R_dense,
        origin=[1.0, 0.0, 0.5],
        scale=0.08,
        only_z=True,
        color_by_order=True,
    )
    print(
        f"Dense: Created {len(dense_arrows.children)} arrow children from {len(R_dense)} rotations"
    )

    # Test 4: Same dense hemisphere but with duplicates removed
    dense_arrows_unique = RList(
        name="dense_views_unique",
        R_list=R_dense,
        origin=[1.5, 0.0, 0.5],
        scale=0.08,
        only_z=True,
        color_by_order=True,
        remove_z_duplicates=True,
    )
    print(
        f"Dense (unique): Created {len(dense_arrows_unique.children)} arrow children from {len(R_dense)} rotations"
    )

    # Draw all visualizations
    artist.draw(frames)
    artist.draw(arrows)
    artist.draw(dense_arrows)
    artist.draw(dense_arrows_unique)

    print("\nVisualization complete! Check the viewer.")
    input("Press Enter to exit...")
