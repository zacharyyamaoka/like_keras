#!/usr/bin/env python3

"""
Test RandomQuaternion sampling with Open3D visualization.

Verifies that the spherical cap sampling works correctly by:
1. Sampling 100 quaternions
2. Visualizing all axes (full frames)
3. Visualizing just z-axes (arrows) to verify hemisphere constraint
"""

# BAM
from ..geometry_msgs import RandomQuaternion
from bam_artist import Artist, ViewerConfig
import bam.msgs.visual_objects as viz

# PYTHON
import numpy as np


def ex_continuous_hemisphere_sampling(
    initial_view=[0, 0, 1],
    hemisphere_angle=np.deg2rad(45),
    rotation_min=0.0,
    rotation_max=2 * np.pi,
    n_samples=100,
):
    """Test continuous hemisphere sampling with visualization."""

    rq = RandomQuaternion.continuous_hemisphere(
        initial_view=initial_view,
        hemisphere_angle=hemisphere_angle,
        rotation_min=rotation_min,
        rotation_max=rotation_max,
    ).with_seed(42)

    R_list = []
    for i in range(n_samples):
        quat = rq.sample()
        R = quat.to_matrix()[:3, :3]  # Extract 3x3 rotation matrix
        R_list.append(R)

    config = ViewerConfig.make_zach_config()
    artist = Artist(config=config)
    artist.attach_o3d_viewer()

    artist.draw(viz.Frame(name="origin"))

    artist.draw(
        viz.RList(
            name="sampled_frames",
            R_list=R_list,
            origin=[0, 0, 0],
            scale=0.05,
            only_z=False,
        )
    )

    artist.render()
    artist.clear_all()

    artist.draw(viz.Frame(name="origin"))

    artist.draw(
        viz.RList(
            name="sampled_z_axes",
            R_list=R_list,
            origin=[0, 0, 0],
            scale=0.05,
            only_z=True,
            color_by_order=False,
            # arrow_props=viz.Arrow(color=viz.RGBA.cyan())
        )
    )
    artist.render()


def ex_discrete_sampling():
    """Test discrete sampling from view_generator."""

    print("\n" + "=" * 70)
    print("Testing RandomQuaternion Discrete Sampling (view_generator)")
    print("=" * 70)

    # Create discrete quaternion sampler
    rq = RandomQuaternion.discrete_from_view_generator(
        initial_view=[0, 0, 1],
        hemisphere_angle=np.deg2rad(45),
        view_step=np.deg2rad(22.5),
        rotation_step=np.deg2rad(90),
        start_angle=0.0,
    )

    print(f"\nConfiguration:")
    print(f"  Total permutations: {rq.permutation_total}")

    # Convert discrete quaternions to rotation matrices
    R_list = []
    for i in range(rq.permutation_total):
        quat, done = rq.permute()
        R = quat.to_matrix()[:3, :3]
        R_list.append(R)

    artist = Artist()

    # Draw reference frame
    artist.draw(
        viz.Frame(
            name="origin",
            axis_length=0.3,
            origin_radius=0.02,
            origin_color=viz.RGBA.white(),
        )
    )

    # Visualization 1: Full frames
    print(f"\nDrawing {rq.permutation_total} discrete orientations (full frames)...")
    frames = viz.RList(
        name="discrete_frames", R_list=R_list, origin=[0, 0, 0], scale=0.2, only_z=False
    )
    artist.draw(frames)
    print(f"  Rendered {len(frames.children)} frames")
    print("\nClose the window to continue...")
    artist.render()

    # Visualization 2: Just z-axes (with duplicates removed from in-plane rotations)
    print(f"\n[2] Drawing z-axes only (duplicates removed)...")
    artist.clear_all()

    # Draw reference frame
    artist.draw(
        viz.Frame(
            name="origin",
            axis_length=0.3,
            origin_radius=0.02,
            origin_color=viz.RGBA.white(),
        )
    )

    arrows = viz.RList(
        name="discrete_z_axes",
        R_list=R_list,
        origin=[0, 0, 0],
        scale=0.25,
        only_z=True,
        remove_z_duplicates=True,
        arrow_props=viz.Arrow(color=viz.RGBA.green()),
    )
    artist.draw(arrows)
    print(
        f"  Rendered {len(arrows.children)} unique z-axes (from {len(R_list)} rotations)"
    )
    print("\nClose the window to finish.")
    artist.render()


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("RandomQuaternion Sampling Visualization Tests")
    print("=" * 70)

    # Test 1: Continuous hemisphere sampling

    # View, Angle, Min, Max
    configs = [
        [[0, 0, 1], np.deg2rad(45), 0.0, 0.0],
        [[0, 0, 1], np.deg2rad(90), 0.0, 0.0],
        [[0, 0, 1], np.deg2rad(180), 0.0, 0.0],
        [[0, 0, 1], np.deg2rad(0), 0.0, np.pi],
        [[0, 0, 1], np.deg2rad(0), 0.0, 2 * np.pi],
        [[1, 1, 1], np.deg2rad(45), 0.0, 0.0],
        [[1, 1, 1], np.deg2rad(90), 0.0, 0.0],
        [[1, 1, 1], np.deg2rad(180), 0.0, 0.0],
        [[1, 1, 1], np.deg2rad(0), 0.0, np.pi],
        [[1, 1, 1], np.deg2rad(0), 0.0, 2 * np.pi],
        [[1, 0, 0], np.deg2rad(45), 0.0, 0.0],
        [[1, 0, 0], np.deg2rad(90), 0.0, 0.0],
        [[1, 0, 0], np.deg2rad(180), 0.0, 0.0],
        [[1, 0, 0], np.deg2rad(0), 0.0, np.pi],
        [[1, 0, 0], np.deg2rad(0), 0.0, 2 * np.pi],
    ]

    # for config in configs:
    #     ex_continuous_hemisphere_sampling(initial_view=config[0], hemisphere_angle=config[1], rotation_min=config[2], rotation_max=config[3])

    ex_continuous_hemisphere_sampling(
        initial_view=[1, 0, 1],
        hemisphere_angle=np.deg2rad(180),
        rotation_min=0.0,
        rotation_max=0,
        n_samples=1000,
    )

    # # Test 2: Discrete sampling
    # ex_discrete_sampling()

    # print("\n" + "="*70)
    # print("All tests complete!")
    # print("="*70)
