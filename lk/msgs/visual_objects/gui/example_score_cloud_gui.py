#!/usr/bin/env python3

"""
Interactive Score Cloud Filtering Example

Demonstrates the ScoreCloudGui for filtering point clouds by score range.
- Multi-slider to adjust score range (0.0 to 1.0)
- Dropdown to select which ScoreCloud to filter
- Real-time filtering and visualization
"""

if __name__ == "__main__":

    # BAM
    import bam.msgs.visual_objects as viz
    from bam_artist import Artist

    # PYTHON
    import numpy as np

    print("Creating Score Clouds...")

    # Create sample data
    np.random.seed(42)

    # Cloud 1: Random sphere with scores
    n_points_1 = 2000
    theta = np.random.uniform(0, 2 * np.pi, n_points_1)
    phi = np.random.uniform(0, np.pi, n_points_1)
    r = np.random.uniform(0.3, 0.5, n_points_1)

    points_1 = np.column_stack(
        [
            r * np.sin(phi) * np.cos(theta),
            r * np.sin(phi) * np.sin(theta),
            r * np.cos(phi),
        ]
    )
    scores_1 = np.random.rand(n_points_1)

    cloud_1 = viz.ScoreCloud.from_numpy(
        points=points_1, scores=scores_1, name="sphere_cloud", colormap="viridis"
    )

    # Cloud 2: Grid with height-based scores
    n_grid = 30
    x = np.linspace(-0.5, 0.5, n_grid)
    y = np.linspace(-0.5, 0.5, n_grid)
    xx, yy = np.meshgrid(x, y)

    points_2 = np.column_stack(
        [xx.ravel() + 1.2, yy.ravel(), np.zeros(n_grid * n_grid)]  # Offset to the right
    )

    # Scores based on distance from center
    dist_from_center = np.sqrt((xx.ravel()) ** 2 + (yy.ravel()) ** 2)
    scores_2 = 1.0 - dist_from_center / dist_from_center.max()

    cloud_2 = viz.ScoreCloud.from_numpy(
        points=points_2, scores=scores_2, name="grid_cloud", colormap="RdYlGn"
    )

    # Create artist with viser viewer
    artist = Artist()
    viewer = artist.attach_viser_viewer(verbose=True)

    # Draw the score clouds
    print("Drawing Score Clouds...")
    artist.draw(cloud_1)
    artist.draw(cloud_2)

    # Draw a grid for reference
    artist.draw(viz.Grid(cell_size=0.1))

    # Draw the GUI control
    print("Drawing GUI...")
    gui = viz.ScoreCloudGui(name="Score Cloud Control")
    artist.draw(gui)

    print("\n" + "=" * 70)
    print("Score Cloud GUI Example Running!")
    print("=" * 70)
    print("\nOpen your browser to interact with the GUI:")
    print(f"  URL: http://localhost:8080")
    print("\nControls:")
    print("  1. Use the multi-slider to filter points by score range")
    print("  2. Use the dropdown to select which cloud to filter")
    print("  3. Watch the point cloud update in real-time!")
    print("\nCreated Clouds:")
    print(f"  - sphere_cloud: {n_points_1} points (random scores)")
    print(f"  - grid_cloud: {n_grid * n_grid} points (center-based scores)")
    print("\nPress Ctrl+C to exit")
    print("=" * 70 + "\n")

    # Keep the viewer running
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nShutting down...")
