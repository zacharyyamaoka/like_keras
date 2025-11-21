#!/usr/bin/env python3

"""
Interactive Score Cloud with Histogram Example

Demonstrates the enhanced ScoreCloudGui with:
- Score range filtering via multi-slider
- Interactive histogram showing score distribution
- Colormap selector dropdown
- Real-time updates
"""

if __name__ == "__main__":

    # BAM
    import bam.msgs.visual_objects as viz
    from bam_artist import Artist

    # PYTHON
    import numpy as np

    print("Creating Score Clouds with Histogram GUI...")

    # Create sample data
    np.random.seed(42)

    # Cloud 1: Random sphere with normal distribution scores
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

    # Normal distribution for scores (centered around 0.6)
    scores_1 = np.clip(np.random.normal(0.6, 0.2, n_points_1), 0.0, 1.0)

    cloud_1 = viz.ScoreCloud.from_numpy(
        points=points_1, scores=scores_1, name="sphere_cloud", colormap="viridis"
    )

    # Cloud 2: Grid with bimodal score distribution
    n_grid = 30
    x = np.linspace(-0.5, 0.5, n_grid)
    y = np.linspace(-0.5, 0.5, n_grid)
    xx, yy = np.meshgrid(x, y)

    points_2 = np.column_stack(
        [xx.ravel() + 1.2, yy.ravel(), np.zeros(n_grid * n_grid)]
    )

    # Bimodal distribution: high scores on edges, low in middle
    dist_from_center = np.sqrt((xx.ravel()) ** 2 + (yy.ravel()) ** 2)
    scores_2 = np.where(
        dist_from_center > 0.3,
        np.random.uniform(0.7, 1.0, n_grid * n_grid),
        np.random.uniform(0.0, 0.3, n_grid * n_grid),
    )

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

    # Draw the enhanced GUI control with histogram
    print("Drawing Enhanced GUI...")
    gui = viz.ScoreCloudGui(
        name="Score Cloud Control", show_histogram=True, histogram_bins=20
    )
    artist.draw(gui)

    print("\n" + "=" * 70)
    print("Enhanced Score Cloud GUI Example Running!")
    print("=" * 70)
    print("\nOpen your browser to interact with the GUI:")
    print(f"  URL: http://localhost:8080")
    print("\nNew Features:")
    print("  1. Interactive histogram shows score distribution")
    print("  2. Histogram bars use the point cloud's colormap colors")
    print("  3. Unselected bars are grayed out, selected bars show colormap")
    print("  4. Multi-slider to filter points by score range")
    print("  5. Dropdown to select which cloud to control")
    print("\nCreated Clouds:")
    print(f"  - sphere_cloud: {n_points_1} points (normal distribution)")
    print(f"  - grid_cloud: {n_grid * n_grid} points (bimodal distribution)")
    print("\nTry This:")
    print("  - Adjust the score range slider and watch the histogram colors update")
    print("  - Switch between clouds to see different distributions and colormaps")
    print("  - Notice how histogram colors match the point cloud colors")
    print("\nPress Ctrl+C to exit")
    print("=" * 70 + "\n")

    # Keep the viewer running
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nShutting down...")
