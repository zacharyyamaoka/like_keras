#!/usr/bin/env python3

"""
ScoreCloud - Point cloud with scores and automatic coloring.

Simple extension of PointCloud that adds score-based coloring using matplotlib colormaps.
Colors can be provided directly or auto-generated from scores.

Example:
    import bam.msgs.visual_objects as viz

    # Auto-color from scores
    cloud = viz.ScoreCloud(
        points=[[x, y, z], ...],
        scores=[0.95, 0.87, ...],
        colormap="viridis"  # or "RdYlGn", "plasma", etc.
    )

    # With binning for clearer visualization
    cloud = viz.ScoreCloud(
        points=points,
        scores=scores,
        colormap="RdYlGn",
        bin_config=viz.BinConfig(first_edge=0.2, last_edge=0.99, n_bins=5)
    )

    # Or provide colors directly
    cloud = viz.ScoreCloud(
        points=points,
        scores=scores,
        colors=[RGBA(...), ...]  # Override colormap
    )

    artist.draw(cloud)

See matplotlib colormaps: https://matplotlib.org/stable/gallery/color/colormap_reference.html
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from ..PointCloud import PointCloud
from ..RGBA import RGBA
from bam.utils import BinConfig, bin_values_vec

# PYTHON
from dataclasses import dataclass, field
import numpy as np
from typing import Optional, Literal


@dataclass
class ScoreCloud(CompoundVisualObject):
    """Point cloud with scores for automatic coloring.

    Essentially: PointCloud + scores + coloring utilities

    ScoreCloud = PointCloud + scores
    - Can auto-generate colors from scores using matplotlib colormaps
    - Can apply optional binning for discrete color levels
    - Provides numpy utilities for dynamic filtering
    """

    # The actual point cloud (holds points, colors, visual properties)
    point_cloud: PointCloud = field(default_factory=PointCloud)

    # Scores associated with each point
    scores: list[float] = field(default_factory=list)

    # Optional: colormap and binning for auto-coloring
    colormap: Optional[str] = None  # If set, auto-generate colors
    bin_config: Optional[BinConfig] = None  # Optional binning

    # Filter state memory (for GUI persistence)
    min_filter: float = 0.0  # Last used minimum filter value
    max_filter: float = 1.0  # Last used maximum filter value

    def __post_init__(self):
        super().__post_init__()
        self.build_model()

    def build_model(self):
        """Build the score cloud model geometry based on current parameters."""
        super().build_model()

        # Auto-generate colors if colormap is specified and colors not provided
        # Check if colors match points (if not, regenerate from colormap)
        n_points = len(self.point_cloud.points)
        n_colors = len(self.point_cloud.colors)

        if self.colormap is not None and (n_colors == 0 or n_colors != n_points):
            colors = self._generate_colors(self.scores)
            self.point_cloud.colors = colors

        # Add point cloud as child (even if empty, so Viser can clear it)
        self.point_cloud.name = f"{self.visual_id}/cloud"
        self.add(self.point_cloud)

    def to_numpy(
        self, alpha: bool = True, base8: bool = False, bgr: bool = False
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Convert to numpy arrays.

        Returns:
            points: (N, 3) array
            colors: (N, 3) or (N, 4) array
            scores: (N,) array
        """
        points, colors = self.point_cloud.to_numpy(alpha=alpha, base8=base8, bgr=bgr)
        scores = np.array(self.scores)
        return points, colors, scores

    def filter_copy(
        self, min_score: float = 0.0, max_score: float = 1.0
    ) -> "ScoreCloud":
        """Return a filtered copy of this ScoreCloud.

        Efficiently filters using numpy for numeric data (points, scores),
        and only loops for RGBA color objects which can't be vectorized.

        Args:
            min_score: Minimum score threshold
            max_score: Maximum score threshold

        Returns:
            New ScoreCloud with filtered data
        """
        # Convert to numpy arrays for efficient filtering
        scores_array = np.array(self.scores)
        points_array = np.array(self.point_cloud.points)

        # Create mask
        mask = (scores_array >= min_score) & (scores_array <= max_score)

        # Filter points and scores using numpy (fast vectorized operations)
        filtered_points = points_array[mask].tolist()
        filtered_scores = scores_array[mask].tolist()

        # Filter colors - only loop for RGBA objects (can't vectorize custom classes)
        if len(self.point_cloud.colors) > 0:
            indices = np.where(mask)[0]
            filtered_colors = [self.point_cloud.colors[i] for i in indices]
        else:
            filtered_colors = []

        # Create filtered point cloud with same properties
        filtered_pc = PointCloud(
            points=filtered_points,
            colors=filtered_colors,
            point_size=self.point_cloud.point_size,
            point_shape=self.point_cloud.point_shape,
        )

        # Create filtered ScoreCloud
        return ScoreCloud(
            name=self.name,
            point_cloud=filtered_pc,
            scores=filtered_scores,
            colormap=self.colormap,
            bin_config=self.bin_config,
        )

    @staticmethod
    def filter_by_score(
        points: np.ndarray,
        scores: np.ndarray,
        colors: Optional[np.ndarray],
        min_score: float = 0.0,
        max_score: float = 1.0,
    ) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
        """Filter points, scores, and colors by score range using numpy masks.

        DEPRECATED: Use filter_copy() instead for cleaner API.

        Args:
            points: (N, 3) array
            scores: (N,) array
            colors: (N, 3/4) array or None
            min_score: Minimum score threshold
            max_score: Maximum score threshold

        Returns:
            filtered_points: (M, 3) array
            filtered_scores: (M,) array
            filtered_colors: (M, 3/4) array or None
        """
        mask = (scores >= min_score) & (scores <= max_score)

        filtered_points = points[mask]
        filtered_scores = scores[mask]
        filtered_colors = colors[mask] if colors is not None else None

        return filtered_points, filtered_scores, filtered_colors

    def _generate_colors(self, scores: list[float]) -> list[RGBA]:
        """Generate colors from scores using matplotlib colormap."""
        import matplotlib.pyplot as plt

        scores_array = np.array(scores)

        # Apply binning if configured (using vectorized binning from bam_utils)
        if self.bin_config is not None:
            scores_array = bin_values_vec(scores_array, self.bin_config)

        # Get matplotlib colormap
        try:
            cmap = plt.get_cmap(self.colormap)
        except ValueError:
            print(
                f"Warning: Unknown colormap '{self.colormap}', falling back to 'viridis'"
            )
            cmap = plt.get_cmap("viridis")

        # Map scores to colors
        colors = []
        for score in scores_array:
            rgba = cmap(score)  # Returns (r, g, b, a) tuple
            colors.append(RGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3]))

        return colors

    @classmethod
    def from_numpy(
        cls,
        points: np.ndarray,
        scores: np.ndarray,
        colors: Optional[np.ndarray] = None,
        name: str = "score_cloud",
        colormap: Optional[str] = "viridis",
        bin_config: Optional[BinConfig] = None,
        point_cloud: Optional[PointCloud] = None,
        **kwargs,
    ) -> "ScoreCloud":
        """Create ScoreCloud from numpy arrays.

        Args:
            points: (N, 3) array of positions
            scores: (N,) array of scores
            colors: Optional (N, 3/4) array of colors (overrides colormap)
            name: Name for the visual object
            colormap: Matplotlib colormap name (if colors not provided)
            bin_config: Optional binning configuration
            point_cloud: Optional PointCloud with custom properties
            **kwargs: Additional properties
        """
        # Create or use provided PointCloud
        if point_cloud is None:
            point_cloud = PointCloud()

        # Set points
        point_cloud.points = points.tolist()

        # Set colors if provided
        if colors is not None:
            if colors.ndim == 2 and colors.shape[1] in (3, 4):
                point_cloud.colors = [RGBA.from_numpy(color) for color in colors]
            else:
                raise ValueError("colors must be (N, 3) or (N, 4) array")

        return cls(
            name=name,
            point_cloud=point_cloud,
            scores=scores.tolist(),
            colormap=(
                colormap if colors is None else None
            ),  # Only use colormap if no colors
            bin_config=bin_config,
            **kwargs,
        )


if __name__ == "__main__":
    # Example usage
    from bam.utils import BinConfig  # Import for standalone testing

    np.random.seed(42)

    # Create sample data
    n_points = 1000
    points = np.random.randn(n_points, 3) * 0.5
    scores = np.random.rand(n_points)

    print("Example 1: Simple viridis colormap")
    cloud1 = ScoreCloud.from_numpy(
        points=points, scores=scores, name="viridis_cloud", colormap="viridis"
    )
    print(f"  Created with {len(cloud1.point_cloud.points)} points")

    print("\nExample 2: RdYlGn colormap with auto-calculated binning")
    cloud2 = ScoreCloud.from_numpy(
        points=points,
        scores=scores,
        name="binned_cloud",
        colormap="RdYlGn",
        bin_config=BinConfig(n_bins=5),  # first_edge and last_edge auto-calculated
    )
    print(f"  Created with {len(cloud2.point_cloud.points)} points")
    print(f"  Binned into 5 evenly spaced discrete color levels")

    print("\nExample 2b: Manual bin edges")
    cloud2b = ScoreCloud.from_numpy(
        points=points,
        scores=scores,
        name="manual_binned_cloud",
        colormap="RdYlGn",
        bin_config=BinConfig(first_edge=0.2, last_edge=0.99, n_bins=5),
    )
    print(f"  Created with manual bin edges at 0.2 and 0.99")

    print("\nExample 3: Dynamic score filtering with filter_copy()")
    # Use filter_copy method (clean, object-oriented approach)
    cloud3 = cloud1.filter_copy(min_score=0.5, max_score=1.0)
    cloud3.name = "filtered_cloud"
    print(f"  Filtered to {len(cloud3.point_cloud.points)} points (score > 0.5)")
    print(f"  Original still has {len(cloud1.point_cloud.points)} points")

    print("\nExample 4: Custom colors (override colormap)")
    colors_np = np.array([[1, 0, 0, 1] if s > 0.5 else [0, 0, 1, 1] for s in scores])
    cloud4 = ScoreCloud.from_numpy(
        points=points, scores=scores, colors=colors_np, name="custom_colors"
    )
    print(f"  Created with custom RGBA colors")

    print("\nExample 5: Custom point cloud properties")
    custom_pc = PointCloud(point_size=0.05, point_shape="diamond")
    cloud5 = ScoreCloud.from_numpy(
        points=points,
        scores=scores,
        name="custom_props",
        colormap="jet",
        point_cloud=custom_pc,
    )
    print(f"  Created with custom point size and shape")

    print("\nExample 6: to_numpy() method")
    pts_np, colors_np, scores_np = cloud1.to_numpy()
    print(
        f"  Exported: points {pts_np.shape}, colors {colors_np.shape}, scores {scores_np.shape}"
    )

    print("\n✓ All examples created successfully!")
    print("\nKey Design:")
    print("  - ScoreCloud = PointCloud + scores")
    print("  - Auto-coloring from scores using matplotlib colormaps")
    print("  - filter_copy() returns filtered copy (clean, object-oriented)")
    print("  - to_numpy() exports all data for custom processing")
    print("\nAvailable matplotlib colormaps:")
    print("  Perceptually Uniform: viridis, plasma, inferno, magma, cividis")
    print("  Diverging: RdYlGn, RdYlBu, RdBu, coolwarm, seismic")
    print("  Sequential: Reds, Blues, Greens, Greys")
    print("  Miscellaneous: jet, rainbow, turbo")
    print("\nSee: https://matplotlib.org/stable/gallery/color/colormap_reference.html")
