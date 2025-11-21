# BAM
from .RGBA import RGBA
from .VisualObject import VisualObject


# PYTHON
from dataclasses import dataclass, field
import copy
import numpy as np
from typing import Literal

"""
API references: 
https://viser.studio/main/examples/scene/point_clouds/

This represents a point cloud with N points, each with its own color.

Internally we store as a flat list of points and colors.

Creating it by numpy is nice, but for dataclasses, we prefer to hold in more basic type.
"""


@dataclass
class PointCloud(VisualObject):
    points: list[list[float, float, float]] = field(
        default_factory=lambda: [[0.0, 0.0, 0.0]]
    )
    colors: list[RGBA] = field(default_factory=lambda: [RGBA.grey()])
    point_size: float = 0.005
    point_shape: Literal["square", "diamond", "circle", "rounded", "sparkle"] = "square"

    def __post_init__(self):
        super().__post_init__()

    @classmethod
    def from_points(
        cls,
        points: list[list[float, float, float]],
        colors: list[RGBA] = [RGBA.grey()],
        name: str = "",
    ) -> "PointCloud":
        return cls(points=points, colors=colors, name=name)

    def to_numpy(
        self, alpha: bool = True, base8: bool = False, bgr: bool = False
    ) -> tuple[np.ndarray, np.ndarray]:
        points_array = np.array(self.points).reshape(-1, 3)

        colors_list = [
            color.to_numpy(alpha=alpha, base8=base8, bgr=bgr) for color in self.colors
        ]
        colors_array = np.array(colors_list).reshape(-1, 3 if not alpha else 4)

        # If single color, return as (3,) or (4,) instead of (1, 3) or (1, 4)
        # This matches viser's expected format for single-color point clouds
        if colors_array.shape[0] == 1:
            colors_array = colors_array.squeeze(0)

        return points_array, colors_array

    @classmethod
    def from_numpy(
        cls,
        points: np.ndarray,
        colors: np.ndarray | RGBA,
        base8: bool = False,
        bgr: bool = False,
        name: str = "",
    ) -> "PointCloud":
        points = copy.deepcopy(points)
        # this supports colors with and without alpha!
        assert (
            points.ndim == 2 and points.shape[1] == 3
        ), "points must be a 2D array with shape (N, 3)"

        # Convert points from (N, 3) to list of lists
        points_flat = points.tolist()
        num_points = len(points_flat)

        # Handle different color input types
        if isinstance(colors, RGBA):  # Single RGBA object - use for all points
            colors_flat = [colors] * num_points

        elif isinstance(colors, np.ndarray):

            # Array of colors (N, 3) or (N, 4)
            assert (
                colors.shape[0] == num_points
            ), "colors must have same length as points"
            assert colors.shape[1] in (
                3,
                4,
            ), "colors must have 3 (RGB) or 4 (RGBA) channels"
            colors_flat = []
            for color in colors:
                colors_flat.append(RGBA.from_numpy(color, base8, bgr))

        else:
            raise TypeError("colors must be np.ndarray or RGBA")

        return cls(points=points_flat, colors=colors_flat, name=name)

    @classmethod
    def from_rgbd(
        cls,
        rgb: np.ndarray,
        depth: np.ndarray,
        camera_intrinsics: dict,
        max_depth: float = 2.0,
        min_depth: float = 0.01,
        downsample: int = 1,
        name: str = "rgbd_cloud",
    ) -> "PointCloud":
        """Create point cloud from RGBD images.

        Args:
            rgb: RGB image (H, W, 3) with values 0-255 or 0-1
            depth: Depth image (H, W) in meters
            camera_intrinsics: Dict with 'fx', 'fy', 'cx', 'cy'
            max_depth: Maximum valid depth in meters
            min_depth: Minimum valid depth in meters
            downsample: Downsample factor (1=no downsampling, 2=every other pixel, etc.)
            name: Name for the point cloud

        Returns:
            PointCloud visual object
        """
        from bam.utils.pointcloud import downsample_rgbd, depth_2_xyz

        # Convert dict intrinsics to matrix format (compatible with pointcloud helpers)
        camera_info = np.array(
            [
                [camera_intrinsics["fx"], 0, camera_intrinsics["cx"]],
                [0, camera_intrinsics["fy"], camera_intrinsics["cy"]],
                [0, 0, 1],
            ]
        )

        # Downsample if requested
        if downsample > 1:
            scale = 1.0 / downsample
            rgb, depth, camera_info = downsample_rgbd(
                rgb, depth, camera_info, scale=scale
            )

        # Convert depth to XYZ using existing helper
        xyz = depth_2_xyz(depth, camera_info)

        # Filter valid depth values
        valid_mask = (depth > min_depth) & (depth < max_depth)

        # Reshape to (H*W, 3) and filter
        points = xyz.reshape(-1, 3)[valid_mask.reshape(-1)]

        # Get colors
        if rgb.dtype == np.uint8:
            colors = (
                rgb.reshape(-1, 3)[valid_mask.reshape(-1)].astype(np.float32) / 255.0
            )
        else:
            colors = rgb.reshape(-1, 3)[valid_mask.reshape(-1)].astype(np.float32)

        return cls.from_numpy(
            points=points, colors=colors, base8=False, bgr=False, name=name
        )
