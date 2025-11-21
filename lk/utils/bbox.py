#!/usr/bin/env python3

"""
Bounding-box helpers for RGB-D derived point clouds.
"""

# PYTHON
import numpy as np
from tf_transformations import quaternion_matrix, xyzrpy_to_matrix

# BAM


def get_2d_rect_corners(
    x_min: float, y_min: float, x_max: float, y_max: float
) -> np.ndarray:
    """Return rectangle corners as 2x4 numpy array."""
    return np.array(
        [[x_min, x_max, x_max, x_min], [y_min, y_min, y_max, y_max]], dtype=float
    )


def corners_from_3d_bbox(
    position: np.ndarray, orientation: np.ndarray, widths: np.ndarray
) -> np.ndarray:
    """Create oriented 3D bounding box corners scaled by widths."""
    corners = np.array(
        [
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5],
        ],
        dtype=float,
    )
    corners *= widths
    rotation = quaternion_matrix(orientation)[:3, :3]
    corners = corners @ rotation.T
    corners += position
    return corners


def axis_aligned_bbox_from_points(points: np.ndarray) -> np.ndarray:
    """
    Compute axis-aligned bounding box dimensions from points.

    Args:
        points: (N, 3) array of 3D points

    Returns:
        extents: (3,) array of [x_size, y_size, z_size]
    """
    if points.shape[0] == 0:
        return np.zeros(3)

    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    extents = max_pt - min_pt

    return extents


def oriented_bbox_from_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute oriented bounding box from points using Open3D.

    Args:
        points: (N, 3) array of 3D points

    Returns:
        extents: (3,) array of box dimensions
        transform: (4, 4) homogeneous transform from box center frame to world frame
    """
    import open3d as o3d  # Lazy load as it's heavy

    # Compute oriented bounding box directly from points
    obb = o3d.geometry.OrientedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(points)
    )

    # Get extents (box dimensions)
    extents = np.array(obb.extent, dtype=np.float64)

    # Get center and rotation matrix
    center = np.array(obb.center, dtype=np.float64)
    R = np.array(obb.R, dtype=np.float64)

    # Construct 4x4 transform matrix from box center frame to world frame
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = R
    transform[:3, 3] = center

    return extents, transform


def points_to_bbox_3d(points: np.ndarray) -> None:
    """Placeholder for future minimal bounding-box fit."""
    return None


def filter_outliers(points_xyz: np.ndarray) -> np.ndarray:
    import open3d as o3d  # Lazy load as takes a while!

    """Remove outliers using Open3D radius filtering."""
    reshaped = points_xyz.reshape(-1, 3)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(reshaped)
    point_cloud = point_cloud.voxel_down_sample(voxel_size=0.002)
    point_cloud, _ = point_cloud.remove_radius_outlier(nb_points=5, radius=0.01)
    return np.asarray(point_cloud.points)


def points_to_bbox_2_5d(
    points_xyz: np.ndarray,
) -> tuple[np.ndarray, list[float], np.ndarray]:
    """Fit 2.5D oriented bounding box using PCA in XY plane."""
    pts = points_xyz.reshape(-1, 3)
    points_xy = pts[:, [0, 1]]
    zmin = float(np.min(pts[:, 2]))
    zmax = float(np.max(pts[:, 2]))
    zcenter = (zmax + zmin) / 2

    means = np.mean(pts, axis=0)
    means_xy = means[[0, 1]]
    points_centered = points_xy - means_xy
    covariance = np.cov(points_centered, rowvar=False)
    eig_val, eig_vec = np.linalg.eig(covariance)
    order = np.flip(np.argsort(np.abs(eig_val)))
    eig_vec = eig_vec[:, order]

    aligned = (eig_vec.T @ points_centered.T).T

    xmin = float(np.min(aligned[:, 0]))
    xmax = float(np.max(aligned[:, 0]))
    ymin = float(np.min(aligned[:, 1]))
    ymax = float(np.max(aligned[:, 1]))

    widths = [xmax - xmin, ymax - ymin, zmax - zmin]
    corners = get_2d_rect_corners(xmin, ymin, xmax, ymax)
    corners = (eig_vec @ corners).T + means_xy

    corners_top = np.column_stack([corners, np.full(4, zmax)])
    corners_bot = np.column_stack([corners, np.full(4, zmin)])
    np.concatenate([corners_top, corners_bot], axis=0)

    bbox_centers = np.array([xmin + xmax, ymin + ymax]) / 2
    rz_rad = float(np.arctan2(eig_vec[1, 0], eig_vec[0, 0]))
    bbox_pose = xyzrpy_to_matrix(
        [bbox_centers[0], bbox_centers[1], zcenter], (np.pi, 0.0, rz_rad)
    )
    obj_pose = xyzrpy_to_matrix([means[0], means[1], zcenter], (np.pi, 0.0, rz_rad))

    return bbox_pose, widths, obj_pose


__all__ = [
    "get_2d_rect_corners",
    "corners_from_3d_bbox",
    "points_to_bbox_3d",
    "filter_outliers",
    "points_to_bbox_2_5d",
]
