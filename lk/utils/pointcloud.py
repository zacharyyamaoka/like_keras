#!/usr/bin/env python3

"""
    Point cloud helpers for RGB-D processing and visualization workflows.
"""

# PYTHON
from __future__ import annotations

import copy
from typing import Iterable, Tuple, TYPE_CHECKING

import cv2  # type: ignore[import-not-found]
import numpy as np
from transforms3d.euler import euler2mat

if TYPE_CHECKING:
    import open3d as o3d

def xyz_2_pc(xyz: np.ndarray, color: list[float] | tuple[float, float, float] | None = None) -> o3d.geometry.PointCloud:
    import open3d as o3d

    """Convert XYZ array to Open3D point cloud with optional uniform color."""
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))
    if color is not None:
        pc.paint_uniform_color(color)
    return pc


def rgbxyz_2_pc(rgb: np.ndarray, xyz: np.ndarray) -> o3d.geometry.PointCloud:
    import open3d as o3d

    """Convert RGB + XYZ arrays into Open3D colored point cloud."""
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))
    colors = rgb.astype(np.float32) / 255.0 if rgb.max() > 1.0 else rgb
    pc.colors = o3d.utility.Vector3dVector(colors.reshape(-1, 3))
    return pc


def rgb_2_patch(rgb: np.ndarray, x: int, y: int, window_size: int = 10) -> np.ndarray:
    """Return RGB patch centered at (x, y)."""
    return xyz_2_patch(rgb, x, y, window_size)


def xyz_2_patch(xyz: np.ndarray, x: int, y: int, window_size: int = 10, verbose: bool = False) -> np.ndarray:
    """Return XYZ patch centered at (x, y)."""
    height, width, _ = xyz.shape
    if not (0 <= x < width and 0 <= y < height):
        raise ValueError(f"Point (x={x}, y={y}) out of bounds for shape {(height, width)}")

    half = window_size // 2
    y_min = max(y - half, 0)
    y_max = min(y + half + (1 if window_size % 2 != 0 else 0), height)
    x_min = max(x - half, 0)
    x_max = min(x + half + (1 if window_size % 2 != 0 else 0), width)

    if verbose:
        print(f"xyz_2_patch center: (x={x}, y={y})")
        print(f"xyz_2_patch bounds: x [{x_min}:{x_max}], y [{y_min}:{y_max}]")

    return xyz[y_min:y_max, x_min:x_max, :]


def plane_2_depth(T_camera_plane: np.ndarray, img_height: int, img_width: int, camera_matrix: np.ndarray) -> np.ndarray:
    """Ray-plane intersection producing dense depth image (slow loop implementation)."""
    plane_normal = T_camera_plane[:3, 2]
    plane_origin = T_camera_plane[:3, 3]
    cx, cy, fx, fy = get_intrinsics(camera_matrix)

    depth_image = np.zeros((img_height, img_width))
    for y_idx in range(img_height):
        for x_idx in range(img_width):
            ray = np.array([(x_idx - cx) / fx, (y_idx - cy) / fy, 1.0])
            ray_unit = ray / np.linalg.norm(ray)
            denom = float(np.dot(ray_unit, plane_normal))
            if abs(denom) > 1e-6:
                t_val = float(np.dot(plane_origin, plane_normal) / denom)
                intersection = t_val * ray_unit
                depth_image[y_idx, x_idx] = intersection[2]
            else:
                depth_image[y_idx, x_idx] = np.nan
    return depth_image


def plane_2_depth_vectorized(T_camera_plane: np.ndarray, img_height: int, img_width: int, camera_matrix: np.ndarray) -> np.ndarray:
    """Vectorized ray-plane intersection ~60x faster for VGA images."""
    plane_normal = T_camera_plane[:3, 2]
    plane_origin = T_camera_plane[:3, 3]
    cx, cy, fx, fy = get_intrinsics(camera_matrix)

    xs = np.arange(img_width)
    ys = np.arange(img_height)
    u_coord, v_coord = np.meshgrid(xs, ys)

    rays = np.stack([(u_coord - cx) / fx, (v_coord - cy) / fy, np.ones_like(u_coord)], axis=-1)
    rays_unit = rays / np.linalg.norm(rays, axis=2, keepdims=True)
    denom = np.tensordot(rays_unit, plane_normal, axes=([2], [0]))

    plane_dot = float(np.dot(plane_origin, plane_normal))
    mask = np.abs(denom) > 1e-6
    t_vals = np.full_like(denom, np.nan, dtype=float)
    t_vals[mask] = plane_dot / denom[mask]

    intersection = t_vals[..., None] * rays_unit
    depth_image = np.full((img_height, img_width), np.nan)
    depth_image[mask] = intersection[..., 2][mask]
    return depth_image


def align_xyz(xyz: np.ndarray, x: float, y: float, z: float, rx: float, ry: float, rz: float) -> np.ndarray:
    """Transform XYZ points into frame defined by pose."""
    if xyz.shape[1] != 3:
        raise ValueError(f"Expected xyz with shape (N, 3), got {xyz.shape}")
    rotation = euler2mat(rx, ry, rz, axes="sxyz")
    translation = np.array([x, y, z])
    translated = xyz - translation
    return (rotation.T @ translated.T).T


def unalign_xyz(xyz_local: np.ndarray, x: float, y: float, z: float, rx: float, ry: float, rz: float) -> np.ndarray:
    """Transform points from local pose frame back to world."""
    if xyz_local.shape[1] != 3:
        raise ValueError(f"Expected xyz with shape (N, 3), got {xyz_local.shape}")
    rotation = euler2mat(rx, ry, rz, axes="sxyz")
    translation = np.array([x, y, z])
    return (rotation @ xyz_local.T).T + translation


def get_intrinsics(camera_info: np.ndarray | object) -> tuple[float, float, float, float]:
    """Extract (cx, cy, fx, fy) from ROS-like camera info or numpy matrix."""
    if hasattr(camera_info, "k"):
        K = camera_info.k
        return float(K[2]), float(K[5]), float(K[0]), float(K[4])
    if hasattr(camera_info, "shape"):
        return float(camera_info[0, 2]), float(camera_info[1, 2]), float(camera_info[0, 0]), float(camera_info[1, 1])
    raise ValueError("camera_info must have 'k' attribute or be 3x3 matrix.")


def update_intriniscs(
    camera_info: np.ndarray | object,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
    crop_x: float = 0.0,
    crop_y: float = 0.0,
) -> np.ndarray | object:
    """Scale and crop intrinsic parameters."""
    updated = copy.deepcopy(camera_info)
    if hasattr(updated, "k"):
        K = list(updated.k)
        K[0] *= scale_x
        K[4] *= scale_y
        K[2] = K[2] * scale_x - crop_x
        K[5] = K[5] * scale_y - crop_y
        updated.k = K
        return updated
    if isinstance(updated, np.ndarray) and updated.shape == (3, 3):
        updated[0, 0] *= scale_x
        updated[1, 1] *= scale_y
        updated[0, 2] = updated[0, 2] * scale_x - crop_x
        updated[1, 2] = updated[1, 2] * scale_y - crop_y
        return updated
    raise ValueError("camera_info must have 'k' attribute or be 3x3 matrix.")


def px_2_point(x: float, y: float, depth: np.ndarray, camera_info: np.ndarray | object) -> tuple[float, float, float]:
    """Return XYZ for pixel coordinate given depth image + intrinsics."""
    cx, cy, fx, fy = get_intrinsics(camera_info)
    z_val = float(depth[round(y), round(x)])
    x_val = z_val * (x - cx) / fx
    y_val = z_val * (y - cy) / fy
    return x_val, y_val, z_val


def average_from_around(depth_img: np.ndarray, px: int, py: int) -> float:
    """Average nearest non-zero depth around pixel using expanding windows."""
    window_sizes = [3, 7, 15, 31, 63, 125]
    for size in window_sizes:
        half_size = size // 2
        min_x = max(px - half_size, 0)
        max_x = min(px + half_size + 1, depth_img.shape[1])
        min_y = max(py - half_size, 0)
        max_y = min(py + half_size + 1, depth_img.shape[0])
        window = depth_img[min_y:max_y, min_x:max_x]
        non_zero_values = window[window > 0]
        if non_zero_values.size > 0:
            return float(np.mean(non_zero_values))
    return 0.0


def px_2_point_safe(x: int, y: int, depth: np.ndarray, camera_info: np.ndarray | object) -> tuple[float, float, float]:
    """Safer px_2_point that fills missing depth from local average."""
    z_val = depth[y, x]
    if z_val == 0:
        z_val = average_from_around(depth, x, y)
        depth[y, x] = z_val
        print(f"No depth at pixel, avg. depth: {z_val}")
    return px_2_point(x, y, depth, camera_info)


def get_new_size(
    original_img: np.ndarray,
    scale: float | None = None,
    new_height: int | None = None,
    new_width: int | None = None,
) -> tuple[int, int]:
    """Compute new image size either by scale or explicit dimensions."""
    original_height, original_width = original_img.shape[:2]
    if scale is not None:
        return int(original_height * scale), int(original_width * scale)
    if new_width is not None and new_height is not None:
        return new_height, new_width
    raise ValueError("Either scale or both new_height and new_width must be provided.")


def downsample_mask(mask: np.ndarray, new_height: int | None = None, new_width: int | None = None, scale: float | None = None) -> np.ndarray:
    """Nearest-neighbor downsample for binary masks."""
    height, width = get_new_size(mask, scale, new_height, new_width)
    return cv2.resize(mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST)


def downsample_color(color: np.ndarray, new_height: int | None = None, new_width: int | None = None, scale: float | None = None) -> np.ndarray:
    """Area downsample for color images."""
    height, width = get_new_size(color, scale, new_height, new_width)
    return cv2.resize(color.astype(np.uint8), (width, height), interpolation=cv2.INTER_AREA)


def downsample_depth(depth: np.ndarray, new_height: int | None = None, new_width: int | None = None, scale: float | None = None) -> np.ndarray:
    """Nearest-neighbor downsample for depth."""
    height, width = get_new_size(depth, scale, new_height, new_width)
    return cv2.resize(depth.astype(np.float32), (width, height), interpolation=cv2.INTER_NEAREST)


def downsample_rgbd(
    color: np.ndarray,
    depth: np.ndarray,
    camera_info: np.ndarray | object,
    new_height: int | None = None,
    new_width: int | None = None,
    scale: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | object]:
    """Downsample color/depth images and adjust intrinsics accordingly."""
    if color.shape[:2] != depth.shape[:2]:
        raise ValueError(f"Shape mismatch: color {color.shape[:2]}, depth {depth.shape[:2]}")
    height, width = get_new_size(color, scale, new_height, new_width)
    color_ds = downsample_color(color, height, width, None)
    depth_ds = downsample_depth(depth, height, width, None)
    original_height, original_width = color.shape[:2]
    scale_x = width / original_width
    scale_y = height / original_height
    camera_ds = update_intriniscs(camera_info, scale_x, scale_y)
    return color_ds, depth_ds, camera_ds


def downsample_rgdb_mask(
    color: np.ndarray,
    depth: np.ndarray,
    camera_info: np.ndarray | object,
    mask: np.ndarray,
    new_height: int | None = None,
    new_width: int | None = None,
    scale: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | object]:
    """Downsample RGB, depth, mask, and intrinsics."""
    if color.shape[:2] != depth.shape[:2] or color.shape[:2] != mask.shape[:2]:
        raise ValueError("Color, depth, and mask must share dimensions.")
    color_ds, depth_ds, camera_ds = downsample_rgbd(color, depth, camera_info, new_height, new_width, scale)
    mask_ds = downsample_mask(mask, new_height, new_width, scale)
    return color_ds, depth_ds, mask_ds, camera_ds


def decimate_xyzrgb(xyz: np.ndarray, rgb: np.ndarray, scale: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
    """Decimate XYZ and RGB arrays via OpenCV resize."""
    xyz_resized = cv2.resize(xyz, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    rgb_resized = cv2.resize(rgb, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return xyz_resized, rgb_resized


def depth_2_xyz(depth: np.ndarray, camera_info: np.ndarray | object) -> np.ndarray:
    """Convert depth image to XYZ coordinates."""
    height, width = depth.shape
    u_vals, v_vals = np.meshgrid(np.arange(width), np.arange(height), sparse=True)
    cx, cy, fx, fy = get_intrinsics(camera_info)
    z_vals = depth
    x_vals = z_vals * (u_vals - cx) / fx
    y_vals = z_vals * (v_vals - cy) / fy
    return np.dstack((x_vals, y_vals, z_vals))


def rgbd_2_xyz(rgb: np.ndarray, depth: np.ndarray, camera_info: np.ndarray | object) -> np.ndarray:
    """Wrapper for depth_2_xyz ensuring RGB/depth shape match."""
    if rgb.shape[:2] != depth.shape:
        raise ValueError(f"Shape mismatch: rgb {rgb.shape[:2]} vs depth {depth.shape}")
    return depth_2_xyz(depth, camera_info)


def save_point_cloud(vertices: np.ndarray, filename: str = "pc.ply") -> None:
    """Save point cloud as ASCII PLY file."""
    header = """ply
  format ascii 1.0
  element vertex %(vert_num)d
  property float x
  property float y
  property float z
  property uchar red
  property uchar green
  property uchar blue
  end_header
"""
    with open(filename, "w", encoding="utf-8") as file_obj:
        file_obj.write(header % dict(vert_num=len(vertices)))
        np.savetxt(file_obj, vertices, "%f %f %f %d %d %d")


__all__ = [
    "xyz_2_pc",
    "rgbxyz_2_pc",
    "rgb_2_patch",
    "xyz_2_patch",
    "plane_2_depth",
    "plane_2_depth_vectorized",
    "align_xyz",
    "unalign_xyz",
    "get_intrinsics",
    "update_intriniscs",
    "px_2_point",
    "px_2_point_safe",
    "downsample_mask",
    "downsample_color",
    "downsample_depth",
    "downsample_rgbd",
    "downsample_rgdb_mask",
    "decimate_xyzrgb",
    "depth_2_xyz",
    "rgbd_2_xyz",
    "save_point_cloud",
]

