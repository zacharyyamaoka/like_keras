#!/usr/bin/env python3

"""
    Helpers for displaying Open3D geometries inside MeshCat.
"""

# PYTHON
from typing import Iterable, Tuple

import meshcat.geometry as g
import numpy as np
import open3d as o3d


def o3d_to_meshcat(
    geometries: Iterable[o3d.geometry.Geometry],
    viz,
    name_prefix: str = "object",
    clear_previous: bool = True,
) -> list[str]:
    """Convert Open3D geometries to MeshCat nodes."""
    if clear_previous:
        viz[name_prefix].delete()

    object_names: list[str] = []
    for idx, geometry in enumerate(geometries):
        obj_name = f"{name_prefix}/{idx}"
        if isinstance(geometry, o3d.geometry.TriangleMesh):
            _convert_triangle_mesh(geometry, viz, obj_name)
        elif isinstance(geometry, o3d.geometry.LineSet):
            _convert_line_set(geometry, viz, obj_name)
        elif isinstance(geometry, o3d.geometry.PointCloud):
            _convert_point_cloud(geometry, viz, obj_name)
        else:
            print(f"[WARNING] Unsupported geometry type: {type(geometry)}")
            continue
        object_names.append(obj_name)

    return object_names


def _convert_triangle_mesh(mesh: o3d.geometry.TriangleMesh, viz, name: str) -> None:
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    if mesh.has_vertex_colors():
        colors = np.asarray(mesh.vertex_colors)
    else:
        colors = np.ones((len(vertices), 3)) * 0.5

    geometry = g.TriangularMeshGeometry(vertices, triangles)
    avg_color = (colors.mean(axis=0) * 255).astype(int)
    color_hex = avg_color[0] * 0x10000 + avg_color[1] * 0x100 + avg_color[2]
    material = g.MeshLambertMaterial(color=color_hex, reflectivity=0.5)
    viz[name].set_object(geometry, material)


def _convert_line_set(line_set: o3d.geometry.LineSet, viz, name: str) -> None:
    points = np.asarray(line_set.points)
    lines = np.asarray(line_set.lines)
    if line_set.has_colors():
        avg_color = np.asarray(line_set.colors).mean(axis=0)
    else:
        avg_color = np.array([0.5, 0.5, 0.5])

    line_points = []
    for line in lines:
        line_points.append(points[line[0]])
        line_points.append(points[line[1]])
    line_points = np.array(line_points).T

    geometry = g.PointsGeometry(line_points)
    color_hex = int(avg_color[0] * 255) * 0x10000 + int(avg_color[1] * 255) * 0x100 + int(avg_color[2] * 255)
    material = g.LineBasicMaterial(color=color_hex, linewidth=2)
    viz[name].set_object(g.LineSegments(geometry, material))


def _convert_point_cloud(pcd: o3d.geometry.PointCloud, viz, name: str) -> None:
    points = np.asarray(pcd.points).T
    if pcd.has_colors():
        colors = np.asarray(pcd.colors).T
    else:
        colors = np.ones_like(points) * 0.5

    viz[name].set_object(g.PointCloud(position=points, color=colors, size=0.01))


def draw_waypoint_action_meshcat(
    waypoint_action,
    viz,
    name_prefix: str = "waypoint_action",
    show_gripper: bool = True,
    frame_scale: float = 0.01,
    point_radius: float = 0.001,
    point_color: list[float] | Tuple[float, float, float] = (0.0, 0.0, 0.0),
    show_lines: bool = True,
    line_color: list[float] | Tuple[float, float, float] = (0.5, 0.5, 0.5),
    clear_previous: bool = True,
) -> None:
    """Draw waypoint action frames/points/lines in MeshCat."""
    if clear_previous:
        viz[name_prefix].delete()

    waypoint_matrices = [wp.to_matrix() for wp in waypoint_action.waypoints]
    if frame_scale > 0.0:
        for idx, transform in enumerate(waypoint_matrices):
            frame_name = f"{name_prefix}/frames/frame_{idx}"
            viz[frame_name].set_object(g.triad(scale=frame_scale))
            viz[frame_name].set_transform(transform)

    if point_radius > 0.0:
        color_hex = int(point_color[0] * 255) * 0x10000 + int(point_color[1] * 255) * 0x100 + int(point_color[2] * 255)
        for idx, transform in enumerate(waypoint_matrices):
            sphere_name = f"{name_prefix}/points/point_{idx}"
            viz[sphere_name].set_object(g.Sphere(point_radius), g.MeshLambertMaterial(color=color_hex))
            viz[sphere_name].set_transform(transform)

    if show_lines and len(waypoint_matrices) > 1:
        line_points = []
        for idx in range(len(waypoint_matrices) - 1):
            line_points.append(waypoint_matrices[idx][:3, 3])
            line_points.append(waypoint_matrices[idx + 1][:3, 3])
        line_points = np.array(line_points).T
        geometry = g.PointsGeometry(line_points)
        color_hex = int(line_color[0] * 255) * 0x10000 + int(line_color[1] * 255) * 0x100 + int(line_color[2] * 255)
        material = g.LineBasicMaterial(color=color_hex, linewidth=2)
        viz[f"{name_prefix}/lines"].set_object(g.LineSegments(geometry, material))

    if show_gripper:
        print("[WARNING] Gripper visualization not yet implemented in MeshCat version")

    print(f"[DISPLAYED] WaypointAction with {len(waypoint_matrices)} waypoints in MeshCat")


__all__ = ["o3d_to_meshcat", "draw_waypoint_action_meshcat"]

