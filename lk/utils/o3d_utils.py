#!/usr/bin/env python3

# BAM
from tf_transformations import xyzrpy_to_matrix, xyzrpy_offset
from .pointcloud import rgbxyz_2_pc, rgbd_2_xyz, xyz_2_pc, depth_2_xyz

# PYTHON
from dataclasses import dataclass, field
import time
from pathlib import Path
from typing import List

import numpy as np


try:
    import open3d as o3d

except ImportError as e:
    raise ImportError(
        "The 'open3d' package is required but not installed. Please install it via 'pip install open3d' and try again."
    ) from e


# region Viewers


@dataclass
class O3DViewerConfig:
    point_size: float = 5.0
    origin_frame_size: float = 0.03

    # window
    # window_name: str = "O3D Viewer"
    width: int = 800
    height: int = 600
    left: int = 100
    top: int = 100

    # view control
    front: list[float] = field(default_factory=lambda: [0.5, 0.5, 0.5])
    lookat: list[float] = field(default_factory=lambda: [0, 0, 0])
    up: list[float] = field(default_factory=lambda: [0, 0, 1.0])
    zoom: float = 1.0

    @classmethod
    def make_optical_frame(cls):
        return cls(up=[0, -1.0, 0])

    def look_at_xz(
        self, cam_pos=[0, -1, 0], lookat=[0, 0, 0], up=[0, 0, 1.0], zoom=1.0
    ):
        self.update_view_control(cam_pos=cam_pos, lookat=lookat, up=up, zoom=zoom)
        return self

    def look_at_yz(
        self, cam_pos=[1, 0, 0.5], lookat=[0, 0, 0.5], up=[0, 0, 1.0], zoom=1.0
    ):
        self.update_view_control(cam_pos=cam_pos, lookat=lookat, up=up, zoom=zoom)
        return self

    def look_at_xy(self, cam_pos=[0, 0, 0], lookat=[0, 0, 0], up=[0, 0, 1.0], zoom=1.0):
        self.update_view_control(cam_pos=cam_pos, lookat=lookat, up=up, zoom=zoom)
        return self

    def update_4k_window(self, left=1500, top=100):
        self.width = 1920
        self.height = 1080
        self.left = left
        self.top = top
        return self

    def update_view_control(self, cam_pos, lookat, up=[0, -1.0, 0], zoom=None):
        """
        Set the camera position for Open3D visualizer by updating view_params.

        Args:
            cam_pos: Camera position as (x, y, z).
            lookat: Point the camera looks at as (x, y, z).
            up: Up vector (default [0, 0, 1]).
            zoom: Zoom value (if None, computed from cam_pos and lookat distance).

        Returns:
            view_params dict for Open3D visualizer.
        """

        cam_pos = np.array(cam_pos)
        lookat = np.array(lookat)
        up = np.array(up)
        front = cam_pos - lookat
        front = front / np.linalg.norm(front)
        up = up / np.linalg.norm(up)

        # Compute zoom if not provided
        if zoom is None:
            dist = np.linalg.norm(cam_pos - lookat)
            # Open3D's zoom is not a direct distance, but this heuristic works for most scenes
            zoom = dist * 1  # Adjust this factor as needed

        self.front = front.tolist()
        self.lookat = lookat.tolist()
        self.up = up.tolist()
        self.zoom = zoom

        return self


class O3DViewer:
    """
    Interactive Open3D viewer with a non-blocking rendering loop.
    Usage:
        viewer = O3DViewer(geometries, ...)
        for ...:
            # update geometry in-place or replace
            viewer.update()
        viewer.close()
    """

    def __init__(
        self,
        geometries: list[o3d.geometry.Geometry3D] = [],
        scene_list: list[list[o3d.geometry.Geometry3D]] = [],
        config: O3DViewerConfig = None,
        origin_frame_scale=0.0,
        camera_frame_scale=0.0,
    ):
        if config is None:
            config = O3DViewerConfig()

        self.origin_frame_scale = origin_frame_scale
        self.camera_frame_scale = camera_frame_scale

        # self.vis = o3d.visualization.Visualizer()
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(
            width=config.width, height=config.height, left=config.left, top=config.top
        )
        self.is_closed = False

        # Doesn't seem to be able to differentiate between upper and lower case keys

        print("Press [SPACE] or [ESC] to end blocking")

        def space_callback(vis):
            # print("Space bar pressed!")
            self._space_pressed = True
            return False  #

        print("Press [T] to switch between scenes")

        def T_callback(vis):
            if self.scene_list:
                self.scene_ptr += 1
                if self.scene_ptr >= len(self.scene_list):
                    self.scene_ptr = 0  # Loop back to start
                self.update(self.scene_list[self.scene_ptr])
            return False

            # Register key callbacks

        self.vis.register_key_callback(32, space_callback)  # Space bar
        self.vis.register_key_callback(ord("T"), T_callback)  # 'T'

        self.geometries = geometries  # holds all the currently loaded geometries
        self.scene_list = scene_list
        if len(self.scene_list) == 0:
            self.scene_list = [self.geometries]
        self.scene_ptr = 0

        self.show_camera_frame = True

        self.set_inital_view = False  # flag as it doesn't seem to set here in init, so need to set agian once later
        self.load_config(config)
        self.update_scenes(self.scene_list)

        self.window = False
        self.is_closed = False

    def _update_geometries(self, new_geometries):
        # Remove previous geometries
        if not new_geometries:
            return

        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=self.origin_frame_scale
        )
        new_geometries.append(origin_frame)

        # store current view parameters to restore after adding geometries
        view_ctl = self.vis.get_view_control()
        params = view_ctl.convert_to_pinhole_camera_parameters()

        for g in self.geometries:
            self.vis.remove_geometry(g)
        self.geometries = []
        for g in new_geometries:
            self.vis.add_geometry(g)
            self.geometries.append(g)

        view_ctl.convert_from_pinhole_camera_parameters(params)

    def load_config(self, config: O3DViewerConfig):
        # Only update attributes if new values are provided
        if self.is_closed:
            raise RuntimeError("Viewer window is closed.")

        ctr = self.vis.get_view_control()
        ctr.set_front(config.front)
        ctr.set_lookat(config.lookat)
        ctr.set_up(config.up)
        ctr.set_zoom(config.zoom)

        render_option = self.vis.get_render_option()
        render_option.point_size = config.point_size

    def update_scenes(self, scene_list, reset_view=False):
        self.scene_list = scene_list
        # self.scene_ptr = 0 # disable to not reset to first scene
        self.update(geometries=self.scene_list[self.scene_ptr], reset_view=reset_view)

    def update(self, geometries=None, point_size=None, reset_view=False):
        """
        Update the geometries in the viewer.
        If geometries is None, update the currently tracked geometries in-place.
        If geometries is provided, replace the geometries.
        Optionally update camera position.
        """
        if self.is_closed:
            raise RuntimeError("Viewer window is closed.")
        self._update_geometries(geometries)

        if (
            not self.set_inital_view or reset_view
        ):  # to fix behaviour where inital view control is not being set
            self.set_inital_view = True
            self.load_config(self.config)

    def add_geometries(self, geometry):
        """
        Add a single geometry to the viewer.
        """
        if self.is_closed:
            raise RuntimeError("Viewer window is closed.")
        if not isinstance(geometry, list):
            geometry = [geometry]

        # store current view parameters to restore after adding geometries
        view_ctl = self.vis.get_view_control()
        params = view_ctl.convert_to_pinhole_camera_parameters()

        for g in geometry:
            self.vis.add_geometry(g)
            # self.vis.add_geometry(g, reset_bounding_box=False) This makes it stop working for some reason..
            self.geometries.append(g)

        view_ctl.convert_from_pinhole_camera_parameters(params)

        # self._set_view_control()

    def run(
        self,
        update_callback=None,
        n_iter=None,
        duration=5.0,
        blocking=False,
        break_on_esc=False,
    ):
        """
        Custom rendering loop.
        If update_callback is provided, it is called every iteration.
        If n_iter is not None, runs for n_iter iterations.
        If duration is not None, runs for the specified number of seconds.
        If both are None, runs until the window is closed.
        If auto_close is True, the window is closed at the end. If False, the window stays open for future updates.
        """

        if self.is_closed:
            raise RuntimeError("Viewer window is closed.")

        self._space_pressed = False

        i = 0
        start_time = time.time()
        while True:
            # if update_callback is not None:
            #     update_callback(self, i)

            for g in self.geometries:
                self.vis.update_geometry(g)
            # esc closing doesn't work beacuse once window is closed it doesn't reset
            # if not self.vis.poll_events():

            success = self.vis.poll_events()
            self.vis.update_renderer()

            i += 1
            if blocking:
                if self._space_pressed:
                    break
                if break_on_esc and not success:
                    break
                continue
            if n_iter is not None and i >= n_iter:
                break
            if duration is not None and (time.time() - start_time) >= duration:
                break

    def close(self):
        if not self.is_closed:
            self.vis.destroy_window()
            self.is_closed = True


def o3d_viewer(
    geometries: List[o3d.geometry.Geometry3D],
    config: O3DViewerConfig = None,
    origin_frame_scale=0.0,
):
    """
    Visualize a list of Open3D geometries with a specified point size and axis length.

    Args:
        geometries: List of Open3D geometry objects to visualize.
        point_size: Size of points in the point cloud.
        axis_length: Length of the coordinate frame arrows.
    """
    _config = config  # simple logic to deal with case if no config passed
    if _config is None:
        _config = O3DViewerConfig()

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        width=_config.width,
        height=_config.height,
        left=_config.left,  # x position in pixels
        top=_config.top,  # y position in pixels
    )

    if origin_frame_scale > 0.0:
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=origin_frame_scale
        )
        geometries.append(origin_frame)

    for g in geometries:
        vis.add_geometry(g)

    # if config is not None:

    #     render_option = vis.get_render_option()
    #     render_option.point_size = config.point_size

    #     # Set camera parameters from view_params
    #     ctr = vis.get_view_control()
    #     ctr.set_front(config.front)
    #     ctr.set_lookat(config.lookat)
    #     ctr.set_up(config.up)
    #     ctr.set_zoom(config.zoom)

    vis.run()
    vis.destroy_window()


# endregion Viewers

# region Model Factory


def calculate_zy_rotation_for_arrow(vec):
    gamma = np.arctan2(vec[1], vec[0])
    Rz = np.array(
        [
            [np.cos(gamma), -np.sin(gamma), 0],
            [np.sin(gamma), np.cos(gamma), 0],
            [0, 0, 1],
        ]
    )

    vec = Rz.T @ vec

    beta = np.arctan2(vec[0], vec[2])
    Ry = np.array(
        [[np.cos(beta), 0, np.sin(beta)], [0, 1, 0], [-np.sin(beta), 0, np.cos(beta)]]
    )
    return Rz, Ry


def create_arrow(
    end,
    origin=np.array([0.0, 0.0, 0.0]),
    thickness_scale=1,
    uniform_thickness=True,
    color=[0.5, 0.0, 0.5],
) -> o3d.geometry.TriangleMesh:
    assert not np.all(end == origin)
    vec = end - origin
    length = np.linalg.norm(vec)
    if length < 1e-9:
        raise ValueError("Arrow length is zero or too small.")

    # Thickness control
    if uniform_thickness:
        radius = 0.02 * thickness_scale  # fixed thickness
    else:
        radius = (length / 30.0) * thickness_scale  # thickness_scales with arrow length

    cone_height = 0.2 * length * thickness_scale
    cyl_height = length - cone_height

    # height of arrow should match height of frame
    mesh = o3d.geometry.TriangleMesh.create_arrow(
        cone_radius=2.0 * radius,
        cone_height=cone_height,
        cylinder_radius=radius,
        cylinder_height=cyl_height,
    )

    Rz, Ry = calculate_zy_rotation_for_arrow(vec)
    mesh.rotate(Ry, center=np.array([0, 0, 0]))
    mesh.rotate(Rz, center=np.array([0, 0, 0]))
    mesh.translate(origin)
    mesh.paint_uniform_color(color)
    return mesh


def draw_vectors(
    vectors,
    origin=np.array([0.0, 0.0, 0.0]),
    colors=None,
    uniform_thickness=True,
    scale=1.0,
    origin_frame_scale=1.0,
    config=None,
    show=True,
) -> List[o3d.geometry.TriangleMesh]:

    geometries = []

    if colors is None:
        colors = [(0.5, 0.0, 0.5)] * len(vectors)  # violet RGB color
        colors = np.array(colors)

    elif len(colors) == 3 and not isinstance(colors[0], (list, tuple, np.ndarray)):
        colors = [colors] * len(vectors)
        colors = np.array(colors)

    for i, v in enumerate(vectors):
        arrow = create_arrow(v, origin, scale, uniform_thickness, color=colors[i])
        geometries.append(arrow)

    # Add reference frame
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=origin_frame_scale)
    geometries.append(frame)

    if show:
        # o3d.visualization.draw_geometries(geometries)
        o3d_viewer(geometries, config)
    return geometries


def draw_R_list(
    R_list,
    scale=1.0,
    only_z=False,
    show=True,
    origin_frame_scale=1.0,
    config: O3DViewerConfig = None,
    color_by_order=False,
) -> List[o3d.geometry.TriangleMesh]:
    """
    Visualize a list of coordinate frames or just their Z axes using Open3D.

    Parameters
    ----------
    R_list : List[np.ndarray] or np.ndarray (3, 3) or (N, 3, 3)
        List of 3x3 rotation matrices or a single one.
    scale : float
        Scale of the coordinate frames or arrows.
    only_z : bool
        If True, only the Z-axis (as an arrow) is visualized.
    """
    geometries = []

    # Normalize input to a list of 3x3 matrices
    if isinstance(R_list, np.ndarray):
        if R_list.ndim == 2:
            R_list = [R_list]
        elif R_list.ndim == 3:
            R_list = list(R_list)
    elif not isinstance(R_list, list):
        R_list = [R_list]

    num_frames = len(R_list)

    for i, R in enumerate(R_list):
        # Compute color gradient from blue to white if color_by_order is True
        if color_by_order and num_frames > 1:
            # Interpolate from blue [0, 0, 1] to white [1, 1, 1]
            t = i / (num_frames - 1)  # 0 to 1
            color = [t, t, 1.0]  # Red and Green go from 0 to 1, Blue stays at 1
        else:
            color = None

        if only_z:
            if color is not None:
                arrow = create_arrow(R[:, 2], np.array([0, 0, 0]), scale, color=color)
            else:
                arrow = create_arrow(R[:, 2], np.array([0, 0, 0]), scale)
            geometries.append(arrow)
        else:
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)
            frame.rotate(R, center=(0, 0, 0))
            # if color is not None:
            # frame.paint_uniform_color(color)
            geometries.append(frame)

    # Global reference frame
    if origin_frame_scale > 0.0:
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=origin_frame_scale
        )
        geometries.append(origin_frame)

    if show:
        o3d_viewer(geometries, config)
    return geometries


def draw_frames(
    frame_list: list[np.ndarray],
    frame_scale: float | list[float] = 1.0,
    only_z: bool = False,
    config: O3DViewerConfig = None,
    show: bool = True,
) -> List[o3d.geometry.TriangleMesh]:
    geometries = []

    if isinstance(frame_scale, float):
        frame_scale = [frame_scale] * len(frame_list)

    for i, frame_i in enumerate(frame_list):
        if only_z:
            arrow = create_arrow(
                frame_i[:, 2], frame_i[:3, 3], frame_scale[i], color=[0.0, 0.0, 1.0]
            )
            geometries.append(arrow)
        else:
            if frame_scale[i] > 0.0:
                frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=frame_scale[i]
                )
                frame.transform(frame_i)
                geometries.append(frame)

    if show:
        o3d_viewer(geometries, config)

    return geometries


def draw_path(
    waypoints: list[np.ndarray],
    show_lines=True,
    line_color: list[float] | list[list[float]] | np.ndarray = [0.5, 0.5, 0.5],
    point_radius=0.0025,
    point_color: list[float] | list[list[float]] | np.ndarray = [0.0, 0.0, 0.0],
    config: O3DViewerConfig = None,
    show: bool = True,
) -> List[o3d.geometry.TriangleMesh]:
    geometries = []

    points = []
    line_start_end_indices = []

    if len(line_color) == 3 and not isinstance(
        line_color[0], (list, tuple, np.ndarray)
    ):
        line_color = [line_color] * (len(waypoints) - 1)
    line_color = np.array(line_color)

    if len(point_color) == 3 and not isinstance(
        point_color[0], (list, tuple, np.ndarray)
    ):
        point_color = [point_color] * len(waypoints)
    point_color = np.array(point_color)

    assert len(line_color) == len(waypoints) - 1
    assert len(point_color) == len(waypoints)

    for i, waypoint in enumerate(waypoints):

        xyz = waypoint[:3, 3].tolist()

        if point_radius > 0.0:
            point = o3d.geometry.TriangleMesh.create_sphere(radius=point_radius)
            point.paint_uniform_color(point_color[i])
            point.translate(xyz)
            geometries.append(point)

        points.append(xyz)
        if i > 0:
            line_start_end_indices.append([i - 1, i])

    if show_lines:  # no support for setting line width at the moment...
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(line_start_end_indices)
        line_set.colors = o3d.utility.Vector3dVector(line_color)
        geometries.append(line_set)

    if show:
        o3d_viewer(geometries, config)
    return geometries


def create_centered_box(pose_matrix, size, color, center_shift=[0, 0, 0]):
    """
    Creates a box centered at the origin with a given size and color.
    The standard o3d box is from the edge

    By default center_shift is [0.5, 0.5, 0.5] which means the box is centered at the origin.
    If you want the box to be centered at the origin, you can set center_shift to [0, 0, 0].
    """
    box = o3d.geometry.TriangleMesh.create_box(
        width=size[0], height=size[1], depth=size[2]
    )
    box.paint_uniform_color(color)
    box.compute_vertex_normals()

    # Shift the box to center it around the origin
    transformation = np.eye(4)
    transformation[:3, 3] = [
        -size[0] * 0.5 + center_shift[0],
        -size[1] * 0.5 + center_shift[1],
        -size[2] * 0.5 + center_shift[2],
    ]
    box.transform(transformation)

    # Then transform to desired pose
    box.transform(pose_matrix)

    return box


def create_centered_cylinder(
    pose_matrix, radius, height, color, center_shift=[0, 0, 0]
):
    """
    Creates a cylinder with a given radius, height, and color.
    The standard o3d cylinder is centered at origin and aligned along Z-axis.

    By default center_shift is [0.5, 0.5, 0.5] which keeps the cylinder centered in XY,
    and centered along Z-axis (height direction).
    center_shift[2] controls the Z-axis shift: 0.5 centers it, 0.0 places base at origin.
    """
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
    cylinder.paint_uniform_color(color)
    cylinder.compute_vertex_normals()

    # Shift the cylinder to adjust centering
    # O3D cylinder is already centered in XY, but extends from -height/2 to +height/2 in Z
    transformation = np.eye(4)
    transformation[:3, 3] = [center_shift[0], center_shift[1], center_shift[2]]
    cylinder.transform(transformation)

    # Then transform to desired pose
    cylinder.transform(pose_matrix)

    return cylinder


# TODO update the grippers to work for tool0 ref vs tcp_world vs tcp_tool
# Defines this symetrical to the TCP, which is useful for seeing TCP location but not viz of wrist offset
def parallel_gripper_model(
    T_world_to_tcp,
    grasp_width,
    thickness=0.01,
    finger_height=0.06,
    finger_width=0.02,
    palm_thickness=0.01,
    color=[1.0, 0.0, 0.0],
    frame_scale=0.025,
    z_into_table=False,
    opening_axis="x",
):
    """
    T_world_to_tcp is defined for tcp_world frame (between fingers tips, with z_axis pointing into hand)
    """

    if z_into_table:
        z_dir = -1
    else:
        z_dir = 1

    if opening_axis == "x":
        T_world_to_tcp_rot = T_world_to_tcp
    elif opening_axis == "y":
        T_world_to_tcp_rot = xyzrpy_offset(
            T_world_to_tcp, rpy=(0, 0, np.pi / 2), local=True
        )

    if grasp_width == 0:
        grasp_width = 0.00001  # avoid errors when drawing boxes

    # Right finger
    T_gripper_finger_r = xyzrpy_to_matrix(
        [grasp_width / 2 + thickness / 2, 0, z_dir * finger_height / 2], [0, 0, 0]
    )
    T_world_finger_r = T_world_to_tcp_rot @ T_gripper_finger_r
    finger_r = create_centered_box(
        T_world_finger_r, [thickness, finger_width, finger_height], color
    )

    # Left finger
    T_gripper_finger_l = xyzrpy_to_matrix(
        [-grasp_width / 2 - thickness / 2, 0, z_dir * finger_height / 2], [0, 0, 0]
    )
    T_world_finger_l = T_world_to_tcp_rot @ T_gripper_finger_l
    finger_l = create_centered_box(
        T_world_finger_l, [thickness, finger_width, finger_height], color
    )

    # Palm
    T_gripper_palm = xyzrpy_to_matrix(
        [0, 0, z_dir * (finger_height + palm_thickness / 2)], [0, 0, 0]
    )
    T_world_palm = T_world_to_tcp_rot @ T_gripper_palm
    palm = create_centered_box(
        T_world_palm, [grasp_width, finger_width, palm_thickness], color
    )

    # Wrist, aka. fork handle
    T_gripper_wrist = xyzrpy_to_matrix(
        [0, 0, z_dir * (finger_height + palm_thickness / 2)], [0, 0, 0]
    )
    T_world_wrist = T_world_to_tcp_rot @ T_gripper_wrist
    wrist = create_centered_box(
        T_world_wrist,
        [finger_width, finger_width, finger_height * 0.2],
        center_shift=[0, 0, (finger_height * 0.2) / 2],
        color=color,
    )

    geometries = [finger_r, finger_l, palm, wrist]
    if frame_scale > 0.0:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_scale)
        frame.transform(T_world_to_tcp)
        geometries.append(frame)

    return geometries


def skeleton_gripper_model(
    T_world_to_tcp,
    grasp_width=0.03,
    color=[1.0, 0.0, 0.0],
    thickness_scale=1.0,
    frame_scale=0.025,
    z_into_table=False,
    opening_axis="x",
):

    thickness = thickness_scale * 0.0025
    finger_height = thickness_scale * 0.075
    finger_width = thickness_scale * 0.0025
    palm_thickness = thickness_scale * 0.0025

    return parallel_gripper_model(
        T_world_to_tcp,
        grasp_width,
        thickness,
        finger_height,
        finger_width,
        palm_thickness,
        color,
        frame_scale,
        z_into_table,
        opening_axis,
    )


# pose is of the wrist, so helpful to see offsets..
def claw_gripper_model(
    T_world_to_tcp,
    opening_angle,
    servo_diameter=0.05,
    finger_length=0.12,
    finger_thickness=0.01,
    finger_width=0.02,
    wrist_offset=0.0,
    color=[1.0, 0.0, 0.0],
    frame_scale=0.025,
    z_into_table=False,
    opening_axis="x",
):

    if z_into_table:
        z_dir = -1
    else:
        z_dir = 1

    if opening_axis == "x":
        T_world_to_tcp_rot = T_world_to_tcp
    elif opening_axis == "y":
        T_world_to_tcp_rot = xyzrpy_offset(
            T_world_to_tcp, rpy=(0, 0, np.pi / 2), local=True
        )

    T_claw_center = xyzrpy_offset(
        T_world_to_tcp_rot,
        xyz=(0, 0, z_dir * (finger_length)),
        rpy=(np.pi / 2, 0, 0),
        local=True,
    )

    # base = create_centered_box(T_world_to_tcp, [servo_diameter, finger_width, servo_diameter], color, center_shift=[0.5,0.5,0])
    servo = create_centered_cylinder(
        T_claw_center, servo_diameter / 2, finger_width * 0.8, color
    )  # reduce thickness a bit by 0.8 so that you can see the fingers

    # Finger 1 - rotate +angle around Z, then translate along x
    T_finger_1 = xyzrpy_offset(
        T_claw_center,
        xyz=(finger_thickness / 2, -z_dir * (finger_length / 2), 0),
        local=True,
    )
    finger_1 = create_centered_box(
        T_finger_1, [finger_thickness, finger_length, finger_width], color
    )

    # Beacuse we need to rotate it, we actually want to offset the visual box and keep this centered...
    T_claw_center_rot = xyzrpy_offset(
        T_claw_center, rpy=(0, 0, -z_dir * opening_angle), local=True
    )
    T_finger_2 = xyzrpy_offset(
        T_claw_center_rot,
        xyz=(-finger_thickness / 2, -z_dir * (finger_length / 2), 0),
        local=True,
    )
    finger_2 = create_centered_box(
        T_finger_2, [finger_thickness, finger_length, finger_width], color
    )

    # geometries = [base, claw_frame, finger_1, finger_2]

    geometries = [servo, finger_1, finger_2]

    if frame_scale > 0.0:

        claw_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_scale)
        claw_frame.transform(T_claw_center)
        geometries.append(claw_frame)

        T_claw_center_half_rot = xyzrpy_offset(
            T_claw_center, rpy=(0, 0, -z_dir * opening_angle / 2), local=True
        )
        T_virtual_tip = xyzrpy_offset(
            T_claw_center_half_rot,
            xyz=(0, -z_dir * finger_length, 0),
            rpy=(-np.pi / 2, 0, 0),
            local=True,
        )
        virtual_tip_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=frame_scale
        )
        virtual_tip_frame.transform(T_virtual_tip)
        geometries.append(virtual_tip_frame)

        tcp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_scale)
        tcp_frame.transform(T_world_to_tcp)
        geometries.append(tcp_frame)

    return geometries


def draw_points(
    points: List[np.ndarray],
    colors: List[np.ndarray] = None,
    point_size=5.0,
    origin_frame_scale=1.0,
    extra_geometries: List[o3d.geometry.Geometry3D] = [],
    config: O3DViewerConfig = None,
    show=True,
) -> List[o3d.geometry.Geometry3D]:
    """
    Visualize a point cloud with coordinate frame(s).

    Args:
        points: (N, 3) numpy array of point cloud.
        origin_frame_scale: Size of the coordinate frame arrows.
        pose_matrix: Optional 4x4 transform to visualize an additional frame.
    """
    geometries = []
    geometries.extend(extra_geometries)

    if colors:
        assert len(points) == len(colors), "Points and colors must have the same length"

    for i, points in enumerate(points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        if colors:
            if isinstance(colors[i], np.ndarray):
                pcd.colors = o3d.utility.Vector3dVector(colors[i])
            else:  # assumes its a list/tuple (R, G, B) ex. (0.1, 0.1, 0.1)
                pcd.paint_uniform_color(colors[i])
        geometries.append(pcd)

    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=origin_frame_scale
    )
    geometries.append(origin_frame)

    if show:
        o3d_viewer(geometries, config)
    return geometries


def draw_rgbd_mask_scenes(
    camera_info,
    color: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    bg_color: np.ndarray = None,
    bg_depth: np.ndarray = None,
    origin_frame_scale=1.0,
    config: O3DViewerConfig = None,
):
    """
    Display 4 Open3D scenes:
    1. Full color point cloud.
    2. Full color point cloud, with non-mask pixels set to black.
    3. Full color background point cloud.
    4. Grey background point cloud, and full color point cloud with non-mask pixels set to black.
    """
    scene_list = []

    if np.nanmedian(depth) > 10:
        assert False, "Depth values in mm"

    # 1. Full color point cloud
    xyz = rgbd_2_xyz(color, depth, camera_info)
    pc = rgbxyz_2_pc(color, xyz)
    scene_list.append([pc])

    # 2. Full color point cloud, non-mask pixels black
    color_masked = color.copy()
    color_masked[mask == 0] = 0
    masked_pc = rgbxyz_2_pc(color_masked, xyz)
    scene_list.append([masked_pc])

    if bg_depth is not None:
        if np.nanmedian(bg_depth) > 10:
            assert False, "Depth values in mm"

        # 3. Full color background point cloud
        if bg_color is not None:
            bg_xyz = rgbd_2_xyz(bg_color, bg_depth, camera_info)
            bg_pc = rgbxyz_2_pc(bg_color, bg_xyz)
        else:
            bg_xyz = depth_2_xyz(bg_depth, camera_info)
            bg_pc = xyz_2_pc(bg_xyz)

        scene_list.append([bg_pc])

        # 4. Grey bg pc, and full color pc with non-mask pixels black
        grey_bg_pc = xyz_2_pc(bg_xyz, [0.1, 0.1, 0.1])
        scene_list.append([masked_pc, bg_pc])

    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=origin_frame_scale
    )
    for scene in scene_list:
        o3d_viewer(scene + [origin_frame], config)


# endregion Model Factory


def main(args=None):

    config = O3DViewerConfig()
    config.update_4k_window()
    config.zoom = 0.5

    center_pose = xyzrpy_to_matrix([0, 0, 0], [0, 0, 0])
    size = [0.05, 0.05, 0.05]
    color = [1, 0, 0]
    # o3d_viewer([create_centered_box(center_pose, size, color, center_shift=[0, 0, 0])], origin_frame_scale=0.1)
    # o3d_viewer([create_centered_box(center_pose, size, color, center_shift=[0, 0, size[2]/2])], origin_frame_scale=0.1)

    # o3d_viewer([create_centered_cylinder(center_pose, size[0], size[1], color, center_shift=[0, 0, 0])], origin_frame_scale=0.1)
    # o3d_viewer([create_centered_cylinder(center_pose, size[0], size[1], color, center_shift=[0, 0, size[2]/2])], origin_frame_scale=0.1)
    # o3d_viewer([create_centered_cylinder(center_pose, size[0], size[1], color, center_shift=[0, 0, -size[2]/2])], origin_frame_scale=0.1)

    # parallel_gripper = parallel_gripper_model(xyzrpy_to_matrix([0,0,0.1],[np.pi,0,0]), 20/1000, frame_scale=0.1, z_into_table=False, opening_axis='y')
    claw_gripper = claw_gripper_model(
        xyzrpy_to_matrix([0, 0.2, 0.1], [np.pi, 0, 0]),
        np.deg2rad(45),
        frame_scale=0.1,
        opening_axis="x",
        z_into_table=True,
    )
    # claw_gripper = claw_gripper_model(xyzrpy_to_matrix([0,0.2,0.1],[0,0,0]), np.deg2rad(45), frame_scale=0.1, opening_axis='x', z_into_table=False)

    # Test center box with shift
    T_table = xyzrpy_to_matrix([0, 0, 0], [0, 0, 0])
    table = create_centered_box(
        T_table, [0.5, 0.5, 0.05], [0, 0, 0], center_shift=[0, 0, -0.05 / 2]
    )

    o3d_viewer([table] + claw_gripper, config, origin_frame_scale=0.1)

    # # Sample random points in a wave
    # N = 200
    # x = np.linspace(-0.2, 0.2, N)
    # y = np.linspace(-0.2, 0.2, N)
    # xx, yy = np.meshgrid(x, y)
    # zz = 0.05 * np.sin(10 * xx) * np.cos(10 * yy)
    # points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)

    # # Assign colors based on z value (normalized)
    # z_norm = (zz.ravel() - zz.min()) / (zz.max() - zz.min())
    # colors = np.stack([z_norm, 1-z_norm, 0.5*np.ones_like(z_norm)], axis=1)

    # # Test the draw_points
    # draw_points([points], [colors], point_size=5.0, axis_length=0.1)
    # # draw_points([points], [colors], point_size=3.0, axis_length=0.1, extra_geometries=[frame, table]+gripper)

    # # Uncomment to show the original geometry viewer
    # # o3d.visualization.draw_geometries([frame, table]+gripper)


# region - Mesh Processing Helpers


def analyze_mesh(
    mesh: o3d.geometry.TriangleMesh, name: str = "Mesh", print_info: bool = True
) -> dict:
    """Analyze mesh and return statistics.

    Args:
        mesh: Open3D triangle mesh
        name: Name for the mesh (used in printed output)
        print_info: Whether to print the analysis

    Returns:
        Dictionary with mesh statistics
    """
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    # Estimate memory size
    mem_vertices = vertices.nbytes / 1024 / 1024  # MB
    mem_triangles = triangles.nbytes / 1024 / 1024 if len(triangles) > 0 else 0
    total_mem = mem_vertices + mem_triangles

    stats = {
        "name": name,
        "num_vertices": len(vertices),
        "num_triangles": len(triangles),
        "memory_mb": total_mem,
        "has_vertex_normals": mesh.has_vertex_normals(),
        "has_triangle_normals": mesh.has_triangle_normals(),
        "has_vertex_colors": mesh.has_vertex_colors(),
    }

    if print_info:
        print(f"\n  {name}:")
        print(f"    Vertices : {stats['num_vertices']:,}")
        print(f"    Triangles: {stats['num_triangles']:,}")
        print(f"    Memory   : {stats['memory_mb']:.2f} MB")
        print(
            f"    Normals  : V={stats['has_vertex_normals']}, T={stats['has_triangle_normals']}"
        )

    return stats


def create_lowres_mesh(
    mesh: o3d.geometry.TriangleMesh,
    target_faces: int = None,
    reduction_ratio: float = 0.01,
    min_faces: int = 500,
    max_faces: int = 10000,
    use_proportional: bool = True,
    verbose: bool = True,
) -> o3d.geometry.TriangleMesh:
    """Create a low-resolution version of a mesh.

    Args:
        mesh: Input high-resolution mesh
        target_faces: Target number of triangles (used if use_proportional=False)
        reduction_ratio: Proportion of original triangles to keep (e.g., 0.02 = 2%)
        min_faces: Minimum number of faces (lower bound)
        max_faces: Maximum number of faces (upper bound)
        use_proportional: If True, reduces proportionally; if False, uses target_faces
        verbose: Whether to print progress

    Returns:
        Low-resolution mesh
    """
    # Check if mesh has triangles
    if len(mesh.triangles) == 0:
        if verbose:
            print("⚠️  Point cloud detected, converting to mesh first...")
        mesh.estimate_normals()
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            mesh, depth=9
        )

    curr_triangles = len(mesh.triangles)

    # Calculate target faces
    if use_proportional:
        target = int(curr_triangles * reduction_ratio)
        target = max(min_faces, min(max_faces, target))
    else:
        target = target_faces if target_faces else 10000

    if verbose:
        print(f"  Decimating {curr_triangles:,} → {target:,} triangles...")

    # Decimate mesh
    if curr_triangles > target:
        mesh_lowres = mesh.simplify_quadric_decimation(target)
    else:
        if verbose:
            print(
                f"  ℹ️  Mesh already has {curr_triangles:,} triangles, skipping decimation"
            )
        mesh_lowres = mesh

    # Compute normals (required for many file formats like STL)
    mesh_lowres.compute_vertex_normals()
    mesh_lowres.compute_triangle_normals()

    return mesh_lowres


def save_mesh_with_verification(
    mesh: o3d.geometry.TriangleMesh,
    output_path: str | Path,
    verify: bool = True,
    verbose: bool = True,
) -> bool:
    """Save mesh and optionally verify it was saved correctly.

    Args:
        mesh: Mesh to save
        output_path: Path to save the mesh
        verify: Whether to load and verify the saved mesh
        verbose: Whether to print progress

    Returns:
        True if save (and verification if enabled) succeeded
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Save mesh
    success = o3d.io.write_triangle_mesh(str(output_path), mesh)

    if not success:
        if verbose:
            print(f"❌ Failed to save mesh to {output_path}")
        return False

    if verbose and not verify:
        print(f"✓ Mesh saved: {output_path}")

    if verify:
        # Load and verify
        mesh_test = o3d.io.read_triangle_mesh(str(output_path))

        if len(mesh_test.vertices) == 0:
            if verbose:
                print(f"❌ Saved file is empty: {output_path}")
            return False

        if len(mesh_test.triangles) == 0:
            if verbose:
                print(f"⚠️  Saved mesh has no triangles: {output_path}")
            # This might still be okay for point clouds

        if verbose:
            print(f"✓ Mesh saved and verified: {output_path}")
            print(
                f"  Loaded: {len(mesh_test.vertices):,} vertices, {len(mesh_test.triangles):,} triangles"
            )

    return True


def visualize_meshes_side_by_side(
    meshes: list[o3d.geometry.TriangleMesh],
    labels: list[str] = None,
    spacing: float = 0.3,
    window_name: str = "Mesh Comparison",
) -> None:
    """Visualize multiple meshes side-by-side in Open3D viewer.

    Args:
        meshes: List of meshes to visualize
        labels: Optional list of labels for each mesh
        spacing: Horizontal spacing between meshes
        window_name: Name for the visualization window
    """
    if labels is None:
        labels = [f"Mesh {i}" for i in range(len(meshes))]

    # Create copies and offset them
    meshes_to_show = []
    for i, mesh in enumerate(meshes):
        mesh_copy = o3d.geometry.TriangleMesh(mesh)
        mesh_copy.translate([i * spacing, 0, 0])
        meshes_to_show.append(mesh_copy)

    # Add coordinate frames at each position
    geometries = []
    for i, (mesh, label) in enumerate(zip(meshes_to_show, labels)):
        geometries.append(mesh)
        # Add a coordinate frame at the base of each mesh
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        frame.translate([i * spacing, 0, 0])
        geometries.append(frame)

    # Visualize
    print(f"\n{'='*70}")
    print(f"Visualizing {len(meshes)} meshes side-by-side")
    print(f"{'='*70}")
    for i, label in enumerate(labels):
        print(f"  Position {i}: {label}")
    print(f"\nSpacing: {spacing}m")
    print(f"{'='*70}\n")

    o3d.visualization.draw_geometries(
        geometries, window_name=window_name, width=1920, height=1080
    )


# endregion - Mesh Processing Helpers


if __name__ == "__main__":
    main()
