#!/usr/bin/env python3

# BAM
from .pin_robot_model import PinRobotModel

# PYTHON
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import time
import numpy as np
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from bam.descriptions import RobotDescription

class PinMeshcat():

    @classmethod
    def from_robot_description(cls, rd: 'RobotDescription', zmq_url="", color=None):
        robot_model = PinRobotModel.from_robot_description(rd)
        return cls(robot_model, zmq_url=zmq_url, color=color)

    @classmethod
    def from_urdf_xml(cls, urdf_xml: str, zmq_url="", color=None):
        robot_model = PinRobotModel.from_urdf_xml(urdf_xml)
        return cls(robot_model, zmq_url=zmq_url, color=color)

    @classmethod
    def from_xacro(cls, xacro_path: str, xacro_args: dict, mesh_package_dirs: str | list[str], zmq_url="", color=None):
        robot_model = PinRobotModel.from_xacro(xacro_path, xacro_args, mesh_package_dirs)
        return cls(robot_model, zmq_url=zmq_url, color=color)
    
    @classmethod
    def from_urdf(cls, urdf_path: str, mesh_package_dirs: str | list[str], zmq_url="", color=None):
        robot_model = PinRobotModel.from_urdf(urdf_path, mesh_package_dirs)
        return cls(robot_model, zmq_url=zmq_url, color=color)

    def __init__(self, robot_model: PinRobotModel, zmq_url="", color=None):
        """
        DescriptionArgs:
            pin_model: PinModel with model, collision_model, and visual_model
            zmq_url: example: tcp://127.0.0.1:6000
            color: color of the robot
        """

        print("[UNCONFIGURED] MeshcatClient")

        self.robot_model = robot_model


        self.viz = MeshcatVisualizer(self.robot_model.model, self.robot_model.collision_model, self.robot_model.visual_model)

        self.update_frames = False
        self.color = color
        
        if zmq_url == "":
            self.viz.initViewer(open=True)
        else:
            print(f"Attemping to connect to Meshcat on: {zmq_url}") 
            self.viz.initViewer(zmq_url=zmq_url) # cli: 
            
            print(f"Connected on: {zmq_url}")

        self.robot_prefix = None
        self._robot_loaded = False
        self.load_robot()

        # time.sleep(1.0) #wait for robot to load

        print("[READY] MeshcatClient")

    def sleep_for_load(self):
        time.sleep(0.5) #wait for robot to load

    def wait_for_load(self):
        time.sleep(0.5) #wait for robot to load

    def load_robot(self):
        """(Re)load the robot model into the Meshcat viewer."""
        if self.robot_model.model is None:
            raise ValueError("Model is not set. Cannot load robot.")
        
        # give a unique id so you can have multiple robots in same viewer
        # random_id = np.random.randint(1000, 9999)
        # self.robot_prefix = f"robot_{random_id}"
        # self.viz.loadViewerModel(rootNodeName=self.robot_prefix, visual_color=self.color)
        self.viz.loadViewerModel(visual_color=self.color)
        self.robot_prefix = getattr(self.viz, "viewerRootNodeName", None)

        q = pin.neutral(self.robot_model.model)
        self.viz.display(q)
        self._robot_loaded = True
        # print(f"[LOADED] Robot with prefix: {self.robot_prefix}")

    @property
    def zmq_url(self) -> str:
        """Return the ZMQ URL of the connected Meshcat viewer."""
        return self.viz.viewer.window.zmq_url
    
    def show_frame(self, frame_name, verbose=True):
        """Show a single frame."""
        return self.display_frames([frame_name], verbose)

    def display_frames(self, frame_names, verbose=True):
        """Show multiple frames at the same time.
        
        DescriptionArgs:
            frame_names: List of frame names to display
            verbose: If True, print frame information
        """
        if verbose:
            print(f"\nFrames (n = {self.model.nframes}):")
            for i, frame in enumerate(self.model.frames):
                print(f"  Frame {i}: {frame.name}, type: {frame.type}, parent: {frame.parentJoint}")

        frame_ids = []
        for frame_name in frame_names:
            try:
                frame_id = self.model.getFrameId(frame_name)
                frame_ids.append(frame_id)
                print(f"Showing Frame {frame_name} - {frame_id}")
            except Exception as e:
                print(f"Warning: Frame '{frame_name}' not found: {e}")
                continue

        if frame_ids:
            self.viz.displayFrames(True, frame_ids, axis_length=0.2, axis_width=3)
            self.viz.updateFrames()
            print(f"Displayed {len(frame_ids)} frames")
        else:
            print("No valid frames to display")

    def reset(self):
        self.viz.reset()
        
    def clear_robot(self):
        """Remove the robot model from the Meshcat viewer."""
        if self.robot_prefix:
            self.viz.viewer[self.robot_prefix].delete()
            print(f"[CLEARED] Robot model: {self.robot_prefix}")
            self.robot_prefix = None
            self._robot_loaded = False

    def get_frame_transform(self, frame_name: str) -> np.ndarray:
        """ Get transform of current display """

        #BUG: This won't for for non joint frames unless you manually update them all!
        # Manually update all frame placements so transforms are up-to-date
        # This ensures that get_frame_transform works for non-joint frames as well

        if not hasattr(self.viz, "frame_ids"):
            # required for the self.viz.updateFrames() to work
            self.viz.frame_ids = []
        self.viz.updateFrames()

        frame_id = self.viz.model.getFrameId(frame_name)

        T_world_to_frame = self.viz.data.oMf[frame_id]
        T = np.eye(4)
        T[:3, :3] = T_world_to_frame.rotation
        T[:3, 3] = T_world_to_frame.translation
        return T

    def display(self, q: np.ndarray):

        if not self._robot_loaded:
            print("[INFO] Robot not loaded — loading now.")
            self.load_robot()

        assert(q.shape[0] == self.robot_model.n_dof), f"Expected {self.robot_model.n_dof} joints, got {q.shape[0]}"

        self.viz.display(q)

    # def display_trajectory_path(self, q, name="trajectory_path"):
    #     self.viz.viewer[name].set_object(g.PointsGeometry(q))
    #     self.viz.viewer[name].set_transform(tf.translation_matrix([0, 0, 0]))

    # def clear_trajectory_path(self, name="trajectory_path"):
    #     self.viz.viewer[name].delete()

    def display_trajectory(self, t, q, speed=1):
        last_time = 0.0 
        for i, q_i in enumerate(q):
            time_from_start = t[i]
            sleep_time = time_from_start - last_time
            last_time = time_from_start  

            self.viz.display(q_i)
            time.sleep(sleep_time*(1/speed))  


    # https://github.com/meshcat-dev/meshcat-python/blob/master/src/meshcat/transformations.py#L1252
    # Meshcat leads with w
    def display_path(self, pose_matrix_list: list[np.ndarray], name="path", axis_scale=0.05):
        """
        Display a nav_msgs/Path message as a series of coordinate frames and connecting lines in Meshcat.

        DescriptionArgs:
            path_msg: nav_msgs.msg.Path message
            name_prefix: Prefix for naming frames in Meshcat
            axis_scale: Size of each triad frame in meters
        """
        if not hasattr(self, "path_frames_group_names"):
            self.path_frames_group_names = []

        # unique_id = str(uuid.uuid4())[:8]
        # group_name = f"{name}_{unique_id}"
        group_name = name

        self.path_frames_group_names.append(group_name)

        points = []


        for i, T in enumerate(pose_matrix_list):
    
            # Display triad at pose
            if axis_scale > 0:
                frame_name = f"{group_name}/frame_{i}"
                self.viz.viewer[frame_name].set_object(g.triad(scale=axis_scale))
                self.viz.viewer[frame_name].set_transform(T)

            # Add line segment from last point to current
            if i > 0:
                prev = pose_matrix_list[i - 1][:3, 3]
                curr = pose_matrix_list[i][:3, 3]
                points.append([prev[0], prev[1], prev[2]])
                points.append([curr[0], curr[1], curr[2]])

        if points:
            line_segments = np.array(points).T  # shape (3, N)
            geometry = g.PointsGeometry(line_segments)
            material = g.LineBasicMaterial(color=0x800080)
            lines = g.LineSegments(geometry, material)

            self.viz.viewer[f"{group_name}/lines"].set_object(lines)

        print(f"[DISPLAYED] Path with {len(pose_matrix_list)} poses under: {group_name}")


    def clear_path(self, name="path", clear_all=False):
        """
        Remove all triads associated with a previously displayed path.
        
        DescriptionArgs:
            name_prefix: The prefix used in `display_path`.
        """
        for group_name in self.path_frames_group_names:
            if group_name == name or clear_all:
                self.viz.viewer[group_name].delete()
                print(f"[CLEARED] Path group '{group_name}' removed.")
                self.path_frames_group_names.remove(group_name)

            
    def display_pointcloud(
        self,
        points: np.ndarray,
        colors: np.ndarray = None,
        size: float = 0.01,
        name: str = "point_cloud",
        hide_black_points=False
    ):
        """
        Display a colored point cloud in Meshcat.

        DescriptionArgs:
            points: (3, N) numpy array of XYZ coordinates in world frame
            colors: (3, N) numpy array of RGB values in [0, 1]; defaults to green/red gradient on Z
            size: float point size in meters
            name: Meshcat path name for the point cloud
        """
        if points.shape[0] != 3 and points.shape[1] == 3:
            points = points.T

        if colors is not None and colors.shape[0] != 3:
            if colors.shape[1] == 3:
                colors = colors.T
            elif colors.shape[1] == 4: # Alpha channel not supported
                colors = colors[:, :3].T

        assert points.shape[0] == 3, f"Expected points shape (3, N), got {points.shape}"

        if colors is None:
            num_points = points.shape[1]
            # Default to semi-transparent green: RGBA = [0, 1, 0, 0.5]
            rgba = np.array([[0.0], [1.0], [0.0], [1.0]])  # shape (4, 1)
            colors = np.repeat(rgba, num_points, axis=1)

        if hide_black_points:
            non_black_mask = ~np.all(colors < 0.01, axis=0)  # Threshold for "black"
            points = points[:, non_black_mask]
            colors = colors[:, non_black_mask]

        self.viz.viewer[name].set_object(
            g.PointCloud(position=points, color=colors, size=size)
        )
        self.viz.viewer[name].set_transform(tf.translation_matrix([0, 0, 0]))

    def clear_pointcloud(self, name: str = "point_cloud"):
        self.viz.viewer[name].delete()

    def display_pose_matrix(self, pose_matrix: np.ndarray, name="frame"):
        self.viz.viewer[name].set_object(g.triad(scale=0.2))
        self.viz.viewer[name].set_transform(pose_matrix)

    def clear_pose_matrix(self, name="frame"):
        self.viz.viewer[name].delete()

    def display_xyzrpy(self, xyz, rpy, name="frame"):

        # Convert to 4x4 transform matrix
        T = tf.euler_matrix(*rpy)
        T[:3, 3] = xyz

        # Display coordinate frame (Triad) at this pose
        self.viz.viewer[name].set_object(g.triad(scale=0.2))
        self.viz.viewer[name].set_transform(T)

    def clear_xyzrpy(self, name="frame"):
        """
        Remove a coordinate frame previously displayed with display_xyzrpy.
        
        DescriptionArgs:
            name: The Meshcat path name used when displaying the frame.
        """
        try:
            self.viz.viewer[name].delete()
            print(f"[CLEARED] Coordinate frame '{name}' removed.")
        except:
            print(f"[WARNING] Coordinate frame '{name}' not found in viewer.")

    def display_arrow(self, start_point: np.ndarray, direction: np.ndarray, name: str = "arrow", 
                     color: np.ndarray = None, scale: float = 1.0):
        """
        Display an arrow vector in 3D space.
        
        DescriptionArgs:
            start_point: (3,) numpy array of XYZ coordinates for arrow start
            direction: (3,) numpy array representing the arrow direction (will be normalized)
            name: Meshcat path name for the arrow
            color: (3,) numpy array of RGB values in [0, 1]; defaults to red
            scale: float scale factor for arrow size
        """
        if start_point.shape != (3,):
            raise ValueError(f"start_point must be shape (3,), got {start_point.shape}")
        if direction.shape != (3,):
            raise ValueError(f"direction must be shape (3,), got {direction.shape}")
        
        # Normalize direction vector
        direction_norm = direction / np.linalg.norm(direction)
        
        # Default to red if no color specified, convert to list
        if color is None:
            color = [1.0, 0.0, 0.0]  # Red
        else:
            # Convert numpy array to list if needed
            color = color.tolist() if hasattr(color, 'tolist') else list(color)
        
        # Create arrow geometry
        arrow_length = 0.2 * scale
        arrow_width = 0.02 * scale
        
        # Create a cylinder for the arrow shaft
        shaft = g.Cylinder(arrow_length, arrow_width)
        
        # Create a shorter, wider cylinder for the arrow head (simulating a cone)
        head_length = 0.05 * scale
        head_width = 0.04 * scale
        head = g.Cylinder(head_length, head_width)
        
        # Calculate transform for the shaft (centered at start, pointing in direction)
        # Find rotation to align z-axis with direction
        z_axis = np.array([0, 0, 1])
        if np.allclose(direction_norm, z_axis):
            rotation = np.eye(3)
        elif np.allclose(direction_norm, -z_axis):
            rotation = tf.rotation_matrix(np.pi, [1, 0, 0])[:3, :3]
        else:
            # Use cross product to find rotation axis
            axis = np.cross(z_axis, direction_norm)
            axis = axis / np.linalg.norm(axis)
            angle = np.arccos(np.clip(np.dot(z_axis, direction_norm), -1, 1))
            rotation = tf.rotation_matrix(angle, axis)[:3, :3]
        
        # Transform matrix for shaft
        shaft_transform = np.eye(4)
        shaft_transform[:3, :3] = rotation
        shaft_transform[:3, 3] = start_point + 0.5 * arrow_length * direction_norm
        
        # Transform matrix for head
        head_transform = np.eye(4)
        head_transform[:3, :3] = rotation
        head_transform[:3, 3] = start_point + arrow_length * direction_norm
        
        # Display shaft and head
        self.viz.viewer[f"{name}_shaft"].set_object(g.Mesh(shaft, g.MeshLambertMaterial(color=color)))
        self.viz.viewer[f"{name}_shaft"].set_transform(shaft_transform)
        
        self.viz.viewer[f"{name}_head"].set_object(g.Mesh(head, g.MeshLambertMaterial(color=color)))
        self.viz.viewer[f"{name}_head"].set_transform(head_transform)
        
        print(f"[ARROW] Displayed arrow '{name}' from {start_point} in direction {direction_norm}")

    def clear_arrow(self, name: str = "arrow"):
        """
        Remove an arrow previously displayed with display_arrow.
        
        DescriptionArgs:
            name: The Meshcat path name used when displaying the arrow.
        """
        try:
            self.viz.viewer[f"{name}_shaft"].delete()
            self.viz.viewer[f"{name}_head"].delete()
            print(f"[CLEARED] Arrow '{name}' removed.")
        except:
            print(f"[WARNING] Arrow '{name}' not found in viewer.")

    def display_cube(self, pose: np.ndarray, size=0.1, name: str = "cube", color: np.ndarray = None, opacity: float = 1.0, wireframe: bool = False):
        """
        Display a box/cube at the specified pose with customizable dimensions.
        
        DescriptionArgs:
            pose: 4x4 numpy array representing the pose (position + orientation)
            size: float for uniform cube OR (3,) array/list for [length, width, height]
            name: Meshcat path name for the cube
            color: (3,) numpy array of RGB values in [0, 1]; defaults to blue
            opacity: float in [0, 1] for transparency (1.0 = opaque, 0.0 = fully transparent)
            wireframe: if True, display wireframe edges for better visibility (default: True)
            
        Examples:
            # Uniform cube
            display_cube(pose, size=0.1, name="cube1")
            
            # Box with different dimensions [length, width, height]
            display_cube(pose, size=[0.2, 0.1, 0.05], name="box1")
            
            # Semi-transparent cube without wireframe
            display_cube(pose, size=0.1, name="cube2", opacity=0.5, wireframe=False)
        """
        if pose.shape != (4, 4):
            raise ValueError(f"pose must be shape (4, 4), got {pose.shape}")
        
        # Handle size as either float or 3-element array
        if isinstance(size, (int, float)):
            dimensions = [size, size, size]
        else:
            if len(size) != 3:
                raise ValueError(f"size must be a float or 3-element array [length, width, height], got {size}")
            dimensions = list(size)
        
        # Default to blue if no color specified
        if color is None:
            color = [0.0, 0.5, 1.0]  # Blue
        else:
            # Convert numpy array to list if needed
            color = color.tolist() if hasattr(color, 'tolist') else list(color)
        
        # Convert RGB to integer format for meshcat (0xRRGGBB)
        color_int = int(color[0] * 255) * 256**2 + int(color[1] * 255) * 256 + int(color[2] * 255)
        
        # Create box geometry with [length, width, height]
        box = g.Box(dimensions)
        
        # Display cube with color and shading (MeshPhongMaterial gives nice 3D appearance)
        material = g.MeshPhongMaterial(color=color_int, opacity=opacity, transparent=(opacity < 1.0))
        self.viz.viewer[name].set_object(g.Mesh(box, material))
        self.viz.viewer[name].set_transform(pose)
        
        # Optionally add wireframe edges for better depth perception
        if wireframe:
            wireframe_color = 0x000000  # Black edges
            wireframe_material = g.MeshBasicMaterial(color=wireframe_color, wireframe=True, linewidth=2)
            self.viz.viewer[f"{name}_wireframe"].set_object(g.Mesh(box, wireframe_material))
            self.viz.viewer[f"{name}_wireframe"].set_transform(pose)
        
        print(f"[CUBE] Displayed cube '{name}' at pose with dimensions [L,W,H]: {dimensions}")

    def clear_cube(self, name: str = "cube"):
        """
        Remove a cube previously displayed with display_cube.
        
        DescriptionArgs:
            name: The Meshcat path name used when displaying the cube.
        """
        try:
            self.viz.viewer[name].delete()
            # Also try to delete wireframe if it exists
            try:
                self.viz.viewer[f"{name}_wireframe"].delete()
            except:
                pass  # Wireframe might not exist
            print(f"[CLEARED] Cube '{name}' removed.")
        except:
            print(f"[WARNING] Cube '{name}' not found in viewer.")

    def set_visbility(self, name: str, visible: bool):
        self.viz.viewer[name].set_property("visible", visible)


    # Camera Controls were not added until 2023
    # https://github.com/meshcat-dev/meshcat-python/commit/bf370150eab898c620817f6a829de4a4f78130b8

    # Pip package was last updated in 2021:
    # https://pypi.org/project/meshcat/#history

    # With virtual env active,
    # pip uninstall meshcat
    # and then follow instructions to install from source: https://github.com/meshcat-dev/meshcat-python

    # Actually meshcat doesn't support setting camera z-axis right now
    # https://stackoverflow.com/questions/79048706/how-to-set-camera-z-axis-orientation-in-meshcat-via-drake

    def set_camera_view_xy_plane(self, zoom: float = 1.0):
        self.set_camera_position(np.array([0.0, 0.0, zoom]))
        self.set_camera_target(np.array([0.0, 0.0, 0.0]))

    def check_camera_controls(self):
        if not hasattr(self.viz, "setCameraPosition"):
            print("[WARNING] pip Meshcat version does not have Camera Control methods")
            print("Install from source: https://github.com/meshcat-dev/meshcat-python")
            assert False, "pip Meshcat version does not have Camera Control methods"

    def set_camera_position(self, position: np.ndarray):
        """Set the camera position using Pinocchio's built-in method."""
        self.check_camera_controls()

        self.viz.setCameraPosition(position)
        print(f"[CAMERA] Set position: {position}")

    def set_camera_target(self, target: np.ndarray):
        """Set the camera target using Pinocchio's built-in method."""
        self.check_camera_controls()

        self.viz.setCameraTarget(target)
        print(f"[CAMERA] Set target: {target}")

    def set_camera_preset(self, preset_key: str):
        """Set camera using a predefined preset from Pinocchio."""
        self.viz.setCameraPreset(preset_key)
        print(f"[CAMERA] Set preset: {preset_key}")

    def set_camera_zoom(self, zoom: float):
        """Set camera zoom level."""
        self.check_camera_controls()

        self.viz.setCameraZoom(zoom)
        print(f"[CAMERA] Set zoom: {zoom}") 

    def set_camera_view(self, position: tuple | list | np.ndarray, target: tuple | list | np.ndarray):
        """Set both camera position and target at once."""
        self.set_camera_position(np.array(position))
        self.set_camera_target(np.array(target))
        print(f"[CAMERA] Set view - position: {position}, target: {target}")

