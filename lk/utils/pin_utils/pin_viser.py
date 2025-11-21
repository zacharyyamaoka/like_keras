#!/usr/bin/env python3

# BAM
import time
from typing import TYPE_CHECKING

import numpy as np

# PYTHON
import pinocchio as pin
from bam.utils.interial.interial import com_sphere_radius, equivalent_inertia_box
from pinocchio.visualize import ViserVisualizer

from .pin_robot_model import PinRobotModel

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription

import viser


class PinViser:
    @classmethod
    def from_robot_description(
        cls, rd: "RobotDescription", port="", color=None, auto_add_controls=True
    ):
        robot_model = PinRobotModel.from_robot_description(rd)

        # Get display link names from robot description
        display_frame_names = (
            rd.links.get_display_link_names() if hasattr(rd, "links") else []
        )

        # Get joint positions from robot description
        joint_positions = {}
        if hasattr(rd, "joint_positions") and rd.joint_positions is not None:
            if hasattr(rd.joint_positions, "positions"):
                joint_positions = rd.joint_positions.positions

        return cls(
            robot_model,
            port=port,
            color=color,
            auto_add_controls=auto_add_controls,
            display_frame_names=display_frame_names,
            joint_positions=joint_positions,
        )

    @classmethod
    def from_urdf_xml(cls, urdf_xml: str, port="", color=None, auto_add_controls=True):
        robot_model = PinRobotModel.from_urdf_xml(urdf_xml)
        return cls(
            robot_model, port=port, color=color, auto_add_controls=auto_add_controls
        )

    @classmethod
    def from_xacro(
        cls,
        xacro_path: str,
        xacro_args: dict,
        mesh_package_dirs: str | list[str],
        port="",
        color=None,
        auto_add_controls=True,
    ):
        robot_model = PinRobotModel.from_xacro(
            xacro_path, xacro_args, mesh_package_dirs
        )
        return cls(
            robot_model, port=port, color=color, auto_add_controls=auto_add_controls
        )

    @classmethod
    def from_urdf(
        cls,
        urdf_path: str,
        mesh_package_dirs: str | list[str],
        port="",
        color=None,
        auto_add_controls=True,
    ):
        robot_model = PinRobotModel.from_urdf(urdf_path, mesh_package_dirs)
        return cls(
            robot_model, port=port, color=color, auto_add_controls=auto_add_controls
        )

    def __init__(
        self,
        robot_model: PinRobotModel,
        port="",
        color=None,
        auto_add_controls=True,
        display_frame_names=None,
        joint_positions: dict[str, list[float]] = None,
    ):
        """
        Initialize PinViser visualizer.

        Args:
            robot_model: PinRobotModel with model, collision_model, and visual_model
            port: Port for Viser server (e.g., "tcp://127.0.0.1:6000"). Empty string for default.
            color: Optional color override for the robot
            auto_add_controls: If True (default), automatically adds all control panels:
                              - Joint position control sliders
                              - Inertial properties (COM and inertia)
                              - Visual appearance (opacity, color, wireframe)
                              - Collision visualization (show/hide and opacity)
                              - Frame control (link/joint frames, labels, size)
                              Set to False to manually add controls via add_*_control() methods.
            display_frame_names: Optional list of frame names to display by default.
                                If None, all frames are shown. If a list is provided,
                                only frames in the list are shown by default (with option
                                to override via "Show Display Frames Only" checkbox).
            joint_positions: Optional dict mapping position names to joint configuration arrays.
                           If provided, a dropdown will be added to the joint control panel
                           to quickly select predefined positions (e.g., "ready", "initial").
        """

        print("[UNCONFIGURED] MeshcatClient")

        self.robot_model = robot_model

        self.viz = ViserVisualizer(
            self.robot_model.model,
            self.robot_model.collision_model,
            self.robot_model.visual_model,
        )

        self.update_frames = False
        self.color = color

        if port == "":
            self.viz.initViewer(open=True)
        else:
            print(f"Attemping to connect to Meshcat on: {port}")
            self.viz.initViewer(port=port)  # cli:

            print(f"Connected on: {port}")

        self.robot_prefix = None
        self._robot_loaded = False
        self._slider_handles: list[viser.GuiInputHandle[float]] | None = None
        self._slider_initial_config: list[float] | None = None

        # Store joint positions for dropdown selector
        self._joint_positions = joint_positions if joint_positions else {}

        self.server = self.viz.viewer

        # Inertial visualization tracking
        self._show_com = False
        self._show_inertia = False
        self._com_handles = {}  # link_name -> viser handle
        self._inertia_handles = {}  # link_name -> viser handle
        self._inertial_opacity = 0.5  # Default opacity for inertia boxes (50%)

        # Visual control tracking
        self._visual_alpha = 0.5  # Default 50% opacity
        self._visual_color = None
        self._visual_wireframe = False

        # Collision control tracking
        self._show_collision = False
        self._collision_opacity = 0.5  # Default 50% opacity

        # Frame control tracking
        self._frame_size = 0.05  # Default axis length
        self._show_link_frames = True  # Show link frames by default
        self._show_joint_frames = False
        self._show_link_frame_labels = True  # Show link frame labels by default
        self._show_joint_frame_labels = False
        self._frame_label_size = 0.05
        self._frame_label_handles = {}  # frame_name -> viser label handle
        self._label_font_mode = "screen"  # 'screen' or 'scene'
        self._label_depth_test = True

        # Display frame filtering
        self._display_frame_names = (
            set(display_frame_names) if display_frame_names else None
        )
        self._show_display_frames_only = (
            True  # By default, filter by display frames if provided
        )

        # Get frame name lists
        self._joint_frame_names = set(self.robot_model.get_joint_frame_names())
        self._link_frame_names = set(self.robot_model.get_link_frame_names())

        # Adjust default visibility if display frames are specified
        if self._display_frame_names and len(self._display_frame_names) > 0:
            # Only show frames by default if we have display frames specified
            self._show_link_frames = True

        self.load_robot()

        # time.sleep(1.0) #wait for robot to load

        print("[READY] MeshcatClient")

        # Auto-add all control panels if enabled
        if auto_add_controls:
            self.add_robot_control_sliders()
            self.add_inertial_control()
            self.add_visual_control()
            self.add_collision_control()
            self.add_frame_control()
            print("[AUTO-CONTROLS] All control panels added automatically")

        # Initialize default visual settings
        if self._visual_alpha != 1.0:
            self._update_visual_alpha(self._visual_alpha)

        # Initialize default frame settings
        if self._show_link_frames or self._show_joint_frames:
            self.viz.displayFrames(True)
            self._update_frame_visibility()

            # Apply initial frame size (must be done after displayFrames)
            self._update_frame_size(self._frame_size)

        if self._show_link_frame_labels or self._show_joint_frame_labels:
            self._update_frame_labels()

    def add_robot_control_sliders(self):
        if self._slider_handles is not None:
            return self._slider_handles, self._slider_initial_config

        with self.server.gui.add_folder("Joint position control"):
            # Add joint position dropdown if positions are available
            if self._joint_positions:
                position_names = list(self._joint_positions.keys())
                position_dropdown = self.server.gui.add_dropdown(
                    label="Preset Positions",
                    options=position_names,
                    initial_value=position_names[0] if position_names else "",
                )
                position_dropdown.on_update(
                    lambda _: self._load_joint_position(position_dropdown.value)
                )

            slider_handles: list[viser.GuiInputHandle[float]] = []
            initial_config: list[float] = []

            lower = self.robot_model.lower_limits
            upper = self.robot_model.upper_limits

            for i, joint_name in enumerate(self.robot_model.get_joint_names()):
                # initial_value = (lower[i] + upper[i]) / 2.0
                initial_value = max(lower[i], 0.0)

                slider = self.server.gui.add_slider(
                    label=joint_name,
                    min=lower[i],
                    max=upper[i],
                    step=1e-3,
                    initial_value=initial_value,
                )
                slider.on_update(  # When sliders move, we update the URDF configuration.
                    lambda _: self.display(
                        np.array([slider.value for slider in slider_handles])
                    )
                )
                slider_handles.append(slider)
                initial_config.append(initial_value)

        self._slider_handles = slider_handles
        self._slider_initial_config = initial_config

        return slider_handles, initial_config

    def _load_joint_position(self, position_name: str):
        """Load a preset joint position and update sliders and display."""
        if position_name not in self._joint_positions:
            print(f"[WARNING] Joint position '{position_name}' not found")
            return

        # Get the joint configuration
        q = self._joint_positions[position_name]

        # Convert to numpy array if it's a list
        if isinstance(q, list):
            q = np.array(q)

        # Update sliders if they exist
        if self._slider_handles:
            for i, slider in enumerate(self._slider_handles):
                if i < len(q):
                    slider.value = float(q[i])

        # Update the display
        self.display(q)

        print(f"[JOINT POSITION] Loaded preset: '{position_name}'")

    def add_inertial_control(self):
        """Add checkboxes to toggle center of mass and inertia visualization."""

        with self.server.gui.add_folder("Inertial Properties"):
            # Center of Mass checkbox
            com_checkbox = self.server.gui.add_checkbox(
                label="Show Center of Mass",
                initial_value=self._show_com,
            )
            com_checkbox.on_update(lambda _: self._toggle_com(com_checkbox.value))

            # Inertia checkbox
            inertia_checkbox = self.server.gui.add_checkbox(
                label="Show Inertia",
                initial_value=self._show_inertia,
            )
            inertia_checkbox.on_update(
                lambda _: self._toggle_inertia(inertia_checkbox.value)
            )

            # Inertial opacity slider
            opacity_slider = self.server.gui.add_slider(
                label="Inertial Opacity",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=self._inertial_opacity,
            )
            opacity_slider.on_update(
                lambda _: self._update_inertial_opacity(opacity_slider.value)
            )

        print("[INERTIAL CONTROL] Added inertial property controls")

    def _toggle_com(self, show: bool):
        """Toggle center of mass visualization."""
        self._show_com = show
        if show:
            self._update_com_visualization()
        else:
            self._clear_com_visualization()

    def _toggle_inertia(self, show: bool):
        """Toggle inertia visualization."""
        self._show_inertia = show
        if show:
            self._update_inertia_visualization()
        else:
            self._clear_inertia_visualization()

    def _update_inertial_opacity(self, opacity: float):
        """Update opacity for inertia visualizations (COM spheres don't support opacity)."""
        self._inertial_opacity = opacity

        # Note: COM spheres (frame origin) don't support opacity, only inertia boxes do

        # Update all existing inertia boxes
        for handle in self._inertia_handles.values():
            if hasattr(handle, "opacity"):
                handle.opacity = opacity

        print(
            f"[INERTIAL CONTROL] Inertial opacity updated to {opacity} (applies to inertia boxes only)"
        )

    def _update_com_visualization(self):
        """Update COM sphere positions based on current robot configuration.
        Assumes forward kinematics has already been updated via viz.display()."""

        # Frame placements need to be updated
        pin.updateFramePlacements(self.robot_model.model, self.viz.data)

        for i, inertia in enumerate(self.robot_model.model.inertias):
            if i == 0:  # Skip universe/world link
                continue

            mass = inertia.mass
            if mass <= 1e-6:  # Skip massless links
                continue

            # Get link name
            link_name = self.robot_model.model.names[i]

            # Compute COM sphere radius
            try:
                radius = com_sphere_radius(mass)
            except ValueError:
                continue  # Skip invalid masses

            # Get link transform
            link_frame_id = self.robot_model.model.getFrameId(link_name)
            T_world_link = self.viz.data.oMf[link_frame_id]

            # COM position in link frame
            com_pos_link = inertia.lever

            # Transform COM to world frame
            com_pos_world = T_world_link.act(com_pos_link)

            # Create or update frame (using only origin sphere, no axes)
            frame_name = f"com/{link_name}"
            if frame_name in self._com_handles:
                # Update existing frame position
                self._com_handles[frame_name].position = tuple(com_pos_world)
                self._com_handles[frame_name].origin_radius = radius
            else:
                # Create new frame with only origin sphere (show_axes must be True to see origin)
                handle = self.server.scene.add_frame(
                    frame_name,
                    wxyz=(1.0, 0.0, 0.0, 0.0),  # Identity quaternion
                    position=tuple(com_pos_world),
                    show_axes=True,  # Must be True to show origin sphere
                    axes_length=0.001,  # Tiny axes (nearly invisible)
                    axes_radius=0.0001,  # Tiny axes
                    origin_radius=radius,
                    origin_color=(255, 0, 0),  # Red as RGB integers
                )
                self._com_handles[frame_name] = handle
                print(
                    f"[COM] {link_name}: mass={mass:.4f}kg, radius={radius:.4f}m, pos={com_pos_world}"
                )

    def _clear_com_visualization(self):
        """Remove all COM spheres."""
        for handle in self._com_handles.values():
            handle.remove()
        self._com_handles.clear()

    def _update_inertia_visualization(self):
        """Update inertia box positions based on current robot configuration.
        Assumes forward kinematics has already been updated via viz.display()."""

        # Frame placements need to be updated
        pin.updateFramePlacements(self.robot_model.model, self.viz.data)

        for i, inertia in enumerate(self.robot_model.model.inertias):
            if i == 0:  # Skip universe/world link
                continue

            mass = inertia.mass
            if mass <= 1e-6:  # Skip massless links
                continue

            # Get link name
            link_name = self.robot_model.model.names[i]

            # Get inertia matrix (3x3)
            # inertia.inertia can be either a numpy array or an Inertia object
            if hasattr(inertia.inertia, "matrix"):
                I_matrix = inertia.inertia.matrix
            elif isinstance(inertia.inertia, np.ndarray):
                I_matrix = inertia.inertia
            else:
                # It's a Symmetric3 object, access the array directly
                I_matrix = np.array(inertia.inertia)

            # Ensure it's the right shape (3x3)
            if I_matrix.shape != (3, 3):
                continue

            # Compute equivalent inertia box
            try:
                box_lengths, R_principal = equivalent_inertia_box(mass, I_matrix)
            except ValueError:
                continue  # Skip invalid inertias

            # Get link transform
            link_frame_id = self.robot_model.model.getFrameId(link_name)
            T_world_link = self.viz.data.oMf[link_frame_id]

            # COM position in link frame
            com_pos_link = inertia.lever

            # Transform COM to world frame
            com_pos_world = T_world_link.act(com_pos_link)

            # Compute box orientation in world frame
            R_world_link = T_world_link.rotation
            R_box_world = R_world_link @ R_principal

            # Fix rotation matrix if it's a reflection (det = -1) instead of rotation (det = +1)
            # This can happen with eigenvectors from np.linalg.eigh
            det = np.linalg.det(R_box_world)
            if det < 0:
                # Flip one axis to convert reflection to rotation
                R_box_world[:, 0] *= -1
                det = np.linalg.det(R_box_world)

            if not np.isclose(det, 1.0, atol=1e-2):
                print(
                    f"[WARNING] Invalid rotation matrix for {link_name}, det={det:.3f}, skipping inertia visualization"
                )
                continue

            # Convert to quaternion using Pinocchio (wxyz format for viser)
            try:
                quat_pinocchio = pin.Quaternion(R_box_world)
                quat_wxyz = quat_pinocchio.coeffs()[
                    [3, 0, 1, 2]
                ]  # Pinocchio uses xyzw, convert to wxyz
            except Exception as e:
                print(
                    f"[WARNING] Could not convert rotation to quaternion for {link_name}: {e}"
                )
                continue

            # Create or update box
            box_name = f"inertia/{link_name}"
            if box_name in self._inertia_handles:
                # Update existing box
                self._inertia_handles[box_name].position = tuple(com_pos_world)
                self._inertia_handles[box_name].wxyz = tuple(quat_wxyz)
            else:
                # Create new box (red to match COM spheres)
                handle = self.server.scene.add_box(
                    box_name,
                    dimensions=tuple(box_lengths),
                    color=(1.0, 0.0, 0.0),  # Red (same as COM)
                    opacity=self._inertial_opacity,
                )
                handle.position = tuple(com_pos_world)
                handle.wxyz = tuple(quat_wxyz)
                self._inertia_handles[box_name] = handle
                print(
                    f"[INERTIA] {link_name}: mass={mass:.4f}kg, dims={box_lengths}, det={det:.3f}"
                )

    def _clear_inertia_visualization(self):
        """Remove all inertia boxes."""
        for handle in self._inertia_handles.values():
            handle.remove()
        self._inertia_handles.clear()

    def add_visual_control(self):
        """Add controls for robot visual appearance (alpha and color)."""

        with self.server.gui.add_folder("Visual Appearance"):
            # Show visual meshes checkbox
            visual_checkbox = self.server.gui.add_checkbox(
                label="Show Visual Meshes",
                initial_value=True,  # Visual meshes shown by default
            )
            visual_checkbox.on_update(
                lambda _: self._toggle_visual(visual_checkbox.value)
            )

            # Alpha slider
            alpha_slider = self.server.gui.add_slider(
                label="Opacity",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=self._visual_alpha,
            )
            alpha_slider.on_update(
                lambda _: self._update_visual_alpha(alpha_slider.value)
            )

            # Color picker (RGB)
            color_picker = self.server.gui.add_rgb(
                label="Color",
                initial_value=(
                    (128, 128, 255)
                    if self._visual_color is None
                    else tuple((np.array(self._visual_color) * 255).astype(int))
                ),
            )
            color_picker.on_update(
                lambda _: self._update_visual_color(
                    np.array(color_picker.value) / 255.0
                )
            )

            # Wireframe checkbox
            wireframe_checkbox = self.server.gui.add_checkbox(
                label="Wireframe Mode",
                initial_value=self._visual_wireframe,
            )
            wireframe_checkbox.on_update(
                lambda _: self._update_visual_wireframe(wireframe_checkbox.value)
            )

        print("[VISUAL CONTROL] Added visual appearance controls")

    def add_collision_control(self):
        """Add controls for collision mesh visualization."""

        with self.server.gui.add_folder("Collision"):
            # Show collision checkbox
            collision_checkbox = self.server.gui.add_checkbox(
                label="Show Collision Meshes",
                initial_value=self._show_collision,
            )
            collision_checkbox.on_update(
                lambda _: self._toggle_collision(collision_checkbox.value)
            )

            # Collision opacity slider
            opacity_slider = self.server.gui.add_slider(
                label="Collision Opacity",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=self._collision_opacity,
            )
            opacity_slider.on_update(
                lambda _: self._update_collision_opacity(opacity_slider.value)
            )

        print("[COLLISION CONTROL] Added collision mesh controls")

    def _toggle_collision(self, show: bool):
        """Toggle collision mesh visibility."""
        self._show_collision = show
        self.viz.displayCollisions(show)

        # Update opacity for all collision meshes
        if show:
            self._update_collision_opacity(self._collision_opacity)

        print(f"[COLLISION CONTROL] Collision meshes {'shown' if show else 'hidden'}")

    def _update_collision_opacity(self, opacity: float):
        """Update collision mesh opacity."""
        self._collision_opacity = opacity

        # Update all collision geometry objects
        if self._show_collision:
            # Use the collisionRootNodeName to get the correct prefix
            collision_prefix = self.viz.collisionRootNodeName
            for geom in self.robot_model.collision_model.geometryObjects:
                collision_name = f"{collision_prefix}/{geom.name}"
                if collision_name in self.viz.frames:
                    handle = self.viz.frames[collision_name]
                    if hasattr(handle, "opacity"):
                        handle.opacity = opacity

        print(f"[COLLISION CONTROL] Collision opacity updated to {opacity}")

    def _toggle_visual(self, show: bool):
        """Toggle visual mesh visibility."""
        self.viz.displayVisuals(show)
        print(f"[VISUAL CONTROL] Visual meshes {'shown' if show else 'hidden'}")

    def _update_visual_alpha(self, alpha: float):
        """Update robot visual opacity."""
        self._visual_alpha = alpha

        # Update all visual model geometry objects
        for geom_name, handle in self.viz.frames.items():
            if hasattr(handle, "opacity"):
                handle.opacity = alpha

    def _update_visual_color(self, color: np.ndarray):
        """Update robot visual color."""
        self._visual_color = color

        # Need to rebuild visual model with new color
        # Convert to list to avoid numpy array truth value ambiguity
        # Format: [R, G, B, Alpha]
        color_rgba = [
            float(color[0]),
            float(color[1]),
            float(color[2]),
            float(self._visual_alpha),
        ]

        # Reload viewer model with new color
        self.viz.loadViewerModel(visual_color=color_rgba)

    def _update_visual_wireframe(self, wireframe: bool):
        """Toggle wireframe mode for all visual meshes."""
        self._visual_wireframe = wireframe

        # Access visual geometry objects and update wireframe property
        prefix = self.viz.viewerRootNodeName + "/visual"

        for geom_id, geometry_object in enumerate(
            self.robot_model.visual_model.geometryObjects
        ):
            frame_name = prefix + "/" + geometry_object.name
            if frame_name in self.viz.frames:
                frame = self.viz.frames[frame_name]
                # Check if it's a mesh handle (has wireframe property)
                if hasattr(frame, "wireframe"):
                    frame.wireframe = wireframe

        print(
            f"[VISUAL CONTROL] Wireframe mode {'enabled' if wireframe else 'disabled'}"
        )

    def add_frame_control(self):
        """Add controls for frame visualization (size, labels, separate for links and joints)."""

        with self.server.gui.add_folder("Frame Control"):
            # Add "Show Display Frames Only" checkbox if display frames are specified
            if self._display_frame_names is not None:
                display_only_checkbox = self.server.gui.add_checkbox(
                    label="Show Display Frames Only",
                    initial_value=self._show_display_frames_only,
                )
                display_only_checkbox.on_update(
                    lambda _: self._toggle_display_frames_only(
                        display_only_checkbox.value
                    )
                )

            # Show link frames checkbox
            link_frames_checkbox = self.server.gui.add_checkbox(
                label="Show Link Frames",
                initial_value=self._show_link_frames,
            )
            link_frames_checkbox.on_update(
                lambda _: self._toggle_link_frames(link_frames_checkbox.value)
            )

            # Show joint frames checkbox
            joint_frames_checkbox = self.server.gui.add_checkbox(
                label="Show Joint Frames",
                initial_value=self._show_joint_frames,
            )
            joint_frames_checkbox.on_update(
                lambda _: self._toggle_joint_frames(joint_frames_checkbox.value)
            )

            # Frame axis length slider
            size_slider = self.server.gui.add_slider(
                label="Frame Axis Length",
                min=0.01,
                max=0.25,
                step=0.01,
                initial_value=self._frame_size,
            )
            size_slider.on_update(lambda _: self._update_frame_size(size_slider.value))

            # Show link frame labels checkbox
            link_labels_checkbox = self.server.gui.add_checkbox(
                label="Show Link Frame Labels",
                initial_value=self._show_link_frame_labels,
            )
            link_labels_checkbox.on_update(
                lambda _: self._toggle_link_frame_labels(link_labels_checkbox.value)
            )

            # Show joint frame labels checkbox
            joint_labels_checkbox = self.server.gui.add_checkbox(
                label="Show Joint Frame Labels",
                initial_value=self._show_joint_frame_labels,
            )
            joint_labels_checkbox.on_update(
                lambda _: self._toggle_joint_frame_labels(joint_labels_checkbox.value)
            )

            # Label font mode (screen vs scene)
            font_mode_dropdown = self.server.gui.add_dropdown(
                label="Label Font Mode",
                options=["screen", "scene"],
                initial_value=self._label_font_mode,
            )
            font_mode_dropdown.on_update(
                lambda _: self._update_label_font_mode(font_mode_dropdown.value)
            )

            # Label size slider (will control screen scale or scene height based on mode)
            label_size_slider = self.server.gui.add_slider(
                label="Label Size",
                min=0.01,
                max=1.0,
                step=0.01,
                initial_value=self._frame_label_size,
            )
            label_size_slider.on_update(
                lambda _: self._update_frame_label_size(label_size_slider.value)
            )

            # Depth test checkbox
            depth_test_checkbox = self.server.gui.add_checkbox(
                label="Label Depth Test",
                initial_value=self._label_depth_test,
            )
            depth_test_checkbox.on_update(
                lambda _: self._update_label_depth_test(depth_test_checkbox.value)
            )

        print("[FRAME CONTROL] Added frame visualization controls")

    def _update_label_font_mode(self, mode: str):
        """Update label font sizing mode (screen vs scene)."""
        self._label_font_mode = mode

        # Update all existing labels
        for handle in self._frame_label_handles.values():
            handle.font_size_mode = mode

            # Apply current size to appropriate property
            if mode == "screen":
                handle.font_screen_scale = self._frame_label_size
            else:  # scene
                handle.font_scene_height = self._frame_label_size

        print(f"[FRAME CONTROL] Label font mode set to '{mode}'")

    def _update_label_depth_test(self, enabled: bool):
        """Update label depth testing."""
        self._label_depth_test = enabled

        # Update all existing labels
        for handle in self._frame_label_handles.values():
            handle.depth_test = enabled

        print(
            f"[FRAME CONTROL] Label depth test {'enabled' if enabled else 'disabled'}"
        )

    def _toggle_display_frames_only(self, enabled: bool):
        """Toggle whether to filter frames by display list."""
        self._show_display_frames_only = enabled

        # Update frame visibility to apply the filter
        if self._show_link_frames or self._show_joint_frames:
            self._update_frame_visibility()

        print(
            f"[FRAME CONTROL] Display frames only filter {'enabled' if enabled else 'disabled'}"
        )

    def _toggle_link_frames(self, show: bool):
        """Toggle link frame visibility."""
        self._show_link_frames = show

        # Ensure frames are displayed if either link or joint frames should be shown
        any_frames_shown = self._show_link_frames or self._show_joint_frames
        self.viz.displayFrames(any_frames_shown)

        # Then update individual frame visibility
        if any_frames_shown:
            self._update_frame_visibility()

    def _toggle_joint_frames(self, show: bool):
        """Toggle joint frame visibility."""
        self._show_joint_frames = show

        # Ensure frames are displayed if either link or joint frames should be shown
        any_frames_shown = self._show_link_frames or self._show_joint_frames
        self.viz.displayFrames(any_frames_shown)

        # Then update individual frame visibility
        if any_frames_shown:
            self._update_frame_visibility()

    def _update_frame_visibility(self):
        """Update visibility of all frames based on link/joint settings and display filter."""
        for frame in self.robot_model.model.frames:
            viser_frame_name = self.viz.framesRootNodeName + "/" + frame.name
            if viser_frame_name in self.viz.frames:
                viser_frame = self.viz.frames[viser_frame_name]

                # Determine if this frame should be visible
                is_link = frame.name in self._link_frame_names
                is_joint = frame.name in self._joint_frame_names

                should_show = (is_link and self._show_link_frames) or (
                    is_joint and self._show_joint_frames
                )

                # Apply display frame filter if enabled
                if (
                    should_show
                    and self._display_frame_names is not None
                    and self._show_display_frames_only
                ):
                    # Only show if frame is in the display list
                    should_show = frame.name in self._display_frame_names

                viser_frame.visible = should_show

    def _update_frame_size(self, size: float):
        """Update frame axis length by accessing internal viser frame handles."""
        self._frame_size = size

        # Access the internal viser frames and update their axes length
        for frame in self.robot_model.model.frames:
            viser_frame_name = self.viz.framesRootNodeName + "/" + frame.name
            if viser_frame_name in self.viz.frames:
                viser_frame = self.viz.frames[viser_frame_name]
                viser_frame.axes_length = size
                viser_frame.axes_radius = size * 0.05  # Proportional radius
                viser_frame.origin_radius = (
                    size * 0.075
                )  # Slightly larger than axes radius

        print(f"[FRAME CONTROL] Frame axis length updated to {size}")

    def _toggle_link_frame_labels(self, show: bool):
        """Toggle link frame label visibility."""
        self._show_link_frame_labels = show

        if show:
            # Add labels to link frames
            self._update_frame_labels()
        else:
            # Remove link frame labels
            self._clear_frame_labels_by_type(is_link=True)

    def _toggle_joint_frame_labels(self, show: bool):
        """Toggle joint frame label visibility."""
        self._show_joint_frame_labels = show

        if show:
            # Add labels to joint frames
            self._update_frame_labels()
        else:
            # Remove joint frame labels
            self._clear_frame_labels_by_type(is_joint=True)

    def _update_frame_labels(self):
        """Add or update labels for visible frames based on link/joint settings.
        Assumes forward kinematics has already been updated via viz.display()."""

        # Frame placements need to be updated
        pin.updateFramePlacements(self.robot_model.model, self.viz.data)

        # Iterate through all frames in the model
        for i, frame in enumerate(self.robot_model.model.frames):
            frame_name = frame.name
            # Label is now a child of the frame: /pinocchio/frames/{frame_name}/label
            viser_frame_name = self.viz.framesRootNodeName + "/" + frame_name
            label_name = viser_frame_name + "/label"

            # Determine if this frame should have a label
            is_link = frame_name in self._link_frame_names
            is_joint = frame_name in self._joint_frame_names

            should_show_label = (is_link and self._show_link_frame_labels) or (
                is_joint and self._show_joint_frame_labels
            )

            # Create, update, or remove label
            try:
                if should_show_label:
                    if label_name not in self._frame_label_handles:
                        handle = self.server.scene.add_label(
                            label_name,
                            text=frame_name,
                        )
                        # Label is a child of the frame, so position is relative to frame origin
                        # Set to (0, 0, 0) to place at the frame's origin
                        handle.position = (0.0, 0.0, 0.0)

                        # Set label properties based on current settings
                        handle.font_size_mode = self._label_font_mode
                        handle.depth_test = self._label_depth_test

                        if self._label_font_mode == "screen":
                            handle.font_screen_scale = self._frame_label_size
                        else:  # scene
                            handle.font_scene_height = self._frame_label_size

                        self._frame_label_handles[label_name] = handle
                    # Note: No need to update position since it's always at frame origin (0,0,0)
                elif label_name in self._frame_label_handles:
                    # Remove label if it shouldn't be shown
                    self._frame_label_handles[label_name].remove()
                    del self._frame_label_handles[label_name]

            except Exception as e:
                print(f"[WARNING] Could not add label for frame {frame_name}: {e}")
                continue

    def _clear_frame_labels_by_type(
        self, is_link: bool = False, is_joint: bool = False
    ):
        """Remove frame labels by type (link or joint)."""
        labels_to_remove = []
        for label_name, handle in self._frame_label_handles.items():
            # Extract frame name from label_name (format: "/pinocchio/frames/{frame_name}/label")
            # Split by "/" and get the second-to-last element (frame name)
            parts = label_name.split("/")
            if len(parts) >= 2:
                frame_name = parts[-2]  # Get frame name before "/label"
            else:
                continue

            # Check if this label should be removed
            should_remove = False
            if is_link and frame_name in self._link_frame_names:
                should_remove = True
            if is_joint and frame_name in self._joint_frame_names:
                should_remove = True

            if should_remove:
                labels_to_remove.append(label_name)

        # Remove the labels
        for label_name in labels_to_remove:
            self._frame_label_handles[label_name].remove()
            del self._frame_label_handles[label_name]

    def _update_frame_label_size(self, size: float):
        """Update frame label size based on current font mode."""
        self._frame_label_size = size

        # Update all existing labels
        for handle in self._frame_label_handles.values():
            if self._label_font_mode == "screen":
                handle.font_screen_scale = size
            else:  # scene
                handle.font_scene_height = size

        print(
            f"[FRAME CONTROL] Label size updated to {size} ({self._label_font_mode} mode)"
        )

    def block_until_input(self):
        input("Press Enter to continue...")
        return self

    def wait_for_load(self):
        time.sleep(0.5)  # wait for robot to load

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

        self.add_robot_control_sliders()
        self._robot_loaded = True
        # print(f"[LOADED] Robot with prefix: {self.robot_prefix}")

    @property
    def port(self) -> str:
        """Return the port of the connected Viser viewer."""
        return str(self.viz.viewer.get_port())

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
                print(
                    f"  Frame {i}: {frame.name}, type: {frame.type}, parent: {frame.parentJoint}"
                )

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
        if self.robot_prefix and hasattr(self.viz.viewer, "__getitem__"):
            self.viz.viewer[self.robot_prefix].delete()
            print(f"[CLEARED] Robot model: {self.robot_prefix}")
            self.robot_prefix = None
        else:
            print("[WARNING] Clearing robot is not supported for this viewer.")
        self._robot_loaded = False

    def get_frame_transform(self, frame_name: str) -> np.ndarray:
        """Get transform of current display"""

        # BUG: This won't for for non joint frames unless you manually update them all!
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
        """Display robot at configuration q and update custom visualizations."""

        if not self._robot_loaded:
            print("[INFO] Robot not loaded — loading now.")
            self.load_robot()

        assert q.shape[0] == self.robot_model.n_dof

        # Display robot using underlying visualizer (this updates forward kinematics and frames)
        self.viz.display(q)

        # Update COM visualization if enabled
        if self._show_com:
            self._update_com_visualization()

        # Update inertia visualization if enabled
        if self._show_inertia:
            self._update_inertia_visualization()

        # Update frame labels if enabled (frames are handled by viz.display)
        if self._show_link_frame_labels or self._show_joint_frame_labels:
            self._update_frame_labels()

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
            time.sleep(sleep_time * (1 / speed))

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

    def set_camera_view(
        self, position: tuple | list | np.ndarray, target: tuple | list | np.ndarray
    ):
        """Set both camera position and target at once."""
        self.set_camera_position(np.array(position))
        self.set_camera_target(np.array(target))
        print(f"[CAMERA] Set view - position: {position}, target: {target}")
