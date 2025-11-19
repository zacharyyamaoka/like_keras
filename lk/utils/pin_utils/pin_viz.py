#!/usr/bin/env python3

"""
    Thin wrapper for Pinocchio visualizers
    
    Provides a unified interface to switch between visualization backends
    (PinViser or PinMeshcat) and easily disable visualization via environment
    variable.
    
    Purpose:
    1. Easy disabling of visualization calls via DISABLE_PIN_VIZ env var
    2. Easy switching between visualization backends
    
    Note: Keep this lightweight. Use BAM Artist for complex visualizations.
    This is just for bringing up the robot model, seeing collisions/visuals,
    controlling joints (with Viser), and displaying frames.
"""

# BAM
from .pin_viser import PinViser
from .pin_meshcat import PinMeshcat
from .pin_robot_model import PinRobotModel

# PYTHON
import os
import numpy as np
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription


class PinViz:
    """
        Thin wrapper around Pinocchio visualizers.
        
        Supports PinViser (default) or PinMeshcat backends.
        Respects DISABLE_PIN_VIZ environment variable to no-op all calls.
    """
    
    @classmethod
    def from_robot_description(cls, rd: 'RobotDescription', backend: Literal["viser", "meshcat"] = "viser", **kwargs):
        robot_model = PinRobotModel.from_robot_description(rd)
        return cls(robot_model, backend=backend, **kwargs)
    
    @classmethod
    def from_urdf_xml(cls, urdf_xml: str, backend: Literal["viser", "meshcat"] = "viser", **kwargs):
        robot_model = PinRobotModel.from_urdf_xml(urdf_xml)
        return cls(robot_model, backend=backend, **kwargs)
    
    @classmethod
    def from_xacro(cls, xacro_path: str, xacro_args: dict, mesh_package_dirs: str | list[str], 
                   backend: Literal["viser", "meshcat"] = "viser", **kwargs):
        robot_model = PinRobotModel.from_xacro(xacro_path, xacro_args, mesh_package_dirs)
        return cls(robot_model, backend=backend, **kwargs)
    
    @classmethod
    def from_urdf(cls, urdf_path: str, mesh_package_dirs: str | list[str], 
                  backend: Literal["viser", "meshcat"] = "viser", **kwargs):
        robot_model = PinRobotModel.from_urdf(urdf_path, mesh_package_dirs)
        return cls(robot_model, backend=backend, **kwargs)
    
    def __init__(self, robot_model: PinRobotModel, backend: Literal["viser", "meshcat"] = "viser", **kwargs):
        """
            Args:
                robot_model: PinRobotModel with model, collision_model, and visual_model
                backend: "viser" (default) or "meshcat"
                **kwargs: Additional arguments passed to backend (port/zmq_url, color, etc.)
        """
        self.robot_model = robot_model
        self.backend_name = backend
        self.disabled = os.environ.get("DISABLE_PIN_VIZ", "").lower() in ("1", "true", "yes")
        
        if self.disabled:
            print("[PIN_VIZ] DISABLED via DISABLE_PIN_VIZ environment variable")
            self.backend = None
            self._sliders_added = False
            return
        
        # Initialize the backend
        if backend == "viser":
            self.backend = PinViser(robot_model, **kwargs)
            self._sliders_added = True  # PinViser adds sliders in load_robot
        elif backend == "meshcat":
            self.backend = PinMeshcat(robot_model, **kwargs)
            self._sliders_added = False  # Meshcat doesn't have sliders
        else:
            raise ValueError(f"Unknown backend: {backend}. Choose 'viser' or 'meshcat'")
        
        print(f"[PIN_VIZ] Initialized with backend: {backend}")
    
    def display(self, q: np.ndarray):
        """
            Display the robot at configuration q.
            
            For Viser backend: automatically adds robot control sliders on first display.
        """
        if self.disabled:
            return
        
        self.backend.display(q)
    
    def displayCollisions(self, visibility: bool):
        """Set whether to display collision objects or not."""
        if self.disabled:
            return
        
        self.backend.viz.displayCollisions(visibility)
    
    def displayVisuals(self, visibility: bool):
        """Set whether to display visual objects or not."""
        if self.disabled:
            return
        
        self.backend.viz.displayVisuals(visibility)
    
    def displayFrames(self, visibility: bool, frame_ids=None, **kwargs):
        """
            Set whether to display frames or not.
            
            Args:
                visibility: True to show frames, False to hide
                frame_ids: Optional list of frame IDs to display. If None, displays all.
                **kwargs: Additional arguments (axis_length, axis_width, etc.)
        """
        if self.disabled:
            return
        
        # ViserVisualizer and MeshcatVisualizer both have displayFrames
        if frame_ids is None:
            self.backend.viz.displayFrames(visibility, **kwargs)
        else:
            self.backend.viz.displayFrames(visibility, frame_ids, **kwargs)
    
    def play(self, q_trajectory, dt=None, callback=None, capture=False, **kwargs):
        """
            Play a trajectory with given time step.
            
            Args:
                q_trajectory: List/array of joint configurations
                dt: Time step between configurations (seconds). If None, plays as fast as possible.
                callback: Optional callback function(i, **kwargs) called at each step
                capture: If True, capture RGB images (implementation depends on backend)
                **kwargs: Additional arguments passed to callback
        """
        if self.disabled:
            return
        
        return self.backend.viz.play(q_trajectory, dt=dt, callback=callback, capture=capture, **kwargs)
    
    @property
    def model(self):
        """Access the Pinocchio model."""
        return self.robot_model.model
    
    @property
    def viz(self):
        """Direct access to the underlying visualizer (ViserVisualizer or MeshcatVisualizer)."""
        if self.disabled:
            return None
        return self.backend.viz
    
    @property
    def is_viser(self) -> bool:
        """Check if using Viser backend."""
        return self.backend_name == "viser" and not self.disabled
    
    @property
    def is_meshcat(self) -> bool:
        """Check if using Meshcat backend."""
        return self.backend_name == "meshcat" and not self.disabled

    def block_until_input(self):
        
        self.backend.block_until_input()


if __name__ == "__main__":
    """
        Simple example demonstrating PinViz usage with both backends.
    """
    # Example: Create a simple robot and visualize it
    print("\n=== PinViz Example ===\n")
    
    # This would work with an actual URDF file
    # viz = PinViz.from_urdf("path/to/robot.urdf", mesh_package_dirs=["path/to/meshes"], backend="viser")
    
    # For demonstration purposes
    import pinocchio as pin
    
    # Create a simple robot model
    model = pin.Model()
    geom_model = pin.GeometryModel()
    
    robot_model = PinRobotModel(model, geom_model, geom_model)
    
    # Test with Viser backend (default)
    print("Testing Viser backend:")
    try:
        viz_viser = PinViz(robot_model, backend="viser")
        q = pin.neutral(model)
        viz_viser.display(q)
        print("✓ Viser backend works")
    except Exception as e:
        print(f"✗ Viser backend error: {e}")
    
    # Test with disabled visualization
    print("\nTesting disabled mode:")
    os.environ["DISABLE_PIN_VIZ"] = "1"
    viz_disabled = PinViz(robot_model, backend="viser")
    viz_disabled.display(q)  # Should be no-op
    viz_disabled.displayCollisions(True)  # Should be no-op
    print("✓ Disabled mode works (no-ops all calls)")
    
    print("\n=== Example complete ===\n")

