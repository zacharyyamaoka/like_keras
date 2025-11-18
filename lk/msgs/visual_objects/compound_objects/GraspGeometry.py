#!/usr/bin/env python3

"""
    GraspGeometry - Compound visual object for SceneObj geometry visualization.
    
    Displays inner/nominal/outer geometry shells for grasp analysis:
    - Inner (compression): Shows where gripper penetrates surface
    - Nominal: Shows actual object geometry
    - Outer (clearance): Shows required gripper clearance space
    
    Colors can be controlled globally or individually per shell. If individual colors
    are not set, the global_color is used with appropriate alpha values.
    
    Supports both solid and wireframe rendering modes for clearance geometry.
    
    Example (using global_color):
        from bam.msgs.scene.scene_obj import SceneObj, GraspProperties
        from bam.msgs import PoseStamped
        from visual_objects import Material, RGBA
        from bam_artist import Artist
        import bam.msgs.visual_objects as viz
        
        obj = SceneObj(
            name="test_box",
            pose=PoseStamped.from_xyzrpy([0, 0, 0], [0, 0, 0]),
            geometry_shape="box",
            geometry_size=(0.1, 0.05, 0.03),
            visual=Material(color=RGBA.red()),
            grasp=GraspProperties(surface_compression=0.002, surface_clearance=0.03),
            ...
        )
        
        # Uses global_color with auto-alpha (clearance=0.5, nominal=0.8, compression=1.0)
        geom_viz = viz.GraspGeometry.from_scene_obj(obj)
        artist.draw(geom_viz)
        
    Example (custom colors per shell with coordinate frame):
        geom_viz = viz.GraspGeometry(
            name="custom",
            pose=PoseStamped(),
            geometry_shape="box",
            inner_size=(0.046, 0.046, 0.03),
            nominal_size=(0.05, 0.05, 0.03),
            outer_size=(0.11, 0.11, 0.03),
            global_color=RGBA.red(),              # Fallback color
            compression_color=RGBA(1.0, 0.0, 0.0, 1.0),  # Custom compression
            nominal_color=RGBA(0.8, 0.2, 0.0, 0.7),      # Custom nominal
            clearance_color=RGBA(0.5, 0.5, 0.5, 0.3),    # Custom clearance
            frame_scale=0.05                       # Show coordinate frame
        )
        geom_viz.build_model()
        artist.draw(geom_viz)
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from ..markers import Box, Cylinder
from ..Frame import Frame
from ..RGBA import RGBA
from bam.msgs import PoseStamped, PoseType

# PYTHON
from dataclasses import dataclass, field
from typing import Literal, Optional
import copy
import numpy as np

@dataclass
class GraspGeometry(CompoundVisualObject):
    """Compound visual object showing inner/nominal/outer geometry shells.
    
    Visualizes the three geometry states of a SceneObj:
    - Inner geometry: where gripper penetrates (compression)
    - Nominal geometry: actual object size
    - Outer geometry: required clearance space
    
    Colors:
    - Set global_color to apply one base color to all shells (auto-alpha applied)
    - Or set individual colors (nominal_color, compression_color, clearance_color) 
      to customize each shell independently
    """
    
    # Pose and shape data
    pose: PoseType = field(default_factory=PoseStamped)
    geometry_shape: Literal["box", "cylinder", "plate"] = "box"
    
    # Geometry sizes (length, width, height for box; radius, height for cylinder/plate)
    inner_size: tuple[float, float, float] = (0.046, 0.046, 0.03)
    nominal_size: tuple[float, float, float] = (0.05, 0.05, 0.03)
    outer_size: tuple[float, float, float] = (0.11, 0.11, 0.03)
    
    # Visual properties - Colors
    global_color: RGBA = field(default_factory=lambda: RGBA.red())
    inner_color: Optional[RGBA] = None    # If None, uses global_color with alpha
    nominal_color: Optional[RGBA] = None  # If None, uses global_color with alpha
    outer_color: Optional[RGBA] = None    # If None, uses global_color with alpha
    
    # Visibility
    inner_visible: bool = True
    nominal_visible: bool = True
    outer_visible: bool = True
    
    # Rendering mode
    outer_wireframe: bool = True  # If True, clearance is wireframe
    outer_visible: bool = True    # If False, clearance geometry is hidden
    inner_visible: bool = True    # If False, inner geometry is hidden
    frame_scale: float = 0.1          # If > 0, displays coordinate frame at pose

    nominal_scale: float = 0.9999 # slightly smaller for properly visual rendering
    outer_scale: float = 0.9999999 # slightly smaller for properly visual rendering

    inner_name: str = ""
    nominal_name: str = ""
    outer_name: str = ""
    
    def build_model(self):
        """Build the compound visual object with three geometry shells."""
        # Call parent to clear children and add parent frame
        super().build_model()
        
        # Determine actual colors to use (with fallback to global_color + alpha)
        # If specific color is None, copy global_color and set appropriate alpha
        if self.outer_color is not None:
            outer_color = copy.deepcopy(self.outer_color)
        else:
            outer_color = copy.deepcopy(self.global_color)
            # Wireframe outer uses lower opacity for subtle outline
            outer_color.a = 1.0 if self.outer_wireframe else 0.33
        
        if self.nominal_color is not None:
            nominal_color = copy.deepcopy(self.nominal_color)
        else:
            nominal_color = self.global_color.scale(0.5) # make it darker
            nominal_color.a = 1.0
        
        if self.inner_color is not None:
            inner_color = copy.deepcopy(self.inner_color)
        else:
            inner_color = copy.deepcopy(self.global_color)
            inner_color.a = 1.0

        if self.inner_name == "":
            self.inner_name = f"{self.visual_id}/inner"
        if self.nominal_name == "":
            self.nominal_name = f"{self.visual_id}/nominal"
        if self.outer_name == "":
            self.outer_name = f"{self.visual_id}/outer"
        
        # Create visual objects based on geometry shape
        if self.geometry_shape == "box":
            # Outer (clearance) - largest, most transparent
            outer_box = Box(
                name=self.outer_name,
                pose=self.pose,
                scale=(np.array(self.outer_size)*self.outer_scale).tolist(),
                color=outer_color,
                wireframe=self.outer_wireframe,
                visible=self.visible and self.outer_visible,
                cast_shadow=False,
                receive_shadow=False,
            )
            self.children.append(outer_box)
            
            # Nominal - medium transparency
            nominal_box = Box(
                name=self.nominal_name,
                pose=self.pose,
                scale=(np.array(self.nominal_size)*self.nominal_scale).tolist(),
                color=nominal_color,
                wireframe=False,
                visible=self.visible and self.nominal_visible,
                cast_shadow=False,
                receive_shadow=False,
            )
            self.children.append(nominal_box)
            
            # Inner - smallest, most opaque
            inner_box = Box(
                name=self.inner_name,
                pose=self.pose,
                scale=list(self.inner_size),
                color=inner_color,
                wireframe=False,
                visible=self.visible and self.inner_visible,
                cast_shadow=False,
                receive_shadow=False,
            )
            self.children.append(inner_box)
        
        elif self.geometry_shape in ["cylinder", "plate"]:
            # Outer (clearance) - largest, most transparent
            outer_cyl = Cylinder(
                name=self.outer_name,
                pose=self.pose,
                radius=self.outer_size[0]*self.outer_scale,
                height=self.outer_size[1]*self.outer_scale,
                color=outer_color,
                wireframe=self.outer_wireframe,
                visible=self.visible and self.outer_visible,
                cast_shadow=False,
                receive_shadow=False,
            )
            self.children.append(outer_cyl)
            
            # Nominal - medium transparency
            nominal_cyl = Cylinder(
                name=self.nominal_name,
                pose=self.pose,
                radius=self.nominal_size[0]*self.nominal_scale,
                height=self.nominal_size[1]*self.nominal_scale,
                color=nominal_color,
                wireframe=False,
                visible=self.visible and self.nominal_visible,
                cast_shadow=False,
                receive_shadow=False,
            )
            self.children.append(nominal_cyl)
            
            # Inner - smallest, most opaque
            inner_cyl = Cylinder(
                name=self.inner_name,
                pose=self.pose,
                radius=self.inner_size[0],
                height=self.inner_size[1],
                color=inner_color,
                wireframe=False,
                visible=self.visible and self.inner_visible,
                cast_shadow=False,
                receive_shadow=False,
            )
            self.children.append(inner_cyl)
        
        else:
            raise ValueError(f"Unknown geometry shape: {self.geometry_shape}")
        
        # Add coordinate frame if enabled
        if self.frame_scale > 0.0:
            frame = Frame(
                name=f"{self.visual_id}/frame",
                transform=self.pose,
                axis_length=self.frame_scale,
                visible=self.visible
            )
            self.children.append(frame)
    
    @classmethod
    def from_scene_obj(cls, scene_obj, outer_wireframe: bool = False, outer_visible: bool = True) -> 'GraspGeometry':
        """Create GraspGeometry from a SceneObj instance.
        
        Deprecated: Use scene_obj.to_grasp_geometry() instead.
        This method is kept for backwards compatibility.
        
        Args:
            scene_obj: SceneObj instance with geometry and grasp properties
            outer_wireframe: If True, render clearance geometry as wireframe
            outer_visible: If False, hide clearance geometry
            
        Returns:
            GraspGeometry instance ready to be drawn
        """
        return scene_obj.to_grasp_geometry(
            outer_wireframe=outer_wireframe,
            outer_visible=outer_visible
        )


if __name__ == "__main__":
    # BAM
    from bam_artist import Artist
    import bam.msgs.visual_objects as viz
    from bam.msgs import PoseStamped, Vector3
    from visual_objects import Material
    
    artist = Artist()
    artist.attach_viser_viewer()
    artist.draw(viz.Grid())
    
    MM = 1/1000
    
    # Example 1: Box with solid clearance and coordinate frame (using global_color)
    box_geom = GraspGeometry(
        name="box_solid",
        pose=PoseStamped.from_xyzrpy([-0.15, 0, 0], [0, 0, 0]),
        geometry_shape="box",
        inner_size=(0.1, 0.046, 0.03),
        nominal_size=(0.1, 0.05, 0.03),
        outer_size=(0.1, 0.11, 0.03),
        global_color=RGBA.red(),
        frame_scale=0.05,  # Show coordinate frame
    )
    box_geom.build_model()
    artist.draw(box_geom)
    
    # Example 2: Cylinder with wireframe clearance and custom colors
    cyl_geom = GraspGeometry(
        name="cylinder_wireframe",
        pose=PoseStamped.from_xyzrpy([0, 0, 0], [0, 0, 0]),
        geometry_shape="cylinder",
        inner_size=(0.013, 0.08, 0.0),
        nominal_size=(0.015, 0.08, 0.0),
        outer_size=(0.045, 0.08, 0.0),
        global_color=RGBA.green(),
    )
    cyl_geom.build_model()
    artist.draw(cyl_geom)
    
    # Example 3: Plate with solid clearance (using global_color)
    plate_geom = GraspGeometry(
        name="plate_solid",
        pose=PoseStamped.from_xyzrpy([0.15, 0, 0], [0, 0, 0]),
        geometry_shape="plate",
        inner_size=(0.04, 0.016, 0.0),
        nominal_size=(0.04, 0.02, 0.0),
        outer_size=(0.04, 0.08, 0.0),
        global_color=RGBA.blue(),
    )
    plate_geom.build_model()
    artist.draw(plate_geom)
    
    print("✓ GraspGeometry examples displayed!")
    input("Press Enter to exit...")

