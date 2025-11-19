#!/usr/bin/env python3

"""
    URDF visual object implementation.

    Provides helpers for converting BAM robot descriptions into visual objects
    with configurable mesh loading and color overrides.
"""

# BAM
from .VisualObject import VisualObject
from .RGBA import RGBA

# PYTHON
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription

@dataclass
class Urdf(VisualObject):
    xml: str = "" # For now lets just use the urdf_xml directly...
    file_path: str = ""
    mesh_package_name: str = ""

    # Color overrides: can be a single RGBA for all links, or dict mapping link_name -> RGBA
    visual_mesh_color_override: Optional[RGBA | dict[str, RGBA]] = None
    collision_mesh_color_override: Optional[RGBA | dict[str, RGBA]] = None
    
    # Visibility: can be a single bool for all links, or dict mapping link_name -> bool
    load_meshes: bool = True
    load_collision_meshes: bool = True
    visual_mesh_visible: bool | dict[str, bool] = True
    collision_mesh_visible: bool | dict[str, bool] = False
    
    # Mutability: force creation of meshes with mutable colors/visibility (loses textures)
    # Set to True if you need to change per-link colors or visibility after creation
    visual_mesh_mutable: bool = False
    collision_mesh_mutable: bool = False

    visible: bool = True

    q: list[float] = field(default_factory=lambda: [0.]*6)
    
    # Joint information for handling mimic joints
    joint_names: list[str] = field(default_factory=list)  # All non-fixed joint names
    mimic_joint_names: list[str] = field(default_factory=list)  # Mimic/virtual joint names
  
    def __post_init__(self):
        super().__post_init__()

    @classmethod
    def from_robot_description(
        cls,
        description: 'RobotDescription',
        q: Optional[list[float]] = None,
        name: str = "",
        load_meshes: bool = True,
        load_collision_meshes: bool = True,
        visual_mesh_visible: bool | dict[str, bool] = True,
        collision_mesh_visible: bool | dict[str, bool] = False,
        visual_mesh_mutable: bool = False,
        collision_mesh_mutable: bool = False,
        visual_mesh_color_override: Optional[RGBA | dict[str, RGBA]] = None,
        collision_mesh_color_override: Optional[RGBA | dict[str, RGBA]] = None,
        visible: bool = True,
    ) -> 'Urdf':
        if q is None:
            joint_positions = getattr(description, "joint_positions", None)
            if joint_positions is not None:
                try:
                    q = joint_positions.get("initial")
                except Exception:
                    q = None
            if not q:
                joint_names_for_default = description.joints.get_actuated_joint_names(prefix=True)
                q = [0.0] * len(joint_names_for_default)

        urdf_xml = description.get_urdf_xml()
        
        # Get joint information for handling mimic joints
        joint_names = description.joints.get_actuated_joint_names(prefix=False)
        mimic_joint_names = []
        
        return cls(
            xml=urdf_xml,
            q=q,
            mesh_package_name=getattr(description.urdf, 'mesh_package_name', ''),
            joint_names=joint_names,
            mimic_joint_names=mimic_joint_names,
            name=name,
            load_meshes=load_meshes,
            load_collision_meshes=load_collision_meshes,
            visual_mesh_visible=visual_mesh_visible,
            collision_mesh_visible=collision_mesh_visible,
            visual_mesh_mutable=visual_mesh_mutable,
            collision_mesh_mutable=collision_mesh_mutable,
            visual_mesh_color_override=visual_mesh_color_override,
            collision_mesh_color_override=collision_mesh_color_override,
            visible=visible,
        )
    
    @property
    def q_active(self) -> list[float]:
        """Get configuration for only active (non-mimic) joints.
        
        This is useful for libraries like yourdfpy that understand mimic joints
        and expect only actuated joint values.
        """
        if not self.mimic_joint_names:
            return self.q
        
        # Filter out mimic joints
        active_indices = [i for i, name in enumerate(self.joint_names) 
                         if name not in self.mimic_joint_names]
        return [self.q[i] for i in active_indices]
    
    @property 
    def num_active_joints(self) -> int:
        """Number of active (non-mimic) joints."""
        return len(self.joint_names) - len(self.mimic_joint_names)
    
    @property
    def num_mimic_joints(self) -> int:
        """Number of mimic joints."""
        return len(self.mimic_joint_names)
    
    @property
    def num_joints(self) -> int:
        """Total number of non-fixed joints."""
        return len(self.joint_names)


if __name__ == "__main__":
    urdf = Urdf()
    print(urdf)
    print(urdf.visual_id)