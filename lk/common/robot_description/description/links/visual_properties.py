"""
    Visual description for link geometry.
"""

# BAM
from bam.msgs.ros_msgs import Pose
from .geometry import Geometry
from .material import Material

# PYTHON
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class VisualProperties:
    """Visual geometry description for a link."""
    name: Optional[str] = None
    origin: Pose = field(default_factory=Pose)
    geometry: Geometry = field(default_factory=Geometry)
    material: Material = field(default_factory=Material)
    
    def to_xml(self) -> str:
        """Generate URDF XML representation of the visual.
        
        Returns:
            str: URDF XML string for the visual (assumes 4-space indentation)
        """
        if not self.geometry.filename:
            return ""
        
        xyz = ' '.join([f'{v:.6f}' for v in self.origin.xyz])
        rpy = ' '.join([f'{v:.6f}' for v in self.origin.rpy])
        
        xml = f'''    <visual>
      <origin xyz="{xyz}" rpy="{rpy}"/>
      <geometry>
        <mesh filename="{self.mesh_path}"/>
      </geometry>
    </visual>'''
        
        return xml


