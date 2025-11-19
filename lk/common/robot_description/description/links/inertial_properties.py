"""
    Inertial properties combining mass, origin, and inertia tensor.
"""

# BAM
from bam.msgs.ros_msgs import Point

# PYTHON
from dataclasses import dataclass, field

from .inertia import Inertia


@dataclass
class InertialProperties:
    mass: float = 0.0
    origin: Point = field(default_factory=Point)
    inertia: Inertia = field(default_factory=Inertia)
    
    def to_xml(self) -> str:
        """Generate URDF XML representation of the inertial.
        
        Returns:
            str: URDF XML string for the inertial (assumes 4-space indentation)
        """
        if self.mass <= 0:
            return ""
        
        xyz = ' '.join([f'{v:.6f}' for v in self.origin.xyz])
        ixx = self.inertia.ixx
        ixy = self.inertia.ixy
        ixz = self.inertia.ixz
        iyy = self.inertia.iyy
        iyz = self.inertia.iyz
        izz = self.inertia.izz
        
        xml = f'''    <inertial>
      <origin xyz="{xyz}" rpy="0 0 0"/>
      <mass value="{self.mass:.6f}"/>
      <inertia ixx="{ixx:.6f}" ixy="{ixy:.6f}" ixz="{ixz:.6f}" iyy="{iyy:.6f}" iyz="{iyz:.6f}" izz="{izz:.6f}"/>
    </inertial>'''
        
        return xml

