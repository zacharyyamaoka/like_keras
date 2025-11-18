"""
    Mesh Visual Marker
    
    Displays a 3D mesh loaded from a file (PLY, OBJ, STL, etc.)
"""

# BAM
from .Marker import Marker
from .. import RGBA

# PYTHON
from dataclasses import dataclass, field
from typing import Literal


@dataclass
class Mesh(Marker):
    """Mesh marker for displaying 3D mesh models."""
    mesh_path: str = ""
    wireframe: bool = False
    material: Literal['standard', 'toon3', 'toon5'] = 'standard'
    flat_shading: bool = True
    side: Literal['front', 'back', 'double'] = 'front'
    cast_shadow: bool = True
    receive_shadow: bool | float = True

