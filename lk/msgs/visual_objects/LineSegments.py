# BAM
from .RGBA import RGBA
from .VisualObject import VisualObject


# PYTHON
from dataclasses import dataclass, field
import copy
import numpy as np

"""
API referneces: 
https://viser.studio/main/examples/scene/lines/
https://wiki.ros.org/rviz/DisplayTypes/Marker#Line_List_.28LINE_LIST.3D5.29


This will draw lines between two points. Color is lerped between the two colors.

Internally we will store as a single flat last, every two points is a line.

Creating it by numpy is nice. but for dataclasses, we prefer to hold in more basic type
"""

@dataclass
class LineSegments(VisualObject):
    points: list[list[float, float, float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0]])
    colors: list[RGBA] = field(default_factory=lambda: [RGBA.grey()])
    line_width: float = 10.0

    def __post_init__(self):
        super().__post_init__() 

    @classmethod
    def from_points(cls, points: list[list[float, float, float]], colors: list[RGBA] = [RGBA.grey()], line_width: float = 1.0, name: str = "") -> 'LineSegments':
        return cls(points=points, colors=colors, line_width=line_width, name=name)

    def to_numpy(self, alpha: bool = True, base8: bool = False, bgr: bool = False) -> tuple[np.ndarray, np.ndarray]:
        points_array = np.array(self.points).reshape(-1, 2, 3)
        
        colors_list = [color.to_numpy(alpha=alpha, base8=base8, bgr=bgr) for color in self.colors]
        colors_array = np.array(colors_list).reshape(-1, 2, 3 if not alpha else 4)
        
        return points_array, colors_array


    @classmethod
    def from_numpy(cls, points: np.ndarray, colors: np.ndarray, line_width: float = 1.0, base8: bool = False, bgr: bool = False, name: str = "") -> 'LineSegments':
        points = copy.deepcopy(points)
        colors = copy.deepcopy(colors)
        # this supports colors with and without alpha!
        assert points.ndim == 3 and points.shape[1] == 2 and points.shape[2] == 3, "points must be a 3D array with shape (N, 2, 3)"
        assert colors.ndim == 3 and colors.shape[0] == points.shape[0] and colors.shape[1] == 2, "colors must be a 3D array with shape (N, 2, 3) or (N, 2, 4)"
        assert colors.shape[2] in (3, 4), "colors must have 3 (RGB) or 4 (RGBA) channels"

        # Flatten points from (N, 2, 3) to list of lists
        points_flat = points.reshape(-1, 3).tolist()

        # Flatten colors from (N, 2, 3/4) to list of RGBA objects
        colors_flat = []
        colors_reshaped = colors.reshape(-1, colors.shape[-1])  # (N*2, 3) or (N*2, 4)
        for color in colors_reshaped:
            colors_flat.append(RGBA.from_numpy(color, base8, bgr))

        return cls(points=points_flat, colors=colors_flat, line_width=line_width, name=name)

if __name__ == "__main__":
    line_segments = LineSegments()
    print(line_segments)