# BAM
from bam.msgs import PoseStamped
from .Marker import Marker
from ..RGBA import RGBA

# PYTHON
from dataclasses import dataclass
import numpy as np


@dataclass
class Arrow(Marker):
    length: float = 0.2
    thickness_scale: float = 1.0
    uniform_thickness: bool = True
    cone_length_frac: float = 0.15
    cone_angle: float = 60.0  # Internal angle of cone in degrees

    # Calculated dimensions (set by build_model)
    cylinder_radius: float = None
    cylinder_height: float = None
    cone_radius: float = None
    cone_height: float = None

    def __post_init__(self):
        super().__post_init__()
        self.build_model()

    def build_model(self):
        """Build the arrow model geometry based on current parameters.

        Convention: pose is at origin, arrow extends along +Z axis
        - cone_length = cone_length_frac * length
        - cylinder_length = (1 - cone_length_frac) * length

        Thickness modes:
        - uniform_thickness=True: cylinder_radius is constant, cone_radius = 2x
        - uniform_thickness=False: cone_radius calculated from cone_angle, cylinder_radius = cone_radius/2
        """
        # Calculate cone and cylinder heights
        self.cone_height = self.cone_length_frac * self.length
        self.cylinder_height = (1.0 - self.cone_length_frac) * self.length

        # Calculate radii based on thickness mode
        if self.uniform_thickness:
            # Uniform: radius is independent of length
            self.cylinder_radius = 0.005 * self.thickness_scale
            # Cone radius is 2x cylinder radius for a nice pointed shape
            self.cone_radius = 2.0 * self.cylinder_radius
        else:
            # Scaled: cone_radius calculated from cone_angle to maintain geometric proportions
            # Internal cone angle defines the angle at the tip between the two sides
            # If internal angle is θ, then half-angle is θ/2
            # tan(θ/2) = cone_radius / cone_height
            half_angle_rad = np.radians(self.cone_angle / 2.0)
            self.cone_radius = (
                self.cone_height * np.tan(half_angle_rad) * self.thickness_scale
            )
            # Cylinder radius is half of cone radius for smooth transition
            self.cylinder_radius = self.cone_radius / 2.0

    @classmethod
    def from_origin_to_end(
        cls,
        origin: list[float, float, float],
        end: list[float, float, float],
        color: RGBA = RGBA.grey(),
        thickness_scale: float = 1.0,
        uniform_thickness: bool = True,
        cone_angle: float = 60.0,
        name: str = "",
    ) -> "Arrow":
        """
        Create arrow from origin to end point.

        Creates a 4x4 transform matrix that:
        - Places origin at the specified origin point
        - Aligns +Z axis with the vector from origin to end
        - Sets length to the distance between origin and end
        """
        vec = np.array(end) - np.array(origin)
        length = np.linalg.norm(vec)
        vec = vec / length  # normalize

        # Define new basis: z' = vec
        z_axis = vec

        # Pick a reference up vector (avoid parallel)
        up = np.array([0, 1, 0]) if abs(z_axis[1]) < 0.9 else np.array([1, 0, 0])

        # x' = up × z'
        x_axis = np.cross(up, z_axis)
        x_axis /= np.linalg.norm(x_axis)

        # y' = z' × x'
        y_axis = np.cross(z_axis, x_axis)

        R = np.column_stack([x_axis, y_axis, z_axis])  # rotation matrix

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = origin

        return cls(
            pose=PoseStamped.from_matrix(T),
            length=length,
            color=color,
            thickness_scale=thickness_scale,
            uniform_thickness=uniform_thickness,
            cone_angle=cone_angle,
            name=name,
        )


if __name__ == "__main__":

    arrow = Arrow()
    print(arrow)

    arrow = Arrow.from_origin_to_end(origin=[0, 0, 0], end=[1, 0, 0])
    print(arrow)
