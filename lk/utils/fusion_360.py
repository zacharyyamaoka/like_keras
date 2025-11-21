"""
Converts Fusion 360 Properties to XML URDF format and units


Step 1. Create a collision component (call something like Shoulder Collision). This is just a simple box that can be used for fast collision checking.
        Just do the best you can doesn't need to be exact. For more complex geometry ie. elbow link, you can always just read the data and then manually add it, etc.
        You can also just select an approiate geometry. The only benefit of making a seperate component is that the center of mass will be perfectly centered so the bbox will be perfectly centered.
Step 2. In Standalone file, import in all components, and add fixed joint for link at origin, +Z upwards, +Y Forwards, +X Right
Step 3. Click the actual component and copy/paste the properties. Hidden components will still reflect in the mass calculation, If you need them there set it to a virtual density of 0
Step 4. CLick the collision component and copy/paste the properties


We could of course automate this more. https://github.com/andreasBihlmaier/FusionSDF

But I found that a bit restricting on the CAD design. Regardless this is something you kinda want to do once, do well and then never look at agian!

When exporting STLs you should hide the PCB boards as adds unnesary triangles

Good idea to have a very clear seperation between exporting and CAD. Exporting has very different requirements than CAD. I don't want to populate the CAD files.

"""

from dataclasses import dataclass, field
import re
import os
import glob
import csv
from typing import Dict, Tuple
import numpy as np


@dataclass
class Properties:
    name: str = None
    mass_g: float = None
    com_mm: list[float] = None
    inertia_gmm2: dict[str, float] = field(default_factory=dict)
    bbox_mm: list[float] = None

    def __post_init__(self):
        self.mass_kg = self.mass_g / 1000.0
        self.com_m = [x / 1000.0 for x in self.com_mm]
        self.inertia_kgm2 = {
            k: v * 1e-9 for k, v in self.inertia_gmm2.items()
        }  # g·mm² to kg·m²
        self.bbox_m = [d / 1000.0 for d in self.bbox_mm]

    def get_inertia_matrix(self) -> np.ndarray:
        """
        Build 3x3 inertia matrix from inertia properties.

        Returns:
            np.ndarray: 3x3 symmetric inertia tensor in kg·m²
        """
        I = self.inertia_kgm2
        return np.array(
            [
                [I["ixx"], I["ixy"], I["ixz"]],
                [I["ixy"], I["iyy"], I["iyz"]],
                [I["ixz"], I["iyz"], I["izz"]],
            ]
        )

    def get_inertia_matrix_string(self) -> str:
        """
        Get formatted string representation of inertia matrix.

        Returns:
            str: Multi-line string showing the 3x3 inertia matrix
        """
        I = self.inertia_kgm2
        return (
            f"  [{I['ixx']:.6e}, {I['ixy']:.6e}, {I['ixz']:.6e}]\n"
            f"  [{I['ixy']:.6e}, {I['iyy']:.6e}, {I['iyz']:.6e}]\n"
            f"  [{I['ixz']:.6e}, {I['iyz']:.6e}, {I['izz']:.6e}]"
        )


def parse_properties_file(filename="properties.txt", verbose=False) -> Properties:
    name = None
    mass_g = None
    com_mm = None
    inertia_gmm2 = dict()
    bbox_mm = None

    inertia_section = False

    def print_v(msg):
        if verbose:
            print(msg)

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            line_split = line.split()
            first_word = line_split[0]

            if "Part Name" in line:
                print_v(line)
                # Extract name from "Part Name\tComponent1" or "Part Name Component1" format
                # Try tab first, then fall back to splitting by whitespace
                if "\t" in line:
                    parts = line.split("\t")
                else:
                    parts = line.split(None, 1)  # Split on whitespace, max 1 split
                if len(parts) >= 2:
                    name = parts[1].strip()
                    print_v(f"Part Name: {name}")

            elif first_word == "Mass":
                print_v(line)
                mass_g = float(line_split[1])
                print_v(f"Mass (g) : {mass_g}")

            elif first_word == "Center":
                print_v(line)
                # Improved regex to capture floats, including scientific notation
                values = re.findall(
                    r"[-+]?\d*\.\d+(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?", line
                )
                com_mm = [float(v) for v in values]
                print_v(f"COM (mm) : {com_mm}")

            elif "Bounding Box" in line:
                print_v(line)
                bbox_mm = []
                for _ in range(3):
                    subline = next(f).strip()
                    print_v(subline)
                    val = float(subline.split()[-2])  # e.g., "Length 104.00 mm"
                    bbox_mm.append(val)
                print_v(f"Bounding Box (mm): {bbox_mm}")

            elif "Moment of Inertia at Center of Mass" in line:
                print_v(line)
                inertia_section = True

            elif inertia_section:
                print_v(line)
                inertia_gmm2[first_word.lower()] = float(line_split[1])
                if len(inertia_gmm2.keys()) >= 9:
                    inertia_section = False
                    print_v(inertia_gmm2)

    return Properties(name, mass_g, com_mm, inertia_gmm2, bbox_mm)


def generate_interial_xml(p: Properties):
    I = p.inertia_kgm2

    urdf = f"""<inertial>
    <origin xyz="{p.com_m[0]:.6f} {p.com_m[1]:.6f} {p.com_m[2]:.6f}"/>
    <mass value="{p.mass_kg:.6f}"/>
    <inertia 
        ixx="{I['ixx']:.6e}" ixy="{I['ixy']:.6e}" ixz="{I['ixz']:.6e}"
        iyy="{I['iyy']:.6e}" iyz="{I['iyz']:.6e}"
        izz="{I['izz']:.6e}"/>
</inertial>"""
    return urdf


def generate_collision_xml(p: Properties):

    return f"""<collision>
    <origin xyz="{p.com_m[0]:.6f} {p.com_m[1]:.6f} {p.com_m[2]:.6f}" rpy="0 0 0"/>
    <geometry>
        <box size="{p.bbox_m[0]:.6f} {p.bbox_m[1]:.6f} {p.bbox_m[2]:.6f}"/>
    </geometry>
</collision>"""


def generate_link_py(name, mesh_path, interial: Properties, collision: Properties):

    I = interial.inertia_kgm2

    return f"""
        {name} = self.links.{name}
        {name}.visual_mesh_path = '{mesh_path}'
        {name}.collision_geometry_type = "box"
        {name}.collision_geometry_size = ({collision.bbox_m[0]:.6f}, {collision.bbox_m[1]:.6f}, {collision.bbox_m[2]:.6f})
        {name}.collision_origin = Pose.from_xyzrpy([{collision.com_m[0]:.6f}, {collision.com_m[1]:.6f}, {collision.com_m[2]:.6f}], [0, 0, 0])
        {name}.inertial.mass = {interial.mass_kg:.6f}
        {name}.inertial.origin = Point(x={interial.com_m[0]:.6f}, y={interial.com_m[1]:.6f}, z={interial.com_m[2]:.6f})
        {name}.inertial.inertia = Inertia(ixx={I['ixx']:.6e}, ixy={I['ixy']:.6e}, ixz={I['ixz']:.6e}, iyy={I['iyy']:.6e}, iyz={I['iyz']:.6e}, izz={I['izz']:.6e})
    """
    # {name}.visual_origin = Pose.from_xyzrpy([0, 0, 0], [0, 0, 0])


# def generate_link_py(name, mesh_path, bbox_mm, com_mm):


def parse_vertices_and_compute_obb(
    vertices_file: str,
) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """
    Parse vertices CSV file and compute oriented bounding boxes for each component.

    Args:
        vertices_file: Path to CSV file with columns: component_name,body_name,vertex_index,x,y,z
                      Assumes vertex coordinates are in centimeters (Fusion 360 default)

    Returns:
        Dictionary mapping component name to tuple of (extents, transform_matrix)
        - extents: numpy array of shape (3,) containing the box dimensions in meters
        - transform_matrix: numpy array of shape (4,4) representing the transform from
          box center frame to world frame (positions in meters)
    """
    from bam.utils.bbox import oriented_bbox_from_points

    # Parse CSV file and group vertices by component
    component_vertices = {}

    with open(vertices_file, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            component_name = row["component_name"]
            # Fusion 360 exports in cm, convert to meters
            x = float(row["x"]) / 100.0
            y = float(row["y"]) / 100.0
            z = float(row["z"]) / 100.0

            if component_name not in component_vertices:
                component_vertices[component_name] = []
            component_vertices[component_name].append([x, y, z])

    # Compute OBB for each component
    result = {}

    for component_name, vertices_list in component_vertices.items():
        # Convert to numpy array (already in meters)
        vertices = np.array(vertices_list, dtype=np.float64)

        # Compute oriented bounding box from points
        extents, transform = oriented_bbox_from_points(vertices)

        result[component_name] = (extents, transform)

    return result


if __name__ == "__main__":

    from bam_descriptions import BamFb, LinkDescription

    arm = BamFb()

    link: LinkDescription
    for link in arm.links:
        link_name = link.name
        if link.is_frame:
            continue

        # link_name = "arm_base_link"
        dir_path = "/home/bam/bam_ws/src/bam_common/bam_descriptions/bam_descriptions/arms/bam_fb/bam_fb_dev"

        # Double bam_descriptions as is inside the python package
        mesh_dir = (
            "package://bam_descriptions/bam_descriptions/arms/bam_fb/bam_fb_dev/mesh/"
        )
        mesh_path = os.path.join(mesh_dir, f"{link_name}.stl")

        interial_file = os.path.join(dir_path, "interial", f"{link_name}.txt")

        interial_props = parse_properties_file(interial_file)
        urdf_inertial = generate_interial_xml(interial_props)

        collision_file = os.path.join(dir_path, "collision", f"{link_name}.txt")
        collision_props = parse_properties_file(collision_file)
        urdf_collision = generate_collision_xml(collision_props)

        # print("\n--- URDF ---")
        # print(urdf_collision)
        # print(urdf_inertial)

        link_py = generate_link_py(
            link_name, mesh_path, interial_props, collision_props
        )

        # print("\n--- PY ---")
        print(link_py)

    # print("\nLoading collision properties from:")
    # # This supports compound collision shapes
    # # Ex: L1.txt, L1_1.txt, L1_2.txt
    # urdf_collision_blocks = []
    # for path in collision_files:
    #     print(f" - {path}")
    #     _, com_mm, _, bbox_mm = parse_properties_file(path)
    #     urdf_collision_blocks.append(generate_collision_xml(bbox_mm, com_mm))
    #     urdf_collision_blocks.append("") # for spacing between

    # urdf_collision = "\n".join(urdf_collision_blocks)

    # print("\n--- URDF ---")
    # print(urdf_collision)
    # print(urdf_inertial)
