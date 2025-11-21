#!/usr/bin/env python3

"""
URDF Converter

Converts between RobotDescription and URDF formats using yourdfpy.
Provides functionality to:
- Parse URDF files/strings into RobotDescription objects
- Export RobotDescription objects to URDF format
- Support round-trip conversion
"""

# BAM
from .description_types import (
    JointDescription,
    LinkDescription,
    PerJointLimits,
    PerJointPhysics,
    InertialProperties,
    Inertia,
    VisualProperties,
    CollisionProperties,
    Geometry,
    Box,
    Cylinder,
    Sphere,
    Mesh,
    Material,
    RGBA,
    RobotInfo,
    UrdfInfo,
)
from bam.msgs.ros_msgs import TransformStamped, Vector3, Quaternion, Pose

# PYTHON
import numpy as np
from typing import Optional
from pathlib import Path
import xml.etree.ElementTree as ET

try:
    import yourdfpy
except ImportError:
    raise ImportError(
        "yourdfpy is required for URDF conversion. Install with: pip install yourdfpy"
    )


def parse_origin(origin_element) -> tuple[list[float], list[float]]:
    """Parse URDF origin element into xyz and rpy lists.

    Args:
        origin_element: yourdfpy origin object or None

    Returns:
        Tuple of (xyz, rpy) as lists of floats
    """
    if origin_element is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    xyz = (
        list(origin_element.xyz) if hasattr(origin_element, "xyz") else [0.0, 0.0, 0.0]
    )
    rpy = (
        list(origin_element.rpy) if hasattr(origin_element, "rpy") else [0.0, 0.0, 0.0]
    )

    return xyz, rpy


def yourdfpy_to_transform(
    parent_link: str, child_link: str, origin
) -> TransformStamped:
    """Convert yourdfpy origin to TransformStamped.

    Args:
        parent_link: Name of parent link
        child_link: Name of child link
        origin: yourdfpy origin object

    Returns:
        TransformStamped with position and orientation
    """
    xyz, rpy = parse_origin(origin)

    # Create transform
    transform = TransformStamped()
    transform.header.frame_id = parent_link
    transform.child_frame_id = child_link
    transform.xyz = xyz
    transform.rpy = rpy

    return transform


def parse_geometry_from_yourdfpy(geom) -> Geometry:
    """Parse yourdfpy geometry into Geometry object.

    Args:
        geom: yourdfpy geometry object

    Returns:
        Geometry subclass instance (Box, Cylinder, Sphere, Mesh, or Geometry)
    """
    if geom is None:
        return Geometry()

    geom_type = geom.__class__.__name__.lower()

    if geom_type == "box":
        return Box(size=list(geom.size) if hasattr(geom, "size") else [1.0, 1.0, 1.0])
    elif geom_type == "cylinder":
        return Cylinder(
            radius=float(geom.radius) if hasattr(geom, "radius") else 1.0,
            length=float(geom.length) if hasattr(geom, "length") else 1.0,
        )
    elif geom_type == "sphere":
        return Sphere(radius=float(geom.radius) if hasattr(geom, "radius") else 1.0)
    elif geom_type == "mesh":
        filename = geom.filename if hasattr(geom, "filename") else ""
        scale = list(geom.scale) if hasattr(geom, "scale") else [1.0, 1.0, 1.0]
        return Mesh(filename=filename, scale=scale)
    else:
        return Geometry()


def parse_material_from_yourdfpy(material) -> Material:
    """Parse yourdfpy material into Material object.

    Args:
        material: yourdfpy material object

    Returns:
        Material instance
    """
    if material is None:
        return Material()

    mat = Material()
    if hasattr(material, "name"):
        mat.name = material.name

    if hasattr(material, "color") and material.color is not None:
        color = material.color
        if hasattr(color, "rgba"):
            mat.color = RGBA(rgba=list(color.rgba))
        elif isinstance(color, (list, tuple)):
            mat.color = RGBA(rgba=list(color))

    if hasattr(material, "texture") and material.texture is not None:
        mat.texture_filename = (
            material.texture.filename if hasattr(material.texture, "filename") else ""
        )

    return mat


def parse_inertial_from_yourdfpy(inertial) -> InertialProperties:
    """Parse yourdfpy inertial into InertialProperties.

    Args:
        inertial: yourdfpy inertial object

    Returns:
        InertialProperties instance
    """
    if inertial is None:
        return InertialProperties()

    mass = (
        float(inertial.mass.value)
        if hasattr(inertial, "mass") and hasattr(inertial.mass, "value")
        else 1.0
    )

    # Parse inertia tensor
    inertia_obj = Inertia()
    if hasattr(inertial, "inertia"):
        inert = inertial.inertia
        if hasattr(inert, "ixx"):
            inertia_obj.ixx = float(inert.ixx)
        if hasattr(inert, "ixy"):
            inertia_obj.ixy = float(inert.ixy)
        if hasattr(inert, "ixz"):
            inertia_obj.ixz = float(inert.ixz)
        if hasattr(inert, "iyy"):
            inertia_obj.iyy = float(inert.iyy)
        if hasattr(inert, "iyz"):
            inertia_obj.iyz = float(inert.iyz)
        if hasattr(inert, "izz"):
            inertia_obj.izz = float(inert.izz)

    # Parse origin
    pose = Pose()
    if hasattr(inertial, "origin"):
        xyz, rpy = parse_origin(inertial.origin)
        pose.xyz = xyz
        pose.rpy = rpy

    return InertialProperties(mass=mass, inertia=inertia_obj, origin=pose)


def parse_link_from_yourdfpy(link) -> LinkDescription:
    """Parse yourdfpy link into LinkDescription.

    Args:
        link: yourdfpy Link object

    Returns:
        LinkDescription instance
    """
    link_desc = LinkDescription(name=link.name)

    # Parse visual properties
    if hasattr(link, "visuals") and link.visuals:
        visuals = []
        for visual in link.visuals:
            vis_prop = VisualProperties()
            if hasattr(visual, "geometry"):
                vis_prop.geometry = parse_geometry_from_yourdfpy(visual.geometry)
            if hasattr(visual, "material"):
                vis_prop.material = parse_material_from_yourdfpy(visual.material)
            if hasattr(visual, "origin"):
                xyz, rpy = parse_origin(visual.origin)
                vis_prop.origin.xyz = xyz
                vis_prop.origin.rpy = rpy
            if hasattr(visual, "name"):
                vis_prop.name = visual.name
            visuals.append(vis_prop)

        link_desc.visual = visuals[0] if len(visuals) == 1 else visuals

    # Parse collision properties
    if hasattr(link, "collisions") and link.collisions:
        collisions = []
        for collision in link.collisions:
            col_prop = CollisionProperties()
            if hasattr(collision, "geometry"):
                col_prop.geometry = parse_geometry_from_yourdfpy(collision.geometry)
            if hasattr(collision, "origin"):
                xyz, rpy = parse_origin(collision.origin)
                col_prop.origin.xyz = xyz
                col_prop.origin.rpy = rpy
            if hasattr(collision, "name"):
                col_prop.name = collision.name
            collisions.append(col_prop)

        link_desc.collision = collisions[0] if len(collisions) == 1 else collisions

    # Parse inertial properties
    if hasattr(link, "inertial") and link.inertial:
        link_desc.inertial = parse_inertial_from_yourdfpy(link.inertial)

    return link_desc


def parse_joint_from_yourdfpy(joint) -> JointDescription:
    """Parse yourdfpy joint into JointDescription.

    Args:
        joint: yourdfpy Joint object

    Returns:
        JointDescription instance
    """
    joint_desc = JointDescription(name=joint.name)

    # Joint type
    if hasattr(joint, "type"):
        joint_desc.type = joint.type

    # Parent and child links via transform
    parent = joint.parent if hasattr(joint, "parent") else ""
    child = joint.child if hasattr(joint, "child") else ""
    origin = joint.origin if hasattr(joint, "origin") else None
    joint_desc.transform = yourdfpy_to_transform(parent, child, origin)

    # Joint axis
    if hasattr(joint, "axis") and joint.axis is not None:
        axis_list = (
            list(joint.axis.xyz) if hasattr(joint.axis, "xyz") else [1.0, 0.0, 0.0]
        )
        joint_desc.axis = Vector3(x=axis_list[0], y=axis_list[1], z=axis_list[2])

    # Joint limits
    if hasattr(joint, "limit") and joint.limit is not None:
        limits = PerJointLimits()
        if hasattr(joint.limit, "lower"):
            limits.min_position = float(joint.limit.lower)
        if hasattr(joint.limit, "upper"):
            limits.max_position = float(joint.limit.upper)
        if hasattr(joint.limit, "effort"):
            limits.max_effort = float(joint.limit.effort)
        if hasattr(joint.limit, "velocity"):
            limits.max_velocity = float(joint.limit.velocity)
        joint_desc.limits = limits

    # Joint dynamics/physics
    if hasattr(joint, "dynamics") and joint.dynamics is not None:
        physics = PerJointPhysics()
        if hasattr(joint.dynamics, "damping"):
            physics.damping = float(joint.dynamics.damping)
        if hasattr(joint.dynamics, "friction"):
            physics.friction = float(joint.dynamics.friction)
        joint_desc.physics = physics

    return joint_desc


def from_yourdfpy_urdf(
    urdf: "yourdfpy.URDF", robot_name: Optional[str] = None
) -> tuple[list[LinkDescription], list[JointDescription], RobotInfo]:
    """Convert yourdfpy URDF to RobotDescription components.

    Args:
        urdf: yourdfpy.URDF object
        robot_name: Optional name override for robot

    Returns:
        Tuple of (links, joints, robot_info)
    """
    links = []
    joints = []

    # Parse links from link_map
    if hasattr(urdf, "link_map"):
        for link_name, link in urdf.link_map.items():
            links.append(parse_link_from_yourdfpy(link))

    # Parse joints from joint_map
    if hasattr(urdf, "joint_map"):
        for joint_name, joint in urdf.joint_map.items():
            joints.append(parse_joint_from_yourdfpy(joint))

    # Create robot info - get name from robot attribute
    if robot_name:
        name = robot_name
    elif hasattr(urdf, "robot") and hasattr(urdf.robot, "name"):
        name = urdf.robot.name
    else:
        name = "robot"

    robot_info = RobotInfo(name=name)

    return links, joints, robot_info


def from_urdf_string(
    urdf_string: str, robot_name: Optional[str] = None
) -> tuple[list[LinkDescription], list[JointDescription], RobotInfo]:
    """Parse URDF string into RobotDescription components.

    Args:
        urdf_string: URDF XML string
        robot_name: Optional name override for robot

    Returns:
        Tuple of (links, joints, robot_info)
    """
    import io

    urdf = yourdfpy.URDF.load(
        io.StringIO(urdf_string), load_meshes=False, build_scene_graph=True
    )
    return from_yourdfpy_urdf(urdf, robot_name)


def from_urdf_file(
    urdf_path: str | Path, robot_name: Optional[str] = None
) -> tuple[list[LinkDescription], list[JointDescription], RobotInfo]:
    """Parse URDF file into RobotDescription components.

    Args:
        urdf_path: Path to URDF file
        robot_name: Optional name override for robot

    Returns:
        Tuple of (links, joints, robot_info)
    """
    urdf = yourdfpy.URDF.load(str(urdf_path), load_meshes=False, build_scene_graph=True)
    return from_yourdfpy_urdf(urdf, robot_name)


# Export functions


def geometry_to_urdf_xml(geom: Geometry, indent: str = "      ") -> str:
    """Convert Geometry to URDF XML string.

    Args:
        geom: Geometry object
        indent: Indentation string

    Returns:
        URDF XML string for geometry
    """
    if isinstance(geom, Box):
        size_str = " ".join([f"{v:.6f}" for v in geom.size])
        return f'{indent}<geometry>\n{indent}  <box size="{size_str}"/>\n{indent}</geometry>'
    elif isinstance(geom, Cylinder):
        return f'{indent}<geometry>\n{indent}  <cylinder radius="{geom.radius:.6f}" length="{geom.length:.6f}"/>\n{indent}</geometry>'
    elif isinstance(geom, Sphere):
        return f'{indent}<geometry>\n{indent}  <sphere radius="{geom.radius:.6f}"/>\n{indent}</geometry>'
    elif isinstance(geom, Mesh):
        scale_str = (
            " ".join([f"{v:.6f}" for v in geom.scale])
            if geom.scale != [1.0, 1.0, 1.0]
            else None
        )
        scale_attr = f' scale="{scale_str}"' if scale_str else ""
        return f'{indent}<geometry>\n{indent}  <mesh filename="{geom.filename}"{scale_attr}/>\n{indent}</geometry>'
    else:
        return f"{indent}<geometry/>"


def material_to_urdf_xml(material: Material, indent: str = "      ") -> str:
    """Convert Material to URDF XML string.

    Args:
        material: Material object
        indent: Indentation string

    Returns:
        URDF XML string for material
    """
    if not material.name and not material.color.rgba and not material.texture_filename:
        return ""

    name_attr = f' name="{material.name}"' if material.name else ""
    lines = [f"{indent}<material{name_attr}>"]

    if material.color.rgba and material.color.rgba != [0.0, 0.0, 0.0, 0.0]:
        rgba_str = " ".join([f"{v:.6f}" for v in material.color.rgba])
        lines.append(f'{indent}  <color rgba="{rgba_str}"/>')

    if material.texture_filename:
        lines.append(f'{indent}  <texture filename="{material.texture_filename}"/>')

    lines.append(f"{indent}</material>")
    return "\n".join(lines)


def origin_to_urdf_xml(pose: Pose, indent: str = "      ") -> str:
    """Convert Pose to URDF origin XML string.

    Args:
        pose: Pose object with xyz and rpy
        indent: Indentation string

    Returns:
        URDF XML string for origin
    """
    xyz = pose.xyz if hasattr(pose, "xyz") else [0.0, 0.0, 0.0]
    rpy = pose.rpy if hasattr(pose, "rpy") else [0.0, 0.0, 0.0]

    # Only output origin if it's not identity
    if xyz == [0.0, 0.0, 0.0] and rpy == [0.0, 0.0, 0.0]:
        return ""

    xyz_str = " ".join([f"{v:.6f}" for v in xyz])
    rpy_str = " ".join([f"{v:.6f}" for v in rpy])
    return f'{indent}<origin xyz="{xyz_str}" rpy="{rpy_str}"/>'


def inertial_to_urdf_xml(inertial: InertialProperties, indent: str = "    ") -> str:
    """Convert InertialProperties to URDF XML string.

    Args:
        inertial: InertialProperties object
        indent: Indentation string

    Returns:
        URDF XML string for inertial
    """
    if inertial.mass == 0.0:
        return ""

    lines = [f"{indent}<inertial>"]

    # Origin
    origin_xml = origin_to_urdf_xml(inertial.origin, indent + "  ")
    if origin_xml:
        lines.append(origin_xml)

    # Mass
    lines.append(f'{indent}  <mass value="{inertial.mass:.6f}"/>')

    # Inertia
    inert = inertial.inertia
    lines.append(
        f'{indent}  <inertia ixx="{inert.ixx:.6f}" ixy="{inert.ixy:.6f}" ixz="{inert.ixz:.6f}" '
        f'iyy="{inert.iyy:.6f}" iyz="{inert.iyz:.6f}" izz="{inert.izz:.6f}"/>'
    )

    lines.append(f"{indent}</inertial>")
    return "\n".join(lines)


def visual_to_urdf_xml(
    visual: VisualProperties, indent: str = "    ", name: str = ""
) -> str:
    """Convert VisualProperties to URDF XML string.

    Args:
        visual: VisualProperties object
        indent: Indentation string
        name: Optional name for visual element

    Returns:
        URDF XML string for visual
    """
    name_attr = f' name="{name}"' if name else ""
    lines = [f"{indent}<visual{name_attr}>"]

    # Origin
    origin_xml = origin_to_urdf_xml(visual.origin, indent + "  ")
    if origin_xml:
        lines.append(origin_xml)

    # Geometry
    geom_xml = geometry_to_urdf_xml(visual.geometry, indent + "  ")
    lines.append(geom_xml)

    # Material
    mat_xml = material_to_urdf_xml(visual.material, indent + "  ")
    if mat_xml:
        lines.append(mat_xml)

    lines.append(f"{indent}</visual>")
    return "\n".join(lines)


def collision_to_urdf_xml(
    collision: CollisionProperties, indent: str = "    ", name: str = ""
) -> str:
    """Convert CollisionProperties to URDF XML string.

    Args:
        collision: CollisionProperties object
        indent: Indentation string
        name: Optional name for collision element

    Returns:
        URDF XML string for collision
    """
    name_attr = f' name="{name}"' if name else ""
    lines = [f"{indent}<collision{name_attr}>"]

    # Origin
    origin_xml = origin_to_urdf_xml(collision.origin, indent + "  ")
    if origin_xml:
        lines.append(origin_xml)

    # Geometry
    geom_xml = geometry_to_urdf_xml(collision.geometry, indent + "  ")
    lines.append(geom_xml)

    lines.append(f"{indent}</collision>")
    return "\n".join(lines)


def link_to_urdf_xml(link: LinkDescription, indent: str = "  ") -> str:
    """Convert LinkDescription to URDF XML string.

    Args:
        link: LinkDescription object
        indent: Indentation string

    Returns:
        URDF XML string for link
    """
    lines = [f'{indent}<link name="{link.name}">']

    # Inertial
    if link.inertial and link.inertial.mass > 0:
        lines.append(inertial_to_urdf_xml(link.inertial, indent + "  "))

    # Visual(s)
    if isinstance(link.visual, list):
        for i, visual in enumerate(link.visual):
            vis_name = (
                visual.name
                if hasattr(visual, "name") and visual.name
                else f"visual_{i}"
            )
            lines.append(visual_to_urdf_xml(visual, indent + "  ", vis_name))
    elif link.visual and not isinstance(link.visual, type(VisualProperties())):
        # Single visual that's been set
        vis_name = (
            link.visual.name
            if hasattr(link.visual, "name") and link.visual.name
            else ""
        )
        lines.append(visual_to_urdf_xml(link.visual, indent + "  ", vis_name))

    # Collision(s)
    if isinstance(link.collision, list):
        for i, collision in enumerate(link.collision):
            col_name = (
                collision.name
                if hasattr(collision, "name") and collision.name
                else f"collision_{i}"
            )
            lines.append(collision_to_urdf_xml(collision, indent + "  ", col_name))
    elif link.collision and not isinstance(link.collision, type(CollisionProperties())):
        # Single collision that's been set
        col_name = (
            link.collision.name
            if hasattr(link.collision, "name") and link.collision.name
            else ""
        )
        lines.append(collision_to_urdf_xml(link.collision, indent + "  ", col_name))

    lines.append(f"{indent}</link>")
    return "\n".join(lines)


def joint_to_urdf_xml(joint: JointDescription, indent: str = "  ") -> str:
    """Convert JointDescription to URDF XML string.

    Args:
        joint: JointDescription object
        indent: Indentation string

    Returns:
        URDF XML string for joint
    """
    lines = [f'{indent}<joint name="{joint.name}" type="{joint.type}">']

    # Origin from transform
    if joint.transform:
        xyz_str = " ".join([f"{v:.6f}" for v in joint.transform.xyz])
        rpy_str = " ".join([f"{v:.6f}" for v in joint.transform.rpy])
        lines.append(f'{indent}  <origin xyz="{xyz_str}" rpy="{rpy_str}"/>')

    # Parent and child links
    parent = joint.transform.header.frame_id if joint.transform else ""
    child = joint.transform.child_frame_id if joint.transform else ""
    if parent:
        lines.append(f'{indent}  <parent link="{parent}"/>')
    if child:
        lines.append(f'{indent}  <child link="{child}"/>')

    # Axis (for revolute/prismatic joints)
    if joint.type in ["revolute", "continuous", "prismatic"] and joint.axis:
        axis_str = f"{joint.axis.x:.6f} {joint.axis.y:.6f} {joint.axis.z:.6f}"
        lines.append(f'{indent}  <axis xyz="{axis_str}"/>')

    # Limits (for non-fixed joints)
    if joint.type != "fixed" and joint.limits:
        limits = joint.limits
        lower = limits.min_position if limits.min_position is not None else 0.0
        upper = limits.max_position if limits.max_position is not None else 0.0
        effort = limits.max_effort if limits.max_effort is not None else 0.0
        velocity = limits.max_velocity if limits.max_velocity is not None else 0.0
        lines.append(
            f'{indent}  <limit lower="{lower:.6f}" upper="{upper:.6f}" '
            f'effort="{effort:.6f}" velocity="{velocity:.6f}"/>'
        )

    # Dynamics
    if joint.physics and (
        joint.physics.damping != 0.0 or joint.physics.friction != 0.0
    ):
        lines.append(
            f'{indent}  <dynamics damping="{joint.physics.damping:.6f}" '
            f'friction="{joint.physics.friction:.6f}"/>'
        )

    lines.append(f"{indent}</joint>")
    return "\n".join(lines)


def to_urdf_string(
    links: list[LinkDescription],
    joints: list[JointDescription],
    robot_name: str = "robot",
) -> str:
    """Convert links and joints to URDF XML string.

    Args:
        links: List of LinkDescription objects
        joints: List of JointDescription objects
        robot_name: Name for the robot

    Returns:
        Complete URDF XML string
    """
    lines = ['<?xml version="1.0"?>']
    lines.append(f'<robot name="{robot_name}">')

    # Add all links
    for link in links:
        lines.append(link_to_urdf_xml(link))

    # Add all joints
    for joint in joints:
        lines.append(joint_to_urdf_xml(joint))

    lines.append("</robot>")
    return "\n".join(lines)


if __name__ == "__main__":
    # Test with a simple URDF string
    test_urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>"""

    links, joints, robot_info = from_urdf_string(test_urdf)
    print(f"Parsed {len(links)} links and {len(joints)} joints")
    print(f"Robot name: {robot_info.name}")

    # Test round-trip
    urdf_out = to_urdf_string(links, joints, robot_info.name)
    print("\nGenerated URDF:")
    print(urdf_out)
