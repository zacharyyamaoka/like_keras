#!/usr/bin/env python3

"""
CompositeDescription

Generic class for combining multiple RobotDescriptions with mounting joints.
Enables flexible composition without creating specialized classes.
"""

# BAM
from typing import Optional
from bam.descriptions import BAM_DESCRIPTIONS_PATH, BAM_MESH_PACKAGE_PATH
from .description_types import LinkDescription, JointDescription


from bam.utils import get_file_name_timestamp, xml_from_xacro, xml_body_from_macro_xml

# PYTHON
from dataclasses import dataclass, field
import os
import textwrap
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription


def generate_composite_xml_body(description: "RobotDescription") -> str:
    """Generate URDF XML body programmatically from a RobotDescription.

    This function generates XML directly from the links and joints in the description.
    All links and joints from children are already merged into the top-level description
    when descriptions are combined, so we only need to generate from the top-level.

    Args:
        description: RobotDescription to generate XML for

    Returns:
        str: The generated URDF XML body content
    """
    # Generate XML directly from links and joints
    # Links XML (each link already has proper indentation)
    links_xml = "\n".join(
        [link.to_xml() for link in description.links.entities.values()]
    )

    # Joints XML (each joint already has proper indentation and newlines)
    joints_xml = "\n".join(
        [joint.to_xml() for joint in description.joints.entities.values()]
    )

    # Combine with proper spacing
    body_parts = []
    if links_xml:
        body_parts.append(links_xml)
    if joints_xml:
        body_parts.append(joints_xml)

    body_content = "\n\n".join(body_parts) if body_parts else ""

    urdf_content = textwrap.dedent(
        f"""
        <!--
        Auto-generated URDF XML Body
        Generated programmatically from RobotDescription

        Note: This XML was generated directly from links and joints in the description.
        All children links and joints are already merged into the top-level description.
        -->
        {body_content}
    """
    ).strip()

    return urdf_content
