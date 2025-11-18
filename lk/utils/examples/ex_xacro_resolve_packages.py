#!/usr/bin/env python3

"""
    Example: Resolving Package Paths in Xacro Files

    Demonstrates how to use resolve_package_paths to convert package:// URIs
    to file:// URIs using a custom package path dictionary.
"""

# BAM
from bam.utils.xacro_utils import resolve_package_paths, post_process_dom

# PYTHON
from pathlib import Path
from xml.dom.minidom import parseString


if __name__ == "__main__":
    # Example URDF content with package:// URIs
    urdf_content = """<?xml version="1.0"?>
    <robot name="example_robot">
        <link name="base_link">
            <visual>
                <geometry>
                    <mesh filename="file://ur_description/meshes/ur5/visual/base.dae"/>
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://ur_description/meshes/ur5/collision/base.stl"/>
                </geometry>
            </collision>
        </link>
        <material name="blue">
            <color rgba="0 0 1 1"/>
        </material>
    </robot>
    """

    # Parse the URDF into a DOM
    dom = parseString(urdf_content)

    # Define package paths mapping
    # In practice, you would get these from your workspace structure
    package_paths = {
        "ur_description": "/path/to/your/ur_description",
    }

    post_process_dom(dom)

    # # Resolve package:// URIs to file:// URIs
    # resolve_package_paths(dom, package_paths)

    # # Print the modified URDF
    print("Modified URDF:")
    print(dom.toprettyxml(indent="  "))

