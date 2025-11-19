#!/usr/bin/env python3

"""
    Xacro Package Path Resolution Utilities

    Utilities for resolving package:// URIs in xacro/URDF files using custom
    package path mappings instead of ROS package resolution.

    Based on xacrodoc: https://github.com/adamheins/xacrodoc/blob/main/src/xacrodoc/xacrodoc.py#L96C1-L97C1

    def _resolve_packages(dom):

    Needed to be adjusted as we don't nessarily use ROS Packages
"""


# PYTHON
import re
import uuid
from pathlib import Path
from xml.dom.minidom import Document

from xacrodoc.xacrodoc import XacroDoc

from .tempfile_utils import temp_xacro_file


def xml_from_xacro(xacro_path: str, xacro_args: dict = None, resolve_packages: bool = False, abs_package_dirs: dict[str, str] = None) -> str:

    """
       directly generate xml to avoid xacrodoc editing the filenames
    """
    doc = xacrodoc_from_file(xacro_path, xacro_args, resolve_packages, abs_package_dirs)
    return doc.dom.toprettyxml(indent="  ")

def xml_body_from_macro_xml(
    macro_xml: str,
    imports: list[str],
    prefix_comment: str = "start macro xml",
    suffix_comment: str = "end macro xml",
    resolve_packages: bool = False,
    abs_package_dirs: dict[str, str] = None
) -> str:
    """Extract expanded XML from macro XML by creating a temporary xacro and expanding it.
    
    Creates a lightweight temporary xacro file that includes the specified imports,
    then includes the macro XML between marker links. After expansion, extracts
    the content between the markers and returns it.
    
    DescriptionArgs:
        macro_xml: The XML that calls the macro (e.g., xacro:include and xacro:make_config tags)
        imports: List of xacro file paths to include before the macro
        prefix_comment: Comment to add before the extracted content (default: "start macro xml")
        suffix_comment: Comment to add after the extracted content (default: "end macro xml")
        resolve_packages: Whether to resolve package:// URIs to file:// paths
        
    Returns:
        str: The expanded XML content extracted from between the markers
        
    Raises:
        ValueError: If the marker links cannot be found in the expanded XML
    """
    # Create unique marker links to identify the macro content
    # Use <link> tags instead of comments to avoid issues with xacro expansion, other ones disappear!
    marker_id = str(uuid.uuid4())[:8]
    start_marker = f'<link name="start_marker_{marker_id}"/>'
    end_marker = f'<link name="end_marker_{marker_id}"/>'
    
    # Build imports section
    imports_section = "\n".join([f'  <xacro:include filename="{imp}"/>' for imp in imports])
    
    # Create lightweight temp xacro content
    temp_xacro_content = f'''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="temp_robot">

{imports_section}

{start_marker}

{macro_xml}

{end_marker}

</robot>
'''
    
    # Expand the xacro and extract the macro content
    with temp_xacro_file(temp_xacro_content) as temp_xacro_path:
        expanded_xml = xml_from_xacro(temp_xacro_path, None, resolve_packages=resolve_packages, abs_package_dirs=abs_package_dirs)
        
        # Extract content between markers
        start_idx = expanded_xml.find(start_marker)
        end_idx = expanded_xml.find(end_marker)
        
        if start_idx == -1 or end_idx == -1:
            raise ValueError(f"Could not find macro markers in expanded XML. Start: {start_idx}, End: {end_idx}")
        
        # Extract content between markers
        macro_content = expanded_xml[start_idx + len(start_marker):end_idx].strip()
        
        # Build result with prefix and suffix comments
        return f"  <!-- {prefix_comment} -->\n{macro_content}\n  <!-- {suffix_comment} -->"

def xacrodoc_from_file(xacro_path: str, xacro_args: dict = None, resolve_packages: bool = False, abs_package_dirs: dict[str, str] = None) -> XacroDoc:
    # Register packages with xacrodoc if provided
    if abs_package_dirs and resolve_packages:
        import xacrodoc.packages
        xacrodoc.packages.update_package_cache(abs_package_dirs)
    
    doc = XacroDoc.from_file(xacro_path, subargs=xacro_args, resolve_packages=resolve_packages)

    """
    resolve_package True vs False
    <mesh filename="file://ur_description/meshes/ur5e/visual/shoulder.dae"/>
    <mesh filename="file:///home/bam/bam_ws/src/ur_description/meshes/ur5e/visual/wrist2.dae"/>

    We need to rename ur_description to ur_descriptions, otherwise the global path will not work.

    So... I feel the urdf should be written with relative paths... perhaps its just about replacing file:// with package:// ?

    I think its reasonable I can go in here and manually edit the dom as needed... 
    """

    # No need for this anymore as we removed the package:// strip
    # if not resolve_packages: # Put back in package:// so that pinocchio can resolve it

    #     replace_file_with_package(doc.dom)


    return doc

def _urdf_elements_with_filenames(dom: Document) -> list:
    """Get all elements in the URDF document with a filename attribute.

    Parameters
    ----------
    dom : xml.dom.minidom.Document
        The XML document.

    Returns
    -------
    list
        List of elements which have a filename attribute.
    """
    elements = dom.getElementsByTagName("mesh") + dom.getElementsByTagName("material")
    return [e for e in elements if e.hasAttribute("filename")]


def post_process_dom(dom: Document) -> None:

    for e in _urdf_elements_with_filenames(dom):
        filename = e.getAttribute("filename")
        print(filename)

        e.setAttribute("filename", filename.replace("file://", "package://"))

def replace_file_with_package(dom: Document) -> str:
    # print(dom.toprettyxml(indent="  "))

    for e in _urdf_elements_with_filenames(dom):
        filename = e.getAttribute("filename")
        # print(filename)

        e.setAttribute("filename", filename.replace("file://", "package://"))



def resolve_package_paths(dom: Document, package_paths: dict[str, str | Path]) -> None:
    """Convert all filenames specified with package:// to full absolute paths.

    This function modifies the DOM in place, replacing package:// URIs with
    file:// URIs using the provided package path mappings.

    Parameters
    ----------
    dom : xml.dom.minidom.Document
        The XML document, which is modified in place.
    package_paths : dict[str, str | Path]
        Dictionary mapping package names to their absolute paths.
        Example: {"ur_description": "/path/to/ur_description"}

    Raises
    ------
    ValueError
        If a package name contains spaces (not allowed).
        If a package:// URI references a package not in package_paths.
    """
    pkg_regex = re.compile(r"package://([ \w-]+)/")
    for e in _urdf_elements_with_filenames(dom):
        filename = e.getAttribute("filename")
        if filename.startswith("package://"):
            match = pkg_regex.search(filename)
            if not match:
                continue
            pkg = match.group(1)

            # ROS doesn't support spaces in package names, and neither do we
            # explicitly check and tell the user about this
            if " " in pkg:
                raise ValueError(
                    f"Package name '{pkg}' contains spaces, which is not allowed."
                )

            # Check if package is in the provided mapping
            if pkg not in package_paths:
                raise ValueError(
                    f"Package '{pkg}' not found in package_paths dictionary. "
                    f"Available packages: {list(package_paths.keys())}"
                )

            # Get absolute path and convert to posix format
            abspath = Path(package_paths[pkg]).absolute().as_posix()
            filename = re.sub(f"package://{pkg}", f"file://{abspath}", filename)
            e.setAttribute("filename", filename)


__all__ = [
    "xml_from_xacro",
    "xml_body_from_macro_xml",
    "xacrodoc_from_file",
    "post_process_dom",
    "replace_file_with_package",
    "resolve_package_paths",
]
