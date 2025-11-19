#!/usr/bin/env python3

"""
    Display URDF/Xacro Viewer using PinViser

    A simple utility to visualize any URDF or xacro file using the PinViser viewer.
    Automatically detects file type and mesh directories.

    Usage:
        display_urdf.py <path_to_urdf_or_xacro> [xacro_arg1=value1] [xacro_arg2=value2] ...

    Examples:
        display_urdf.py robot.urdf
        display_urdf.py robot.urdf.xacro
        display_urdf.py robot.urdf.xacro use_gripper:=true sim_mode:=false
"""

# BAM
from bam.utils.pin_utils.pin_viser import PinViser
import bam.descriptions as desc

# PYTHON
import sys
import argparse
from pathlib import Path


def get_mesh_package_dirs() -> dict[str, str]:
    """
    Get all available mesh package directories from bam.descriptions.
    
    Pinocchio will intelligently substitute package:// URIs using these directories.
    """
    mesh_dirs = {}
    
    # Add all available mesh package directories
    if desc.BAM_MESH_PACKAGE_PATH:
        mesh_dirs['bam_descriptions'] = desc.BAM_MESH_PACKAGE_PATH
    
    if desc.UR_MESH_PACKAGE_PATH:
        mesh_dirs['ur_description'] = desc.UR_MESH_PACKAGE_PATH
    
    if desc.ROBOTIQ_MESH_PACKAGE_PATH:
        mesh_dirs['robotiq_description'] = desc.ROBOTIQ_MESH_PACKAGE_PATH
    
    return mesh_dirs


def parse_xacro_args(args: list[str]) -> dict:
    """
    Parse xacro arguments from command line.
    
    Format: key:=value or key=value
    """
    xacro_args = {}
    
    for arg in args:
        if ':=' in arg:
            key, value = arg.split(':=', 1)
        elif '=' in arg:
            key, value = arg.split('=', 1)
        else:
            print(f"Warning: Ignoring invalid argument format: {arg}")
            continue
        
        # Try to convert to bool
        if value.lower() in ['true', 'false']:
            value = value.lower() == 'true'
        # Try to convert to int
        elif value.isdigit():
            value = int(value)
        # Try to convert to float
        else:
            try:
                value = float(value)
            except ValueError:
                pass  # Keep as string
        
        xacro_args[key] = value
    
    return xacro_args


def main():
    parser = argparse.ArgumentParser(
        description='Display URDF/Xacro files using PinViser',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s robot.urdf
  %(prog)s robot.urdf.xacro
  %(prog)s robot.urdf.xacro use_gripper:=true sim_mode:=false
        """
    )
    
    parser.add_argument('file_path', type=str, help='Path to URDF or xacro file')
    parser.add_argument('xacro_args', nargs='*', help='Xacro arguments in format key:=value')
    parser.add_argument('--port', type=str, default='', help='Viser port (optional)')
    parser.add_argument('--color', type=str, default=None, help='Robot color (optional)')
    
    args = parser.parse_args()
    
    file_path = Path(args.file_path).resolve()
    
    if not file_path.exists():
        print(f"Error: File not found: {file_path}")
        sys.exit(1)
    
    # Get mesh package directories from bam.descriptions
    mesh_package_dirs = get_mesh_package_dirs()
    print(f"Using mesh package directories: {list(mesh_package_dirs.values())}")
    
    # Check if it's a xacro file
    is_xacro = file_path.suffix == '.xacro' or '.xacro' in file_path.name
    
    try:
        if is_xacro:
            # Parse xacro arguments
            xacro_args_dict = parse_xacro_args(args.xacro_args)
            print(f"Loading xacro: {file_path}")
            if xacro_args_dict:
                print(f"Xacro arguments: {xacro_args_dict}")
            
            # Create viewer from xacro
            viz = PinViser.from_xacro(
                xacro_path=str(file_path),
                xacro_args=xacro_args_dict,
                mesh_package_dirs=mesh_package_dirs,
                port=args.port,
                color=args.color
            )
        else:
            # Create viewer from URDF
            print(f"Loading URDF: {file_path}")
            viz = PinViser.from_urdf(
                urdf_path=str(file_path),
                mesh_package_dirs=mesh_package_dirs,
                port=args.port,
                color=args.color
            )
        
        print("\n" + "="*60)
        print("PinViser viewer is running!")
        print(f"View in browser at: http://localhost:{viz.port}")
        print("="*60 + "\n")
        
        # Block until user input
        viz.block_until_input()
        
        print("Exiting viewer...")
        
    except Exception as e:
        print(f"Error loading robot model: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

