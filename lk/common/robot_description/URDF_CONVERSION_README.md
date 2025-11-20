# URDF Import/Export Functionality

This module provides seamless conversion between URDF files and RobotDescription objects using the `yourdfpy` library.

## Features

- ✅ Parse URDF files/strings into RobotDescription objects
- ✅ Export RobotDescription to URDF format
- ✅ Support for all geometry types (box, cylinder, sphere, mesh)
- ✅ Preserve inertial properties, joint limits, and dynamics
- ✅ Handle multiple visual/collision elements per link
- ✅ Round-trip conversion with validation

## Installation

```bash
pip install yourdfpy
```

Optional: For testing with real robot URDFs
```bash
pip install robot_descriptions
```

## Usage

### Import from URDF

```python
from lk.common.robot_description import RobotDescription

# From URDF file
robot_desc = RobotDescription.from_urdf_file('path/to/robot.urdf')

# From URDF string
urdf_xml = """<?xml version="1.0"?>
<robot name="my_robot">
  ...
</robot>"""
robot_desc = RobotDescription.from_urdf_xml(urdf_xml)

# Optionally override robot name
robot_desc = RobotDescription.from_urdf_file('robot.urdf', robot_name='my_custom_name')
```

### Export to URDF

```python
# Export RobotDescription to URDF XML string
urdf_xml = robot_desc.to_urdf_xml()

# Save to file
with open('exported_robot.urdf', 'w') as f:
    f.write(urdf_xml)
```

### Round-Trip Conversion

```python
# Load, modify, and export
robot_desc = RobotDescription.from_urdf_file('original.urdf')

# Make modifications to robot_desc...
robot_desc.info.name = "modified_robot"

# Export back to URDF
urdf_xml = robot_desc.to_urdf_xml()
```

## What Gets Preserved

The converter preserves:
- Link names and structure
- Joint names, types (revolute, prismatic, fixed, continuous, etc.)
- Parent/child link relationships
- Joint transforms (xyz, rpy)
- Joint axes
- Joint limits (position, effort, velocity)
- Joint dynamics (damping, friction)
- Inertial properties (mass, inertia tensor, origin)
- Visual properties (geometry, material, origin)
- Collision properties (geometry, origin)
- Multiple visual/collision elements per link

## Supported Geometry Types

- **Box**: size (x, y, z)
- **Cylinder**: radius, length
- **Sphere**: radius
- **Mesh**: filename, scale

## Testing

### Unit Tests

```bash
cd /path/to/like_keras
python lk/common/robot_description/tests/test_urdf_conversion.py
```

### Round-Trip Tests (with robot_descriptions)

```bash
# Requires: pip install robot_descriptions
python lk/common/robot_description/tests/test_urdf_roundtrip.py
```

This will test conversion on real robot URDFs including:
- Universal Robots (UR5, UR10)
- Franka Emika Panda
- KUKA IIWA
- Kinova Gen3
- And more...

### Demo

```bash
python lk/common/robot_description/demo_urdf_conversion.py
```

## Implementation Details

### Architecture

The conversion is implemented in three layers:

1. **urdf_converter.py**: Low-level conversion functions
   - `from_urdf_string()`: Parse URDF XML → Links/Joints lists
   - `from_urdf_file()`: Parse URDF file → Links/Joints lists
   - `to_urdf_string()`: Links/Joints lists → URDF XML

2. **robot_description.py**: High-level RobotDescription methods
   - `RobotDescription.from_urdf_xml()`: URDF XML → RobotDescription
   - `RobotDescription.from_urdf_file()`: URDF file → RobotDescription
   - `RobotDescription.to_urdf_xml()`: RobotDescription → URDF XML

3. **Description types**: Data structures that mirror URDF elements
   - LinkDescription, JointDescription
   - Geometry types (Box, Cylinder, Sphere, Mesh)
   - Properties (Inertial, Visual, Collision)

### yourdfpy Integration

We use `yourdfpy` for parsing because it:
- Is pure Python (no C++ dependencies)
- Handles malformed URDFs gracefully
- Provides clean attribute access to URDF elements
- Has good performance

### Differences from Original URDF

When round-tripping, the exported URDF may differ from the original in:
- Whitespace and formatting
- Order of elements (links before joints)
- Numeric precision (6 decimal places)
- Default values may be explicit
- Comments are not preserved

However, the **semantic content** is preserved, meaning the parsed URDF structures will be equivalent.

## Inspiration

This implementation was inspired by:
- [pyppet](https://github.com/david-dorf/pyppet) - Python-based robot model format
- [yourdfpy](https://github.com/clemense/yourdfpy) - Robust URDF parser

## Example

```python
from lk.common.robot_description import RobotDescription

# Load UR10 from robot_descriptions (if installed)
try:
    from robot_descriptions import ur10_description
    robot_desc = RobotDescription.from_urdf_file(
        ur10_description.URDF_PATH, 
        robot_name="ur10"
    )
    print(f"Loaded {len(robot_desc.links.entities)} links")
    print(f"Loaded {len(robot_desc.joints.entities)} joints")
    
    # Export and verify
    urdf_xml = robot_desc.to_urdf_xml()
    print(f"Exported URDF: {len(urdf_xml)} characters")
    
except ImportError:
    print("robot_descriptions not installed")
```

## Future Enhancements

Potential improvements:
- Support for URDF transmission elements
- Support for Gazebo-specific tags
- Material library handling
- Package path resolution
- URDF validation
- SDF (Simulation Description Format) support




