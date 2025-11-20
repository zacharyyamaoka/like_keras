# Robot Description

Python-based robot description system with URDF import/export capabilities.

## Features

- ✅ **URDF Import/Export**: Load and save robot models in URDF format
- ✅ **Type-safe Descriptions**: Full type hints for all components
- ✅ **Programmatic Generation**: Build robots using Python code
- ✅ **Composite Robots**: Combine multiple robot descriptions
- ✅ **Configuration Management**: YAML-based configuration
- ✅ **Visualization Support**: RViz integration

## Quick Start

### Import from URDF

```python
from lk.common.robot_description import RobotDescription

# Load from URDF file
robot_desc = RobotDescription.from_urdf_file('path/to/robot.urdf')

# Load from URDF string
robot_desc = RobotDescription.from_urdf_xml(urdf_xml_string)
```

### Export to URDF

```python
# Export to URDF XML string
urdf_xml = robot_desc.to_urdf_xml()

# Save to file
with open('robot.urdf', 'w') as f:
    f.write(urdf_xml)
```

## Documentation

- **[URDF Conversion Guide](URDF_CONVERSION_README.md)** - Complete guide to URDF import/export
- **[Implementation Summary](URDF_IMPLEMENTATION_SUMMARY.md)** - Technical details
- **[Design Notes](design-notes.md)** - Design decisions and rationale

## Installation

```bash
# For URDF support
pip install yourdfpy

# Optional: For testing with real robots
pip install robot_descriptions
```

## Classes and Helpers

Main classes for robot description system. See individual files for details.
