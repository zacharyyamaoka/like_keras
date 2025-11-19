from bam.msgs.ros_msgs import Pose, Transform, PoseStamped, TransformStamped
from .description_types import ConfigFileInfo

from .joint_positions import JointPositions


from enum import Enum
import numpy as np

from typing import TYPE_CHECKING
# from .robot_description import RobotDescription

"""
Idea is to format the data structures for xacro.

"""
def format_for_xacro(obj) -> dict[str, str]:
    if isinstance(obj, dict):
        return {k: format_for_xacro(v) for k, v in obj.items()}
    # BUG: If we convert all to dict we lose the type information
    # if hasattr(obj, "__dataclass_fields__"):
    #     # Convert dataclass to dict and recursively format
    #     return {k: format_for_xacro(v) for k, v in asdict(obj).items()}
    # If obj is of PoseType, convert to dict with "xyz" and "rpy"
    if isinstance(obj, (Pose, Transform, PoseStamped, TransformStamped)):
        return {
            "xyz": format_for_xacro(obj.xyz.tolist()),
            "rpy": format_for_xacro(obj.rpy.tolist())
        }

    if isinstance(obj, JointPositions):
        # Convert JointPositions to dict for xacro compatibility
        return {
            "joint_names": format_for_xacro(obj.joint_names),
            "positions": {k: format_for_xacro(v) for k, v in obj.positions.items()},
            "default_positions": obj.default_positions
        }

    if isinstance(obj, ConfigFileInfo): # return so you can easily load into xacro arg
        return obj.path
        
    # Do special procesing above ^^^, before turning dataclass into dict

    if hasattr(obj, "__dataclass_fields__"):
        # Manually iterate through fields instead of using asdict()
        # This allows us to check each field's type before converting
        # Use __dict__ instead of __dataclass_fields__ to include dynamically added fields
        # (e.g., from Joints.combine() which adds fields via setattr)
        result = {}
        for field_name in obj.__dict__:
            # Skip private fields
            if field_name.startswith('_'):
                continue
            field_value = getattr(obj, field_name)
            result[field_name] = format_for_xacro(field_value)
        return result
    if isinstance(obj, bool):
        # Format bools as lowercase strings for xacro
        return "true" if obj else "false"
    if isinstance(obj, Enum):  # Handle enums
        return str(obj.value)
    if isinstance(obj, str):
        return obj
    if isinstance(obj, (list, tuple)):
        # Check if all elements are simple types (can be converted directly to string)
        simple_types = (int, float, str, bool, np.float32, np.float64, Enum)
        all_simple = all(isinstance(item, simple_types) for item in obj)
        
        if all_simple and len(obj) > 0:
            # Convert simple list/tuple to space-separated string for xacro
            return ' '.join(map(str, obj))
        else:
            return [format_for_xacro(item) for item in obj]
    if isinstance(obj, int):
        return str(obj)
    if isinstance(obj, (float, np.float32, np.float64)):
        return str(round(float(obj), 6))
    if obj is None:
        return ""
    else:
        assert False, f"Unsupported type: {type(obj)}"