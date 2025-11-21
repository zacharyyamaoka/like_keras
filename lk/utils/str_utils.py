#!/usr/bin/env python3

"""
String formatting and manipulation utilities

Pure Python utilities for string operations including:
- Case conversion (camel to snake)
- Timestamp and hash formatting for file names
- Name extraction from file paths
- Enum to string conversion
"""

# PYTHON
import re
import hashlib
from typing import Type
from datetime import datetime
from pprint import pprint
import json


def get_file_name_timestamp():
    """Generate a timestamp string suitable for file names."""
    return datetime.now().strftime("%I_%M%p_%d_%b_%Y").lower()


def get_file_name_hash(input_str: str, length: int = 12) -> str:
    """Generate a hash string suitable for file names.

    DescriptionArgs:
        input_str: String to hash
        length: Length of hash to return (default 12)

    Returns:
        Hex hash string of specified length
    """
    hash_full = hashlib.sha256(input_str.encode()).hexdigest()
    return hash_full[:length]


def camel_to_snake(CamelCaseString: str):
    """Convert CamelCase string to snake_case.

    Handles common acronyms like API, ROS, IK, FK, DH, UR, BAM.
    """
    for acronym in ["API", "ROS", "IK", "FK", "DH", "UR", "BAM"]:
        CamelCaseString = CamelCaseString.replace(acronym, acronym.capitalize())

    snake_case_string = re.sub(r"(?<!^)(?=[A-Z])", "_", CamelCaseString).lower()
    return snake_case_string


def node_name_from_file(file_path):
    """Extract node name from file path by removing .py and _node suffix."""
    name = file_path.split("/")[-1].replace(".py", "").replace("_node", "")
    return name


def instance_to_snake(type_instance):
    """Convert class instance name to snake_case."""
    return camel_to_snake(type_instance.__class__.__name__)


def type_to_name(msg_type):
    """Convert type name to snake_case."""
    type_name = msg_type.__name__
    return camel_to_snake(type_name)


def enum_to_str(enum_class: Type, value: int) -> str:
    """Converts an enum value to its string name.

    DescriptionArgs:
        enum_class: The class containing the enum constants.
        value: The integer value of the enum.
    Returns:
        str: Formatted like "SUCCESS(1)" or "UNKNOWN NAME(42)"
    """
    mapping = {}
    # Iterate over all attributes in the enum class
    for attr_name in dir(enum_class):
        # Consider only uppercase attributes (common convention for enums)
        if attr_name.isupper():
            attr_value = getattr(enum_class, attr_name)
            # Check if the attribute is an integer (enum values are typically int)
            if isinstance(attr_value, int):
                mapping[attr_value] = attr_name

    name = mapping.get(value, "UNKNOWN NAME")
    return f"{name}({value})"


def server_node_name_from_type(client_or_server_type):
    """Extract server node name from client or server type."""
    name = type_to_name(client_or_server_type)
    if "client" in name:
        return name.replace("client", "server")
    elif "server" in name:
        return name
    else:
        msg = f"Couldn't extract valid server name from: {name}"
        raise ValueError(msg)


if __name__ == "__main__":
    # Test the string utilities
    print("== TESTING STRING UTILITIES ==")

    print("\nFile name utilities:")
    print("get_file_name_timestamp():", get_file_name_timestamp())
    print(
        "get_file_name_hash('my_config_file', 12):",
        get_file_name_hash("my_config_file", 12),
    )
    print(
        "get_file_name_hash('my_config_file', 8):",
        get_file_name_hash("my_config_file", 8),
    )

    print("\nCase conversion:")
    print("camel_to_snake('MyROSAPI'):", camel_to_snake("MyROSAPI"))
    print(
        "camel_to_snake('GenericActionClient'):", camel_to_snake("GenericActionClient")
    )
    print("camel_to_snake('BAMNode'):", camel_to_snake("BAMNode"))

    print("\nPath utilities:")
    print(
        "node_name_from_file('/path/to/my_node.py'):",
        node_name_from_file("/path/to/my_node.py"),
    )
    print(
        "node_name_from_file('/path/to/test_node.py'):",
        node_name_from_file("/path/to/test_node.py"),
    )

    print("\nType utilities:")

    # Test with a mock class
    class MockClass:
        pass

    mock_instance = MockClass()
    print("instance_to_snake(mock_instance):", instance_to_snake(mock_instance))
    print("type_to_name(MockClass):", type_to_name(MockClass))
