#!/usr/bin/env python3

"""
DiagnosticTaskList Base Class

Base class for component diagnostic task containers.
Provides automatic serialization, deserialization, and task name prefixing.
"""

# PYTHON
from typing import get_type_hints


class DiagnosticTaskList:
    """
    Base class for component diagnostic task containers.

    Provides automatic serialization, deserialization, and task name prefixing
    for all diagnostic tasks defined as dataclass fields.

    Usage:
        @dataclass
        class EnvDiagnostics(DiagnosticTaskList):
            step_latency: BoxChartTask = field(...)
            reset_latency: BoxChartTask = field(...)

    The class automatically provides:
    - __post_init__(): Sets task names from field names
    - to_config(): Serialize all tasks to dict
    - from_config(): Deserialize all tasks from dict
    - prefix_task_names(): Add component_id prefix to all task names
    """

    def __post_init__(self) -> None:
        """
        Automatically set task names from field names.
        This is called after dataclass __init__ to populate task names.
        """
        for field_name, field_value in self.__dict__.items():
            # Check if field is a diagnostic task (has name attribute)
            if hasattr(field_value, "name"):
                # Set the name from the field name if not already set explicitly
                if field_value.name == "unnamed_task":
                    field_value.name = field_name

    def to_config(self) -> dict:
        """
        Convert all tasks to config dicts for serialization.

        Returns:
            Dictionary mapping field names to task configs
        """
        return {
            field_name: getattr(self, field_name).to_config()
            for field_name in self.__dataclass_fields__
        }

    @classmethod
    def from_config(cls, config: dict):
        """
        Reconstruct all tasks from config using type introspection.

        Args:
            config: Dictionary mapping field names to task configs

        Returns:
            Instance of the DiagnosticTaskList subclass with reconstructed tasks
        """
        type_hints = get_type_hints(cls)
        return cls(
            **{
                field_name: type_hints[field_name].from_config(
                    task_config, name=field_name
                )
                for field_name, task_config in config.items()
            }
        )

    def prefix_task_names(self, component_id: str):
        """
        Add component_id prefix to all task names.

        Args:
            component_id: Component identifier to use as prefix
        """
        for field_name in self.__dataclass_fields__:
            task = getattr(self, field_name)
            task.name = f"{component_id}/{field_name}"
