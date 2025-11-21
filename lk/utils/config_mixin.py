#!/usr/bin/env python3

"""
ConfigMixin - Base class for config-like data structures with file I/O.
"""

# BAM
from .str_utils import instance_to_snake, get_file_name_timestamp
from .file_utils import remove_old_files, get_latest_file
from .dot_dict_utils import dict_to_dot_dict, dot_dict_to_dict

# PYTHON
from dataclasses import dataclass, field, asdict, is_dataclass, fields
from typing import Any, Dict
import copy
import json
import os
import yaml
from dacite import (
    from_dict,
)  # convert nested dataclasses to dicts properly, standard asdict() doesn't work

"""
ConfigMixin is a base class for config like data structures (RobotDescription, LaunchArgs, etc.)

- It adds a standard metadata field
- Adds methods to read/write to files

- Ensure all values are basic python so easy to save/load, no special classes, numpy, etc.
"""


# Useful for generating launch args, and replacing type NodeParams with their file paths

# Instead of saving a complex object, you can save a config file that holds all the information...


def replace_types_with_file_paths(
    dataclass_obj: Any,
    target_types: tuple[type, ...],
    get_file_path_method: str = "save_yaml_file",
) -> Any:
    """
    Recursively replace instances of target_types with their file paths.

    DescriptionArgs:
        obj: The dataclass object to process
        target_types: Tuple of types to match and replace
        get_file_path_method: Name of method to call to get file path

    Returns:
        A new dataclass instance with replacements made
    """
    if not is_dataclass(dataclass_obj):
        return dataclass_obj

    # Create a deep copy so we don't modify the original
    modified_obj = copy.deepcopy(dataclass_obj)

    # Walk through the dataclass structure and replace target instances
    _walk_and_replace(modified_obj, target_types, get_file_path_method)

    return modified_obj


def _walk_and_replace(
    dataclass_obj: Any, target_types: tuple[type, ...], get_file_path_method: str
) -> None:
    if not is_dataclass(dataclass_obj):
        return

    for field in fields(dataclass_obj):
        field_value = getattr(dataclass_obj, field.name)

        # Check if this field is an instance of one of our target types
        if isinstance(field_value, target_types):
            # First recursively process any nested dataclasses within this instance
            _walk_and_replace(field_value, target_types, get_file_path_method)

            # Now call the get_file_path method to save and get path
            get_path_func = getattr(field_value, get_file_path_method)
            file_path = get_path_func()

            # Replace the object with the file path string
            setattr(dataclass_obj, field.name, file_path)

        elif is_dataclass(field_value):
            # Recursively process nested dataclasses that aren't target types
            _walk_and_replace(field_value, target_types, get_file_path_method)


@dataclass
class ConfigMixin:
    _metadata: Dict[str, Any] = field(
        default_factory=lambda: {
            "version": "1.0",
            "created_by": os.popen("git config --get user.name").read().strip()
            or "unknown",
            "created_at": get_file_name_timestamp(),
        }
    )

    # use _ to note private, so when turning to launch args for example, we don't include them!
    _save_dir: str = ""
    _file_name: str = ""
    _file_name_stamped: str = ""

    def __post_init__(self):
        pass

        if self._save_dir == "":

            if "CONFIG_DUMP_DIR" in os.environ:
                self._save_dir = os.environ["CONFIG_DUMP_DIR"]
            else:
                self._save_dir = os.path.join(os.path.dirname(__file__), "config_dumps")
                os.makedirs(self._save_dir, exist_ok=True)

        if self._file_name == "":
            self._file_name = instance_to_snake(self)
            self._file_name_stamped = self._file_name + "_" + get_file_name_timestamp()

    def __str__(self):
        return json.dumps(asdict(self), indent=2)

    @classmethod
    def from_dot_dict(cls, dot_dict: dict):
        return from_dict(data_class=cls, data=dot_dict_to_dict(dot_dict))

    def to_dot_dict(self):
        return dict_to_dot_dict(self.to_json())

    def replace_types_with_file_paths(
        self,
        target_types: tuple[type, ...],
        get_file_path_method: str = "save_yaml_file",
    ) -> Any:
        return replace_types_with_file_paths(self, target_types, get_file_path_method)

    def to_json(self):
        data = asdict(self)
        return data

    def save_json_file(self, file_path="", verbose=True) -> str:

        if not file_path:
            file_path = os.path.join(self._save_dir, self._file_name_stamped + ".json")
            remove_old_files(self._save_dir, days_old=7)

        else:
            save_dir = os.path.dirname(file_path)  # assume that dir exists already
            remove_old_files(save_dir, days_old=7)

        data = self.to_json()
        with open(file_path, "w") as f:
            json.dump(data, f, indent=4)

        if verbose:
            print(f"Saved {file_path}")

        return file_path

    def save_yaml_file(self, file_path="", data=None, verbose=True) -> str:

        if not file_path:
            file_path = os.path.join(self._save_dir, self._file_name_stamped + ".yaml")
            remove_old_files(self._save_dir, days_old=7)

        else:
            save_dir = os.path.dirname(file_path)  # assume that dir exists already
            remove_old_files(save_dir, days_old=7)

        if data is None:
            data = self.to_json()

        with open(file_path, "w") as f:
            yaml.dump(data, f, indent=4)

        if verbose:
            print(f"Saved {file_path}")

        return file_path

    def set_env_var_path(self, file_path: str) -> str:
        ENV_VAR = instance_to_snake(self).upper()
        os.environ[ENV_VAR] = file_path

        return ENV_VAR

    @classmethod
    def from_json_file(cls, file_path="", verbose=False):

        if not file_path:
            temp_cls = (
                cls()
            )  # create a temporary instance to get the _save_dir and _file_name
            # BUG: don't match on exact file name with date
            # file_path = get_latest_file(temp_cls.save_dir, temp_cls.file_name + ".json" , verbose=verbose)
            file_path = get_latest_file(
                temp_cls._save_dir,
                temp_cls._file_name,
                file_ending=".json",
                verbose=verbose,
            )

        with open(file_path, "r") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("ConfigMixin file must be a JSON object at the top level.")
        return from_dict(
            data_class=cls, data=data
        )  # dataclass takes care of filling in fields

    @classmethod
    def from_yaml_file(cls, file_path="", verbose=False):

        if not file_path:
            temp_cls = (
                cls()
            )  # create a temporary instance to get the _save_dir and _file_name
            file_path = get_latest_file(
                temp_cls._save_dir,
                temp_cls._file_name,
                file_ending=".yaml",
                verbose=verbose,
            )

        with open(file_path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        if not isinstance(data, dict):
            raise ValueError("ConfigMixin file must be a YAML object at the top level.")
        # return cls(**data)  # dataclass takes care of filling in fields, doesn't work for nested dataclasses
        return from_dict(
            data_class=cls, data=data
        )  # Use dacite, instead which handles nested dataclasses/lists/etc.
