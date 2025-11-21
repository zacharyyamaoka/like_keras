"""
Robot description metadata for assembled robot records.
"""

# BAM

# PYTHON
from dataclasses import dataclass, asdict
import hashlib
import json
import yaml
from typing import Optional

# PYTHON
import os


@dataclass
class RobotInfo:
    prefix: Optional[str] = None
    name: str = ""
    type: str = ""
    sku: str = ""
    version: str = "0.0.0"
    config_key: str = ""
    uuid: str = ""
    urdf_hash: Optional[str] = None
    save_dir: str = ""

    def get_prefix(self) -> str:
        return self.prefix if self.prefix else ""

    def add_prefix(self, prefix: str) -> None:
        """Add a prefix to the existing prefix, appending instead of overwriting.

        This allows calling prefix() multiple times to build up a prefix chain.
        If prefix is None, sets it to the new prefix. Otherwise, appends the new prefix.

        Args:
            prefix: The prefix string to add
        """
        if self.prefix is None:
            self.prefix = prefix
        else:
            self.prefix = self.prefix + prefix

    def get_info_file_name(self) -> str:
        """Get generic file name based on prefix, name, sku, and version."""
        return f"{self.get_prefix()}{self.name}_{self.sku}_v{self.version}"

    def get_hash_file_name(self) -> str:
        """Get file name based on hash."""
        if self.urdf_hash is None:
            raise ValueError("URDF hash is not set")
        return f"{self.get_prefix()}{self.name}_{self.urdf_hash}"

    def get_file_name(self) -> str:
        """Get file name, choosing between hash-based or info-based name.

        If urdf_hash is set, returns hash-based name. Otherwise returns info-based name.
        """
        if self.urdf_hash:
            return self.get_hash_file_name()
        else:
            return self.get_info_file_name()

    def get_file_path(self, file_extension: str, subdir: Optional[str] = None) -> str:
        """Get full file path with the specified extension.

        Args:
            file_extension: File extension (e.g., '.yaml', '.json', '.srdf')
                           Should include the leading dot.
            subdir: Optional subdirectory to add between save_dir and filename

        Returns:
            Full file path: save_dir / [subdir] / file_name + file_extension
        """
        if not self.save_dir:
            raise ValueError("save_dir is not set on RobotInfo")
        file_name = self.get_file_name()

        if subdir:
            target_dir = os.path.join(self.save_dir, subdir)
        else:
            target_dir = self.save_dir

        file_path = os.path.join(target_dir, file_name + file_extension)
        # Ensure the parent directory exists (not the file itself!)
        os.makedirs(target_dir, exist_ok=True)

        return file_path

    def __str__(self) -> str:
        """Return RobotInfo as YAML string."""
        data = asdict(self)
        # Filter out empty strings and None values for cleaner output
        # filtered_data = {k: v for k, v in data.items() if v not in (None, "")}
        return yaml.dump(data, default_flow_style=False, sort_keys=False).strip()


if __name__ == "__main__":
    robot = RobotInfo(name="example_robot")
    print(robot)
