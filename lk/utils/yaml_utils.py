#!/usr/bin/env python3

"""
    Helper for applying lightweight modifications to YAML files.
"""

# PYTHON
from pathlib import Path
from typing import Any

import yaml


def load_and_edit_save_yaml(
    yaml_path: str,
    changes: dict[str, Any],
    save_path: str | None = None,
    verbose: bool = False,
) -> str:
    """Load a YAML file, apply updates, and persist to disk."""
    path = Path(yaml_path).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"YAML file not found: {path}")

    with path.open("r", encoding="utf-8") as stream:
        yaml_data = yaml.safe_load(stream) or {}

    yaml_data.update(changes)

    if save_path is None:
        save_path = str(path.with_stem(f"{path.stem}_edited"))

    save_location = Path(save_path).expanduser()
    save_location.parent.mkdir(parents=True, exist_ok=True)

    with save_location.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(yaml_data, stream, default_flow_style=False, sort_keys=False)

    if verbose:
        print(f"Saving to {save_location}:")
        for key, value in yaml_data.items():
            print(f"    {key}: {value}")

    return str(save_location)


__all__ = ["load_and_edit_save_yaml"]

