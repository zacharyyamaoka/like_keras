#!/usr/bin/env python3

"""
    JSON serialization helpers for dataclasses and nested containers.
"""

# PYTHON
from collections.abc import Iterable, Mapping
from dataclasses import asdict, is_dataclass
from typing import Any

import numpy as np


def to_jsonable(obj: Any, array_as_shape: bool = False) -> Any:
    """Recursively convert dataclasses, numpy objects, and containers to JSON."""
    if is_dataclass(obj):
        return to_jsonable(asdict(obj), array_as_shape=array_as_shape)

    if isinstance(obj, Mapping):
        return {key: to_jsonable(value, array_as_shape=array_as_shape) for key, value in obj.items()}

    if isinstance(obj, (list, tuple, set)):
        return [to_jsonable(value, array_as_shape=array_as_shape) for value in obj]

    if isinstance(obj, np.ndarray):
        if array_as_shape:
            return f"np.ndarray[{obj.shape}, {obj.dtype}]"
        return obj.tolist()

    if isinstance(obj, np.integer):
        return int(obj)

    if isinstance(obj, np.floating):
        return float(obj)

    if hasattr(obj, "__dict__"):
        try:
            return {
                key: to_jsonable(value, array_as_shape=array_as_shape)
                for key, value in obj.__dict__.items()
            }
        except Exception:
            return str(obj)

    return obj


__all__ = ["to_jsonable"]


if __name__ == "__main__":
    sample = {"demo": 1}
    print(to_jsonable(sample))

