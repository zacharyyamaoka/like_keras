#!/usr/bin/env python3

"""
Parametric 2D path generators plus derivative helpers.
"""

# PYTHON
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np


def snake_path(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    step: float = 0.01,
    n_row: int = 3,
    axis: str = "x",
) -> list[list[float]]:
    """Generate lawnmower-style path covering axis-aligned rectangle."""
    path: list[list[float]] = []
    x_forward = np.arange(x_min, x_max + step, step)
    x_backwards = np.arange(x_max, x_min - step, -step)
    row_step = (y_max - y_min) / max(n_row - 1, 1)
    y_row = np.arange(y_max, y_min - 1e-3, -row_step)

    for idx, y_val in enumerate(y_row):
        xs = x_forward if idx % 2 == 0 else x_backwards
        for x_val in xs:
            path.append([x_val, y_val])

        if idx != n_row - 1:
            for y_between in np.arange(y_val - step, y_val - row_step - 1e-3, -step):
                path.append([xs[-1], y_between])

    if axis == "y":
        return [[point[1], point[0]] for point in path]
    return path


def ellipse_path(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    step: float = 0.1,
) -> list[list[float]]:
    """Generate points tracing an ellipse within bounding box."""
    a = (x_max - x_min) / 2
    b = (y_max - y_min) / 2
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2

    t_vals = np.arange(0, 2 * np.pi, step)
    return [[x_center + a * np.cos(t), y_center + b * np.sin(t)] for t in t_vals]


def round_trip(path: Iterable[Iterable[float]]) -> list[list[float]]:
    """Go to the end of a path and return along the same points."""
    path_list = [list(point) for point in path]
    return path_list + list(reversed(path_list))


def plot_path(path: Iterable[Iterable[float]]) -> None:
    """Plot ordered list of [x, y] points."""
    pts = np.array(list(path))
    plt.figure(figsize=(6, 6))
    plt.plot(pts[:, 0], pts[:, 1], "o-", markersize=0.01)
    for idx, (x_val, y_val) in enumerate(pts):
        plt.annotate(
            str(idx),
            (x_val, y_val),
            textcoords="offset points",
            xytext=(2, 2),
            fontsize=6,
        )

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("2D Path")
    plt.grid(True)
    plt.show()


def centered_differences(
    path: Iterable[Iterable[float]], dt: float = 0.1
) -> np.ndarray:
    """Compute centered finite difference derivative for path samples."""
    path_arr = np.asarray(list(path), dtype=float)
    count, _ = path_arr.shape
    diffs = np.zeros_like(path_arr)

    for idx in range(count):
        if 0 < idx < count - 1:
            diffs[idx] = (path_arr[idx + 1] - path_arr[idx - 1]) / (2 * dt)
        elif idx == 0:
            diffs[idx] = (path_arr[idx + 1] - path_arr[idx]) / dt
        else:
            diffs[idx] = (path_arr[idx] - path_arr[idx - 1]) / dt

    return diffs


__all__ = [
    "snake_path",
    "ellipse_path",
    "round_trip",
    "plot_path",
    "centered_differences",
]
