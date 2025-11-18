#!/usr/bin/env python3

"""
    Lightweight Matplotlib helpers for notebook visualizations.
"""

# PYTHON
from typing import Iterable

import cv2
import matplotlib.pyplot as plt
import numpy as np


def display_image(img: np.ndarray, scale: float = 2.0, title: str | None = None) -> None:
    """Display an image using Matplotlib with optional scaling."""
    output = img
    if img.ndim == 3:
        output = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif img.ndim != 2:
        raise ValueError("Image must be grayscale or RGB.")

    fig = plt.gcf()
    fig.set_size_inches(fig.get_size_inches() * scale)
    plt.imshow(output, cmap=None if img.ndim == 3 else "gray")
    plt.axis("off")
    if title:
        plt.title(title)
    plt.show()


def show_heatmap(
    rgb_image: np.ndarray,
    heatmap_slice: np.ndarray,
    alpha: float = 0.5,
    title: str = "Overlay",
) -> None:
    """Overlay a heatmap slice on top of an RGB image."""
    if rgb_image.shape[:2] != heatmap_slice.shape:
        raise ValueError("Heatmap and RGB image must match in spatial dimensions.")

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.imshow(rgb_image)
    overlay = ax.imshow(heatmap_slice, cmap="hot", alpha=alpha)
    ax.set_title(title)
    ax.axis("off")

    cbar = fig.colorbar(overlay, ax=ax)
    cbar.set_label("Intensity")
    plt.tight_layout()
    plt.show()


def show_heatmap_grid(
    rgb_image: np.ndarray,
    heatmaps: Iterable[np.ndarray],
    alpha: float = 0.5,
    cols: int = 3,
) -> None:
    """Display multiple heatmap overlays in a grid."""
    heatmaps_list = list(heatmaps)
    if not heatmaps_list:
        raise ValueError("heatmaps must contain at least one slice.")

    rows = int(np.ceil(len(heatmaps_list) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
    axes_array = np.array(axes).reshape(rows, cols)

    for idx, heatmap in enumerate(heatmaps_list):
        row = idx // cols
        col = idx % cols
        ax = axes_array[row, col]
        ax.imshow(rgb_image)
        ax.imshow(heatmap, cmap="hot", alpha=alpha)
        ax.set_title(f"Heatmap {idx}")
        ax.axis("off")

    for idx in range(len(heatmaps_list), rows * cols):
        row = idx // cols
        col = idx % cols
        axes_array[row, col].axis("off")

    plt.tight_layout()
    plt.show()


__all__ = ["display_image", "show_heatmap", "show_heatmap_grid"]

