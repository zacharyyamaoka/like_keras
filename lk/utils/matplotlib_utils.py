#!/usr/bin/env python3

"""
    Convenience helpers for plotting RGB-D + mask tensors.
"""

# PYTHON
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


def plot_rgbd_mask(color: np.ndarray, depth: np.ndarray, mask: np.ndarray) -> None:
    """Display RGB, depth, mask, and overlay views."""
    fig = plt.figure(figsize=(10, 10))

    ax_color = fig.add_subplot(2, 2, 1)
    ax_color.imshow(color)
    ax_color.set_title("Color Image")
    ax_color.axis("off")

    ax_depth = fig.add_subplot(2, 2, 2)
    depth_im = ax_depth.imshow(depth, cmap="plasma")
    ax_depth.set_title("Depth Image")
    ax_depth.axis("off")
    fig.colorbar(depth_im, ax=ax_depth, fraction=0.046, pad=0.04)

    ax_mask = fig.add_subplot(2, 2, 3)
    ax_mask.imshow(mask, cmap="gray")
    ax_mask.set_title("Mask Image")
    ax_mask.axis("off")

    ax_overlay = fig.add_subplot(2, 2, 4)
    ax_overlay.imshow(color)

    unique_vals = np.unique(mask)
    if len(unique_vals) > 2:
        norm_mask = (mask.astype(float) - mask.min()) / (mask.max() - mask.min() + 1e-8)
        cmap = plt.get_cmap("jet")
        mask_rgb = cmap(norm_mask)[..., :3]
        alpha = (mask > 0).astype(float) * 0.5
        ax_overlay.imshow(mask_rgb, alpha=alpha)
    else:
        mask_overlay = mask > 0
        red = np.zeros_like(color, dtype=np.float32)
        red[..., 0] = 1.0
        overlay = np.where(mask_overlay[..., None], red, 0)
        ax_overlay.imshow(overlay, alpha=0.4)

    ax_overlay.set_title("Color + Mask Overlay")
    ax_overlay.axis("off")

    plt.tight_layout()
    plt.show()


__all__ = ["plot_rgbd_mask"]

