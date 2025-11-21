#!/usr/bin/env python3

"""
Utility helpers for inspecting datasets and CUDA availability.
"""

# PYTHON
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import subprocess

try:
    import torch
    from torch.utils.data import DataLoader
    from torchvision.utils import make_grid

except ImportError as e:
    raise ImportError(
        "The 'torch' package is required but not installed. Please install it via 'pip install torch' and try again."
    ) from e


def cuda_health_check() -> None:
    """Print CUDA availability plus nvidia-smi summary."""
    print("torch:", torch.__version__)
    print("is_available:", torch.cuda.is_available())
    print("device_count:", torch.cuda.device_count())
    if torch.cuda.is_available():
        print("current_device:", torch.cuda.current_device())
        print("name:", torch.cuda.get_device_name(0))
        print("capability:", torch.cuda.get_device_capability(0))
        try:
            a_tensor = torch.randn(1, device="cuda")
            _ = a_tensor.cpu()
            print("basic CUDA op: OK")
            free, total = torch.cuda.mem_get_info()
            print("mem free/total (MB):", free // 2**20, "/", total // 2**20)
        except Exception as exc:
            print("basic CUDA op failed:", exc)

    try:
        output = subprocess.check_output(["nvidia-smi"]).decode().splitlines()[0]
        print(output)
    except Exception as exc:
        print("nvidia-smi failed:", exc)


def show_batch(data_loader: DataLoader) -> None:
    """Plot sample image and label tensors from a data loader."""
    for images, labels in data_loader:
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))

        axes[0].set_xticks([])
        axes[0].set_yticks([])
        axes[0].set_title("Images")
        img_grid = make_grid(images[:64], nrow=8, pad_value=1).permute(1, 2, 0)
        axes[0].imshow(img_grid)

        axes[1].set_xticks([])
        axes[1].set_yticks([])
        axes[1].set_title("Labels")
        label_grid = make_grid(labels[:64], nrow=8, pad_value=1).permute(1, 2, 0)
        axes[1].imshow(label_grid)

        plt.tight_layout()
        plt.show()
        break


def show_batch_histograms(data_loader: DataLoader, n_batches: int = 1) -> None:
    """Render RGB pixel histograms plus mask distribution for N batches."""
    for batch_idx, (images, masks) in enumerate(data_loader):
        img_vals = images.permute(1, 0, 2, 3).reshape(images.shape[1], -1)
        mask_vals = masks.reshape(-1)

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        colors = ["r", "g", "b"] if images.shape[1] == 3 else ["k"]
        for channel in range(images.shape[1]):
            axes[0].hist(
                img_vals[channel].numpy().ravel(),
                bins=50,
                color=colors[channel],
                alpha=0.5,
                label=f"Channel {channel}",
            )
        axes[0].set_title("Image Pixel Distribution")
        axes[0].legend()

        axes[1].hist(mask_vals.numpy().ravel(), bins=50, color="gray", alpha=0.7)
        axes[1].set_title("Mask Pixel Distribution")

        plt.show()

        if batch_idx + 1 >= n_batches:
            break


def numpy_to_tensor_pairs(
    numpy_pairs: list[tuple[np.ndarray, np.ndarray]],
) -> list[tuple[torch.Tensor, torch.Tensor]]:
    """Convert list of numpy (image, label) pairs to tensors."""
    tensor_pairs: list[tuple[torch.Tensor, torch.Tensor]] = []

    grayscale_label = numpy_pairs[0][1].ndim == 2

    for img, label in numpy_pairs:
        img_tensor = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0

        if grayscale_label:
            label_tensor = torch.from_numpy(label).unsqueeze(0).float()
        else:
            label_tensor = torch.from_numpy(label).permute(2, 0, 1).float()

        tensor_pairs.append((img_tensor, label_tensor))
    return tensor_pairs


def plot_numpy_pairs(numpy_pairs: list[tuple[np.ndarray, np.ndarray]]) -> None:
    """Plot image, label, and overlay for up to five samples."""
    n_plot = min(5, len(numpy_pairs))

    pairs_copy = numpy_pairs
    if numpy_pairs[0][1].ndim == 3:
        pairs_copy = []
        for img, label in numpy_pairs[:n_plot]:
            max_heatmap = np.max(label, axis=2)
            pairs_copy.append((img, max_heatmap))
        pairs_copy.extend(numpy_pairs[n_plot:])

    fig, axes = plt.subplots(n_plot, 3, figsize=(12, 3 * n_plot))
    axes = np.atleast_2d(axes)

    for idx in range(n_plot):
        img, label = pairs_copy[idx]

        img_rgb = img.astype(np.float32)
        if img_rgb.max() > 1.0:
            img_rgb /= 255.0

        label_norm = np.zeros_like(label, dtype=float)
        lmin, lmax = np.nanmin(label), np.nanmax(label)
        if lmax > lmin:
            label_norm = (label - lmin) / (lmax - lmin)
        heat_rgb = plt.get_cmap("jet")(label_norm)[..., :3]
        alpha = (idx + 1) / n_plot
        overlay = (1 - alpha) * img_rgb + alpha * heat_rgb

        axes[idx, 0].imshow(img_rgb)
        axes[idx, 0].set_title("Image")
        axes[idx, 1].imshow(heat_rgb)
        axes[idx, 1].set_title("Label")
        axes[idx, 2].imshow(overlay)
        axes[idx, 2].set_title(f"Overlay α={alpha:.2f}")

        for col in range(3):
            axes[idx, col].axis("off")

    plt.tight_layout()
    plt.show()


def env_to_tensor_pairs(
    env: Any,
    train_size: int,
    test_size: int,
    verbose: bool = True,
    show: bool = False,
) -> tuple[
    list[tuple[torch.Tensor, torch.Tensor]], list[tuple[torch.Tensor, torch.Tensor]]
]:
    """Create dataset tensors via an env that exposes create_dataset."""
    if not hasattr(env.unwrapped, "create_dataset"):
        raise AttributeError("Environment must have a create_dataset method")

    _ = env.unwrapped.create_dataset(1)

    np_train_pairs = env.unwrapped.create_dataset(train_size)
    np_test_pairs = env.unwrapped.create_dataset(test_size)

    if show:
        plot_numpy_pairs(np_train_pairs)

    train_data_pairs = numpy_to_tensor_pairs(np_train_pairs)
    test_data_pairs = numpy_to_tensor_pairs(np_test_pairs)

    if verbose:
        print("Size of train tensor dataset:", len(train_data_pairs))
        print("Shape of first image:", train_data_pairs[0][0].shape)
        print("Shape of first mask:", train_data_pairs[0][1].shape)
        print("Size of test tensor dataset:", len(test_data_pairs))
        print("Shape of first image:", test_data_pairs[0][0].shape)
        print("Shape of first mask:", test_data_pairs[0][1].shape)

    return train_data_pairs, test_data_pairs


__all__ = [
    "cuda_health_check",
    "show_batch",
    "show_batch_histograms",
    "numpy_to_tensor_pairs",
    "plot_numpy_pairs",
    "env_to_tensor_pairs",
]
