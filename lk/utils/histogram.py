#!/usr/bin/env python3

"""
Histogram helpers for score/value visualization.

Combines the previous math_utils histogram helpers with the plotting
utilities so everything lives under bam.utils.histogram.
"""

# PYTHON
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class BinConfig:
    """Configuration for binning scores before coloring."""

    first_edge: float | None = None
    last_edge: float | None = None
    n_bins: int = 5
    min_value: float | None = 0.0
    max_value: float | None = 1.0
    normalize: bool = True

    def __post_init__(self) -> None:
        self.n_bins = max(self.n_bins, 2)
        self._ensure_edge_spacing()

    def _ensure_edge_spacing(self) -> None:
        """Populate first/last edges when the bounds are known."""
        if self.min_value is None or self.max_value is None:
            return

        bin_width = (self.max_value - self.min_value) / self.n_bins
        even_space_first_edge = self.min_value + bin_width
        even_space_last_edge = self.max_value - bin_width

        if self.first_edge is None:
            self.first_edge = even_space_first_edge
        if self.last_edge is None:
            self.last_edge = even_space_last_edge

    def ensure_bounds(self, values: Iterable[float] | None = None) -> None:
        """Ensure numeric bounds exist, optionally inferring them from values."""
        if values is not None:
            array = np.asarray(list(values), dtype=float)
            if array.size:
                if self.min_value is None:
                    self.min_value = float(np.min(array))
                if self.max_value is None:
                    self.max_value = float(np.max(array))

        if self.min_value is None or self.max_value is None:
            raise ValueError("min_value and max_value must be set before binning")

        self._ensure_edge_spacing()
        if not (self.min_value < self.first_edge < self.max_value):
            raise ValueError("first_edge must be within [min_value, max_value]")
        if not (self.min_value < self.last_edge < self.max_value):
            raise ValueError("last_edge must be within [min_value, max_value]")
        if not (self.first_edge < self.last_edge):
            raise ValueError("first_edge must be less than last_edge")

    def edges(self) -> list[float]:
        """Compute bin edges spanning [min_value, max_value]."""
        self.ensure_bounds()

        n_inner = self.n_bins - 2
        edge_step = (self.last_edge - self.first_edge) / max(n_inner, 1)

        edges = [self.min_value]
        for idx in range(n_inner):
            edges.append(self.first_edge + idx * edge_step)
        edges.extend([self.last_edge, self.max_value])
        return edges


def bin_values(
    values: np.ndarray,
    first_edge: float = 0.2,
    last_edge: float = 0.99,
    n_bins: int = 5,
    min_value: float = 0.0,
    max_value: float = 1.0,
    normalize: bool = True,
) -> tuple[np.ndarray, list[float]]:
    """Loop-based binning with explicit first/last edge control."""
    config = BinConfig(
        first_edge=first_edge,
        last_edge=last_edge,
        n_bins=n_bins,
        min_value=min_value,
        max_value=max_value,
        normalize=normalize,
    )
    config.ensure_bounds(values)

    bin_edges = config.edges()
    bin_ids: list[float] = []
    for val in values:
        for edge_idx, edge in enumerate(bin_edges[1:]):
            if val <= edge:
                bin_ids.append(edge_idx + 1)
                break

    bin_array = np.asarray(bin_ids, dtype=float)
    if config.normalize:
        bin_array -= 1
        bin_array /= max(config.n_bins - 1, 1)

    return bin_array, bin_edges


def bin_values_vec(values: Iterable[float], config: BinConfig) -> np.ndarray:
    """Vectorized binning using numpy digitize, returns normalized ids."""
    array = np.asarray(list(values), dtype=float)
    if array.size == 0:
        return array

    config.ensure_bounds(array)
    bin_edges = config.edges()

    bin_ids = np.digitize(array, bin_edges) - 1
    bin_ids = np.clip(bin_ids, 0, config.n_bins - 1)

    if config.normalize:
        bin_ids = bin_ids.astype(float) / max(config.n_bins - 1, 1)

    return bin_ids


def plot_bins(
    values: np.ndarray,
    bin_edges: Sequence[float],
    x_label: str = "Values",
    fig: Optional[plt.Figure] = None,
    axes: Optional[np.ndarray] = None,
    show: bool = True,
) -> tuple[plt.Figure, np.ndarray]:
    """Plot raw and binned histograms."""
    if fig is None or axes is None:
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    raw_counts, raw_bins, _ = axes[0].hist(
        values, bins=20, color="gray", edgecolor="black"
    )
    axes[0].set_title("Raw Values")
    axes[0].set_xlabel(x_label)
    axes[0].set_ylabel("Count")
    axes[0].set_xticks(raw_bins)
    axes[0].set_xticklabels(
        [f"{edge:.2f}" for edge in raw_bins], rotation=45, ha="right"
    )

    for x_val, count in zip(raw_bins[:-1], raw_counts):
        if count > 0:
            axes[0].text(
                x_val + (raw_bins[1] - raw_bins[0]) / 2,
                count,
                str(int(count)),
                ha="center",
                va="bottom",
                fontsize=8,
            )

    hist_counts, _ = np.histogram(values, bins=bin_edges)
    bar_positions = np.arange(len(hist_counts))
    bar_labels = [
        f"{bin_edges[idx]:.2f} – {bin_edges[idx + 1]:.2f}"
        for idx in range(len(hist_counts))
    ]
    bars = axes[1].bar(
        bar_positions, hist_counts, width=0.8, color="steelblue", edgecolor="black"
    )

    axes[1].set_xticks(bar_positions)
    axes[1].set_xticklabels(bar_labels, rotation=45, ha="right")
    axes[1].set_title("Binned Values")
    axes[1].set_xlabel(x_label)
    axes[1].set_ylabel("Count")

    for bar in bars:
        height = bar.get_height()
        axes[1].text(
            bar.get_x() + bar.get_width() / 2,
            height,
            str(int(height)),
            ha="center",
            va="bottom",
            fontsize=8,
        )

    plt.tight_layout()

    if show:
        plt.show()

    return fig, axes


def main() -> None:
    """Lightweight manual test when run as a script."""
    sample_values = [0.1, 0.5, 0.9]
    config = BinConfig(n_bins=4)
    config.ensure_bounds(sample_values)
    edges = config.edges()
    print(f"Edges: {edges}")
    print(f"Bins : {bin_values_vec(sample_values, config)}")
    plot_bins(np.asarray(sample_values), edges, show=False)


__all__ = ["BinConfig", "bin_values", "bin_values_vec", "plot_bins"]


if __name__ == "__main__":
    main()
