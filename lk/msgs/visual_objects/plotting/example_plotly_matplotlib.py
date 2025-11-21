#!/usr/bin/env python3

"""
Plotly and Matplotlib Viewer Examples

Demonstrates:
- Line charts (single and multiple series)
- Histograms (auto and custom binning)
- Using both PlotlyViewer and MatplotlibViewer
"""

if __name__ == "__main__":

    # BAM
    import bam.msgs.visual_objects as viz
    from bam_artist.viewers import PlotlyViewer, MatplotlibViewer
    from bam.utils import BinConfig

    # PYTHON
    import numpy as np

    print("=" * 70)
    print("Plotly and Matplotlib Viewer Examples")
    print("=" * 70)

    # Generate sample data
    np.random.seed(42)

    # Line chart data
    x = np.linspace(0, 2 * np.pi, 100)
    y_sin = np.sin(x)
    y_cos = np.cos(x)
    y_multi = np.column_stack([y_sin, y_cos, np.sin(2 * x)])

    # Histogram data
    normal_values = np.random.randn(1000)

    # Custom bin config
    bin_config = BinConfig(
        n_bins=10,
        min_value=normal_values.min(),
        max_value=normal_values.max(),
        first_edge=-2.0,
        last_edge=2.0,
    )

    # === Plotly Viewer Demo ===
    print("\n1. Creating Plotly Viewer...")
    print("-" * 70)

    plotly_viewer = PlotlyViewer(verbose=True)

    # Line chart
    line_chart = viz.LineChart(
        x=x,
        y=y_sin,
        x_label="Angle (rad)",
        y_label="sin(x)",
        title="Sine Wave (Plotly)",
        name="sine_plotly",
    )
    plotly_viewer.draw(line_chart)

    # Multi-series line chart
    multi_line = viz.LineChart(
        x=x,
        y=y_multi,
        labels=["sin(x)", "cos(x)", "sin(2x)"],
        x_label="Angle (rad)",
        y_label="Value",
        title="Multiple Series (Plotly)",
        name="multi_plotly",
    )
    plotly_viewer.draw(multi_line)

    # Histogram with auto binning
    hist_auto = viz.Histogram(
        values=normal_values,
        x_label="Value",
        y_label="Frequency",
        title="Normal Distribution - Auto Bins (Plotly)",
        name="hist_auto_plotly",
    )
    plotly_viewer.draw(hist_auto)

    # Histogram with custom BinConfig
    hist_custom = viz.Histogram(
        values=normal_values,
        bin_config=bin_config,
        x_label="Value",
        y_label="Frequency",
        title="Normal Distribution - Custom Bins (Plotly)",
        color="coral",
        name="hist_custom_plotly",
    )
    plotly_viewer.draw(hist_custom)

    print("Opening Plotly plots in browser...")
    plotly_viewer.render(None)

    # === Matplotlib Viewer Demo ===
    print("\n2. Creating Matplotlib Viewer...")
    print("-" * 70)

    matplotlib_viewer = MatplotlibViewer(verbose=True, interactive=False)

    # Line chart
    line_chart_mpl = viz.LineChart(
        x=x,
        y=y_sin,
        x_label="Angle (rad)",
        y_label="sin(x)",
        title="Sine Wave (Matplotlib)",
        name="sine_mpl",
    )
    matplotlib_viewer.draw(line_chart_mpl)

    # Multi-series line chart
    multi_line_mpl = viz.LineChart(
        x=x,
        y=y_multi,
        labels=["sin(x)", "cos(x)", "sin(2x)"],
        x_label="Angle (rad)",
        y_label="Value",
        title="Multiple Series (Matplotlib)",
        name="multi_mpl",
    )
    matplotlib_viewer.draw(multi_line_mpl)

    # Histogram with auto binning
    hist_auto_mpl = viz.Histogram(
        values=normal_values,
        x_label="Value",
        y_label="Frequency",
        title="Normal Distribution - Auto Bins (Matplotlib)",
        name="hist_auto_mpl",
    )
    matplotlib_viewer.draw(hist_auto_mpl)

    # Histogram with custom BinConfig
    hist_custom_mpl = viz.Histogram(
        values=normal_values,
        bin_config=bin_config,
        x_label="Value",
        y_label="Frequency",
        title="Normal Distribution - Custom Bins (Matplotlib)",
        color="coral",
        name="hist_custom_mpl",
    )
    matplotlib_viewer.draw(hist_custom_mpl)

    print("Opening Matplotlib plots...")
    matplotlib_viewer.render(None)

    print("\n" + "=" * 70)
    print("Summary:")
    print("=" * 70)
    print(f"  Plotly plots    : {len(plotly_viewer.figures)} figures opened in browser")
    print(f"  Matplotlib plots: {len(matplotlib_viewer.figures)} figures displayed")
    print("\nClose all matplotlib windows to exit.")
    print("=" * 70)
