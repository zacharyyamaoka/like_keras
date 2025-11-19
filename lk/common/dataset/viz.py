#!/usr/bin/env python3

"""
    Visualization utilities for datasets.
    
    Provides functions to visualize and analyze recorded data:
    - Histograms of value distributions
    - Heatmaps showing differences
    - Time-series plots
    
    Note: These are stubs for now. Full implementations will be
    added as visualization needs become clearer.
"""

# BAM
from lk.common.dataset.dataset import Dataset

# PYTHON
from typing import Optional, Any
import warnings


def plot_histogram(dataset: Dataset, 
                  key: str,
                  bins: int = 50,
                  title: Optional[str] = None,
                  show: bool = True) -> None:
    """
        Plot histogram of values for a given key.
        
        Useful for visualizing data distributions from recorded datasets.
        
        Args:
            dataset: Dataset to visualize
            key: Key to plot histogram for
            bins: Number of histogram bins
            title: Optional plot title
            show: Whether to show plot immediately
            
        Note: This is a stub. Full implementation requires matplotlib.
        
        Example:
            >>> dataset = Dataset.load('training_data.pkl')
            >>> plot_histogram(dataset, 'sensor_readings', bins=100)
    """
    warnings.warn("plot_histogram is a stub. Full implementation coming soon.")
    
    print(f"\n[Stub] plot_histogram()")
    print(f"  Dataset: {dataset}")
    print(f"  Key: {key}")
    print(f"  Bins: {bins}")
    print(f"  Title: {title or 'Histogram'}")
    
    # TODO: Implement using matplotlib
    # data = dataset.read(key)
    # Extract numeric values from Msg objects
    # plt.hist(values, bins=bins)
    # plt.title(title or f'Distribution of {key}')
    # plt.xlabel('Value')
    # plt.ylabel('Frequency')
    # if show:
    #     plt.show()


def plot_heatmap(dataset1: Dataset,
                dataset2: Dataset,
                key: str,
                title: Optional[str] = None,
                show: bool = True) -> None:
    """
        Plot heatmap showing differences between two datasets.
        
        Visualizes where and how much two datasets differ.
        Useful for understanding discrepancies in implementations.
        
        Args:
            dataset1: First dataset (reference)
            dataset2: Second dataset (comparison)
            key: Key to compare
            title: Optional plot title
            show: Whether to show plot immediately
            
        Note: This is a stub. Full implementation requires matplotlib.
        
        Example:
            >>> ref_data = Dataset.load('original.pkl')
            >>> new_data = Dataset.load('compiled.pkl')
            >>> plot_heatmap(ref_data, new_data, 'outputs')
    """
    warnings.warn("plot_heatmap is a stub. Full implementation coming soon.")
    
    print(f"\n[Stub] plot_heatmap()")
    print(f"  Dataset 1: {dataset1}")
    print(f"  Dataset 2: {dataset2}")
    print(f"  Key: {key}")
    print(f"  Title: {title or 'Difference Heatmap'}")
    
    # TODO: Implement using matplotlib/seaborn
    # data1 = dataset1.read(key)
    # data2 = dataset2.read(key)
    # Compute differences
    # plt.imshow(differences, cmap='RdYlGn', aspect='auto')
    # plt.colorbar(label='Difference')
    # plt.title(title or f'Differences in {key}')
    # if show:
    #     plt.show()


def plot_timeseries(dataset: Dataset,
                   keys: Optional[list[str]] = None,
                   title: Optional[str] = None,
                   show: bool = True) -> None:
    """
        Plot time-series data from dataset.
        
        Shows how values evolve over time.
        
        Args:
            dataset: Dataset to visualize
            keys: Keys to plot (None = all keys)
            title: Optional plot title
            show: Whether to show plot immediately
            
        Note: This is a stub. Full implementation requires matplotlib.
        
        Example:
            >>> dataset = Dataset.load('robot_trajectory.pkl')
            >>> plot_timeseries(dataset, keys=['position', 'velocity'])
    """
    warnings.warn("plot_timeseries is a stub. Full implementation coming soon.")
    
    if keys is None:
        keys = dataset.keys()
    
    print(f"\n[Stub] plot_timeseries()")
    print(f"  Dataset: {dataset}")
    print(f"  Keys: {keys}")
    print(f"  Title: {title or 'Time Series'}")
    
    # TODO: Implement using matplotlib
    # for key in keys:
    #     data = dataset.read(key)
    #     timestamps = dataset.get_timestamps(key)
    #     plt.plot(timestamps, data, label=key)
    # plt.xlabel('Time (s)')
    # plt.ylabel('Value')
    # plt.title(title or 'Time Series Plot')
    # plt.legend()
    # if show:
    #     plt.show()


def plot_comparison(dataset1: Dataset,
                   dataset2: Dataset,
                   key: str,
                   labels: tuple[str, str] = ('Dataset 1', 'Dataset 2'),
                   title: Optional[str] = None,
                   show: bool = True) -> None:
    """
        Plot overlay comparison of two datasets.
        
        Shows both datasets on same plot for visual comparison.
        
        Args:
            dataset1: First dataset
            dataset2: Second dataset
            key: Key to compare
            labels: Labels for the two datasets
            title: Optional plot title
            show: Whether to show plot immediately
            
        Note: This is a stub. Full implementation requires matplotlib.
        
        Example:
            >>> original = Dataset.load('python_impl.pkl')
            >>> compiled = Dataset.load('cpp_impl.pkl')
            >>> plot_comparison(original, compiled, 'output',
            ...                 labels=('Python', 'C++'))
    """
    warnings.warn("plot_comparison is a stub. Full implementation coming soon.")
    
    print(f"\n[Stub] plot_comparison()")
    print(f"  Dataset 1: {dataset1} ({labels[0]})")
    print(f"  Dataset 2: {dataset2} ({labels[1]})")
    print(f"  Key: {key}")
    print(f"  Title: {title or f'Comparison of {key}'}")
    
    # TODO: Implement using matplotlib
    # data1 = dataset1.read(key)
    # data2 = dataset2.read(key)
    # ts1 = dataset1.get_timestamps(key)
    # ts2 = dataset2.get_timestamps(key)
    # plt.plot(ts1, data1, label=labels[0], alpha=0.7)
    # plt.plot(ts2, data2, label=labels[1], alpha=0.7)
    # plt.xlabel('Time (s)')
    # plt.ylabel('Value')
    # plt.title(title or f'Comparison: {key}')
    # plt.legend()
    # if show:
    #     plt.show()


if __name__ == '__main__':
    
    print("\n" + "="*70)
    print("Dataset Visualization Utilities (Stubs)")
    print("="*70)
    
    print("\nThese are stub implementations. Future versions will use matplotlib")
    print("to provide rich visualizations of dataset contents and comparisons.")
    
    # Create sample dataset
    dataset = Dataset.create(backend='memory')
    from lk.msgs.msg import Action
    
    for i in range(10):
        dataset.write('data', Action(data=[i, i*2]), timestamp=i*0.1)
    
    # Demo stubs
    print("\nDemonstrating stub functions:")
    
    plot_histogram(dataset, 'data', bins=20)
    
    dataset2 = Dataset.create(backend='memory')
    for i in range(10):
        dataset2.write('data', Action(data=[i+0.1, i*2+0.2]), timestamp=i*0.1)
    
    plot_heatmap(dataset, dataset2, 'data')
    plot_timeseries(dataset, keys=['data'])
    plot_comparison(dataset, dataset2, 'data')
    
    print("\n" + "="*70)
    print("Full implementations will be added based on visualization needs")
    print("="*70 + "\n")


