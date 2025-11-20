"""
    Dataset infrastructure for time-series data recording and playback.
    
    Provides components and utilities for AI-compile and NN-compile workflows:
    - Dataset: Backend-agnostic storage (memory, pickle, MCAP, etc.)
    - DataRecorder: Component for recording data
    - DataPlayback: Component for replaying data
    - diffdiff: Compare datasets for verification
    - Visualization utilities (stubs)
"""

from .dataset import Dataset
from .recorder import DataRecorder
from .playback import DataPlayback
from .diff import diffdiff, assert_datasets_equal, DiffResult, KeyDiff
from .viz import plot_histogram, plot_heatmap, plot_timeseries, plot_comparison

# Backend imports (for advanced users)
from .backend import DatasetBackend
from .memory_backend import MemoryBackend
from .pickle_backend import PickleBackend

__all__ = [
    # Core classes
    'Dataset',
    'DataRecorder',
    'DataPlayback',
    
    # Comparison utilities
    'diffdiff',
    'assert_datasets_equal',
    'DiffResult',
    'KeyDiff',
    
    # Visualization (stubs)
    'plot_histogram',
    'plot_heatmap',
    'plot_timeseries',
    'plot_comparison',
    
    # Backends (advanced)
    'DatasetBackend',
    'MemoryBackend',
    'PickleBackend',
]


