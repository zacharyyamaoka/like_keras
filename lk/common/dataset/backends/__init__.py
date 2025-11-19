"""
    Dataset storage backends.
    
    Provides pluggable storage implementations for datasets.
"""

from .base import DatasetBackend
from .memory import MemoryBackend
from .pickle_backend import PickleBackend

