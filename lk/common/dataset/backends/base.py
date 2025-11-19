#!/usr/bin/env python3

"""
    Base backend interface for dataset storage.
    
    Provides abstract interface that all storage backends must implement.
    Enables pluggable storage: memory, pickle, MCAP, ReductStore, etc.
"""

# PYTHON
from abc import ABC, abstractmethod
from typing import Any, Optional
from pathlib import Path


class DatasetBackend(ABC):
    """
        Abstract base class for dataset storage backends.
        
        All backends (memory, pickle, MCAP, etc.) must implement this interface.
        Provides uniform API for reading/writing time-series data.
    """
    
    def __init__(self, path: Optional[Path] = None, **kwargs):
        """
            Initialize backend.
            
            Args:
                path: Optional storage path (for file-based backends)
                **kwargs: Backend-specific configuration
        """
        self.path = path
        self._is_open = False
    
    @abstractmethod
    def write(self, key: str, value: Any, timestamp: Optional[float] = None) -> None:
        """
            Write a single data point to the backend.
            
            Args:
                key: Data identifier (e.g., port name)
                value: Data to store (typically a Msg instance)
                timestamp: Optional timestamp for time-series data
        """
        pass
    
    @abstractmethod
    def read(self, key: str, index: Optional[int] = None) -> Any:
        """
            Read data from the backend.
            
            Args:
                key: Data identifier
                index: Optional index for time-series data (None = all data)
                
            Returns:
                Single value if index provided, list of all values otherwise
        """
        pass
    
    @abstractmethod
    def keys(self) -> list[str]:
        """
            Get all keys in the dataset.
            
            Returns:
                List of all data identifiers
        """
        pass
    
    @abstractmethod
    def __len__(self) -> int:
        """
            Get number of data points in the dataset.
            
            For time-series data, returns number of time steps.
            
            Returns:
                Total number of recorded data points
        """
        pass
    
    @abstractmethod
    def get_timestamps(self, key: Optional[str] = None) -> list[float]:
        """
            Get timestamps for time-series data.
            
            Args:
                key: Optional key to get timestamps for specific data stream
                
            Returns:
                List of timestamps
        """
        pass
    
    @abstractmethod
    def open(self) -> None:
        """
            Open the backend for reading/writing.
            
            For file-based backends, this opens the file.
            For network backends, this establishes connection.
        """
        pass
    
    @abstractmethod
    def close(self) -> None:
        """
            Close the backend and flush any pending writes.
        """
        pass
    
    @abstractmethod
    def flush(self) -> None:
        """
            Flush any buffered writes to storage.
        """
        pass
    
    def clear(self) -> None:
        """
            Clear all data from the backend.
            
            Optional method - not all backends need to support this.
        """
        raise NotImplementedError(f"{self.__class__.__name__} does not support clear()")
    
    def __enter__(self):
        """Context manager entry."""
        self.open()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        return False
    
    @property
    def is_open(self) -> bool:
        """Check if backend is currently open."""
        return self._is_open
    
    def __repr__(self) -> str:
        status = "open" if self._is_open else "closed"
        path_str = f", path={self.path}" if self.path else ""
        return f"{self.__class__.__name__}({status}{path_str})"


if __name__ == '__main__':
    
    print("\n" + "="*70)
    print("DatasetBackend Base Class")
    print("="*70)
    
    print("\nDatasetBackend is an abstract base class for all storage backends.")
    print("It provides a uniform interface for reading/writing time-series data.")
    
    print("\n" + "Required methods:")
    print("  - write(key, value, timestamp) - Store data point")
    print("  - read(key, index=None) - Retrieve data")
    print("  - keys() - List all data identifiers")
    print("  - __len__() - Get number of data points")
    print("  - get_timestamps(key=None) - Get time-series timestamps")
    print("  - open() - Initialize backend")
    print("  - close() - Cleanup backend")
    print("  - flush() - Force write buffered data")
    
    print("\n" + "Optional methods:")
    print("  - clear() - Remove all data")
    
    print("\n" + "Backends to implement:")
    print("  - MemoryBackend - In-memory dict storage (fast, temporary)")
    print("  - PickleBackend - Python pickle files (simple, portable)")
    print("  - MCAPBackend - MCAP format (ROS 2 standard)")
    print("  - ReductStoreBackend - Time-series database (production)")
    
    print("\n" + "="*70 + "\n")

