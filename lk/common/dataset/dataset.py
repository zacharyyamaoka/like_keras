#!/usr/bin/env python3

"""
    Dataset abstraction for time-series data.
    
    Backend-agnostic dataset API that can use different storage:
    - Memory (temporary, fast)
    - Pickle (persistent, Python-only)
    - MCAP (ROS 2 standard, cross-language)
    - ReductStore (production time-series database)
"""

# BAM  
from .backend import DatasetBackend
from .memory_backend import MemoryBackend
from .pickle_backend import PickleBackend

# PYTHON
from typing import Any, Optional
from pathlib import Path
from abc import ABC


class Dataset:
    """
        High-level dataset interface.
        
        Provides unified API for recording and playing back time-series data.
        Storage backend is pluggable (memory, pickle, MCAP, etc.).
        
        Usage:
            # Create new dataset
            dataset = Dataset.create(backend='memory')
            dataset.write('sensor1', value, timestamp)
            
            # Load existing dataset
            dataset = Dataset.load('data.pkl')
            values = dataset.read('sensor1')
    """
    
    def __init__(self, backend: DatasetBackend):
        """
            Initialize dataset with a backend.
            
            Args:
                backend: Storage backend instance
        """
        self.backend = backend
        
        # Ensure backend is open
        if not backend.is_open:
            backend.open()
    
    def write(self, key: str, value: Any, timestamp: Optional[float] = None) -> None:
        """
            Write data point to dataset.
            
            Args:
                key: Data identifier (e.g., port name)
                value: Data to store
                timestamp: Optional timestamp
        """
        self.backend.write(key, value, timestamp)
    
    def read(self, key: str, index: Optional[int] = None) -> Any:
        """
            Read data from dataset.
            
            Args:
                key: Data identifier
                index: Optional index (None = all data)
                
            Returns:
                Single value if index provided, list otherwise
        """
        return self.backend.read(key, index)
    
    def keys(self) -> list[str]:
        """
            Get all data keys in the dataset.
            
            Returns:
                List of data identifiers
        """
        return self.backend.keys()
    
    def __len__(self) -> int:
        """
            Get number of time steps in dataset.
            
            Returns:
                Number of recorded time steps
        """
        return len(self.backend)
    
    def get_timestamps(self, key: Optional[str] = None) -> list[float]:
        """
            Get timestamps for time-series data.
            
            Args:
                key: Optional key to get timestamps for
                
            Returns:
                List of timestamps
        """
        return self.backend.get_timestamps(key)
    
    def flush(self) -> None:
        """Force write buffered data to storage."""
        self.backend.flush()
    
    def close(self) -> None:
        """Close dataset and release resources."""
        self.backend.close()
    
    def clear(self) -> None:
        """Clear all data from dataset."""
        self.backend.clear()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        return False
    
    def __repr__(self) -> str:
        keys = self.keys()
        n_keys = len(keys)
        n_samples = len(self)
        return f"Dataset(backend={self.backend.__class__.__name__}, keys={n_keys}, samples={n_samples})"
    
    def __getitem__(self, key: str) -> Any:
        """
            Dictionary-style access to data.
            
            Example:
                data = dataset['sensor1']
        """
        return self.read(key)
    
    def items(self):
        """
            Iterate over (key, data) pairs.
            
            Example:
                for key, data in dataset.items():
                    print(f"{key}: {len(data)} samples")
        """
        for key in self.keys():
            yield key, self.read(key)
    
    # Factory methods
    
    @classmethod
    def create(cls, backend: str = 'memory', path: Optional[Path] = None, **kwargs) -> 'Dataset':
        """
            Create new dataset with specified backend.
            
            Args:
                backend: Backend type ('memory', 'pickle', 'mcap', 'reductstore')
                path: Path for file-based backends
                **kwargs: Backend-specific options
                
            Returns:
                New Dataset instance
                
            Example:
                >>> dataset = Dataset.create(backend='memory')
                >>> dataset = Dataset.create(backend='pickle', path='data.pkl')
        """
        if backend == 'memory':
            backend_inst = MemoryBackend(**kwargs)
        
        elif backend == 'pickle':
            if path is None:
                raise ValueError("PickleBackend requires 'path' argument")
            backend_inst = PickleBackend(path=path, mode='write', **kwargs)
        
        elif backend == 'mcap':
            # Stub for future implementation
            raise NotImplementedError("MCAPBackend not yet implemented")
        
        elif backend == 'reductstore':
            # Stub for future implementation
            raise NotImplementedError("ReductStoreBackend not yet implemented")
        
        else:
            raise ValueError(f"Unknown backend: {backend}")
        
        backend_inst.open()
        return cls(backend=backend_inst)
    
    @classmethod
    def load(cls, path: Path, backend: Optional[str] = None, **kwargs) -> 'Dataset':
        """
            Load existing dataset from file.
            
            Args:
                path: Path to dataset file
                backend: Backend type (auto-detected from extension if None)
                **kwargs: Backend-specific options
                
            Returns:
                Loaded Dataset instance
                
            Example:
                >>> dataset = Dataset.load('data.pkl')
                >>> dataset = Dataset.load('data.mcap', backend='mcap')
        """
        path = Path(path)
        
        # Auto-detect backend from extension
        if backend is None:
            suffix = path.suffix.lower()
            if suffix == '.pkl' or suffix == '.pickle':
                backend = 'pickle'
            elif suffix == '.mcap':
                backend = 'mcap'
            else:
                raise ValueError(f"Cannot auto-detect backend for '{suffix}' files")
        
        if backend == 'pickle':
            backend_inst = PickleBackend(path=path, mode='read', **kwargs)
        
        elif backend == 'mcap':
            raise NotImplementedError("MCAPBackend not yet implemented")
        
        else:
            raise ValueError(f"Unknown backend: {backend}")
        
        backend_inst.open()
        return cls(backend=backend_inst)
    
    def to_dict(self) -> dict[str, Any]:
        """
            Convert dataset to dictionary format.
            
            Useful for inspection and serialization.
            
            Returns:
                Dict mapping keys to their data and timestamps
        """
        result = {}
        for key in self.keys():
            result[key] = {
                'data': self.read(key),
                'timestamps': self.get_timestamps(key)
            }
        return result
    
    @classmethod
    def from_dict(cls, data: dict[str, Any], backend: str = 'memory') -> 'Dataset':
        """
            Create dataset from dictionary.
            
            Args:
                data: Dictionary with structure {key: {'data': [...], 'timestamps': [...]}}
                backend: Backend type to use
                
            Returns:
                New Dataset instance
        """
        dataset = cls.create(backend=backend)
        
        for key, values in data.items():
            data_list = values['data']
            timestamps = values.get('timestamps', [None] * len(data_list))
            
            for value, timestamp in zip(data_list, timestamps):
                dataset.write(key, value, timestamp)
        
        return dataset


if __name__ == '__main__':
    
    print("\n" + "="*70)
    print("Dataset - Backend-Agnostic Time-Series Storage")
    print("="*70)
    
    # Example 1: Memory backend
    print("\nExample 1: Memory backend (temporary)")
    dataset = Dataset.create(backend='memory')
    dataset.write('position', [1.0, 2.0, 3.0], timestamp=0.0)
    dataset.write('position', [1.5, 2.5, 3.5], timestamp=0.1)
    dataset.write('velocity', [0.5, 0.5, 0.5], timestamp=0.0)
    
    print(f"  Keys: {dataset.keys()}")
    print(f"  Length: {len(dataset)}")
    print(f"  Position: {dataset['position']}")
    print(f"  Timestamps: {dataset.get_timestamps()}")
    
    # Example 2: Pickle backend
    print("\nExample 2: Pickle backend (persistent)")
    import tempfile
    with tempfile.NamedTemporaryFile(suffix='.pkl', delete=False) as tmp:
        tmp_path = Path(tmp.name)
    
    # Write
    with Dataset.create(backend='pickle', path=tmp_path) as ds:
        ds.write('sensor1', 42.0)
        ds.write('sensor1', 43.5)
        print(f"  Wrote dataset to {tmp_path}")
    
    # Read
    with Dataset.load(tmp_path) as ds:
        print(f"  Loaded dataset: {ds}")
        print(f"  Data: {ds['sensor1']}")
    
    # Cleanup
    tmp_path.unlink()
    
    # Example 3: Dictionary conversion
    print("\nExample 3: Dictionary conversion")
    data_dict = {
        'x': {'data': [1, 2, 3], 'timestamps': [0.0, 0.1, 0.2]},
        'y': {'data': [4, 5, 6], 'timestamps': [0.0, 0.1, 0.2]}
    }
    dataset = Dataset.from_dict(data_dict)
    print(f"  Created dataset from dict: {dataset}")
    print(f"  Back to dict: {dataset.to_dict()}")
    
    print("\n" + "="*70)
    print("Dataset provides uniform API across different storage backends")
    print("="*70 + "\n")


