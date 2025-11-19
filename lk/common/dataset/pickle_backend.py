#!/usr/bin/env python3

"""
    Pickle-based dataset backend.
    
    Simple file-based storage using Python's pickle.
    Portable and easy to use for Python-only workflows.
"""

# BAM
from .backend import DatasetBackend

# PYTHON
from typing import Any, Optional
from pathlib import Path
import pickle
import time


class PickleBackend(DatasetBackend):
    """
        File-based storage using Python pickle.
        
        Simple and portable for Python workflows.
        Stores entire dataset as a single pickle file.
        
        Pros:
        - Easy to use
        - Supports arbitrary Python objects
        - Good for small-medium datasets
        
        Cons:
        - Not cross-language compatible
        - Loads entire file into memory
        - Security risk with untrusted data
        
        File structure:
        {
            'key1': {'data': [val1, val2, ...], 'timestamps': [t1, t2, ...]},
            'key2': {'data': [val1, val2, ...], 'timestamps': [t1, t2, ...]},
        }
    """
    
    def __init__(self, path: Path, mode: str = 'write', **kwargs):
        """
            Initialize pickle backend.
            
            Args:
                path: Path to pickle file
                mode: 'write' (create new) or 'read' (load existing)
                **kwargs: Additional options
        """
        super().__init__(path=Path(path), **kwargs)
        self.mode = mode
        self._data: dict[str, dict[str, list]] = {}
        self._modified = False
    
    def write(self, key: str, value: Any, timestamp: Optional[float] = None) -> None:
        """
            Write data point.
            
            Args:
                key: Data identifier
                value: Data to store
                timestamp: Optional timestamp
        """
        if not self._is_open:
            raise RuntimeError("Backend not open. Call open() first.")
        
        if self.mode == 'read':
            raise RuntimeError("Cannot write to backend opened in 'read' mode")
        
        if key not in self._data:
            self._data[key] = {'data': [], 'timestamps': []}
        
        if timestamp is None:
            timestamp = time.time()
        
        self._data[key]['data'].append(value)
        self._data[key]['timestamps'].append(timestamp)
        self._modified = True
    
    def read(self, key: str, index: Optional[int] = None) -> Any:
        """
            Read data.
            
            Args:
                key: Data identifier
                index: Optional index (None = all data)
                
            Returns:
                Single value if index provided, list otherwise
        """
        if not self._is_open:
            raise RuntimeError("Backend not open. Call open() first.")
        
        if key not in self._data:
            raise KeyError(f"Key '{key}' not found in dataset")
        
        data = self._data[key]['data']
        
        if index is not None:
            return data[index]
        return data
    
    def keys(self) -> list[str]:
        """Get all keys."""
        return list(self._data.keys())
    
    def __len__(self) -> int:
        """Get number of time steps."""
        if not self._data:
            return 0
        
        first_key = next(iter(self._data))
        return len(self._data[first_key]['data'])
    
    def get_timestamps(self, key: Optional[str] = None) -> list[float]:
        """Get timestamps."""
        if not self._data:
            return []
        
        if key is None:
            key = next(iter(self._data))
        
        if key not in self._data:
            raise KeyError(f"Key '{key}' not found in dataset")
        
        return self._data[key]['timestamps']
    
    def open(self) -> None:
        """
            Open pickle file.
            
            In 'read' mode: Load existing file
            In 'write' mode: Create new file (or overwrite)
        """
        if self._is_open:
            return
        
        if self.mode == 'read':
            # Load existing file
            if not self.path.exists():
                raise FileNotFoundError(f"File not found: {self.path}")
            
            with open(self.path, 'rb') as f:
                self._data = pickle.load(f)
        
        else:  # write mode
            # Create parent directory if needed
            self.path.parent.mkdir(parents=True, exist_ok=True)
            
            # Start with empty data
            self._data = {}
        
        self._is_open = True
        self._modified = False
    
    def close(self) -> None:
        """Close and save file."""
        if not self._is_open:
            return
        
        # Save if modified and in write mode
        if self.mode == 'write' and self._modified:
            self.flush()
        
        self._is_open = False
    
    def flush(self) -> None:
        """Save data to file."""
        if self.mode != 'write':
            return
        
        with open(self.path, 'wb') as f:
            pickle.dump(self._data, f, protocol=pickle.HIGHEST_PROTOCOL)
        
        self._modified = False
    
    def clear(self) -> None:
        """Clear all data."""
        self._data.clear()
        self._modified = True
    
    @classmethod
    def load(cls, path: Path) -> 'PickleBackend':
        """
            Convenience method to load existing dataset.
            
            Args:
                path: Path to pickle file
                
            Returns:
                Opened PickleBackend in read mode
        """
        backend = cls(path=path, mode='read')
        backend.open()
        return backend


if __name__ == '__main__':
    
    print("\n" + "="*70)
    print("PickleBackend - File-Based Dataset Storage")
    print("="*70)
    
    import tempfile
    
    # Create temporary file for demo
    with tempfile.NamedTemporaryFile(suffix='.pkl', delete=False) as tmp:
        tmp_path = Path(tmp.name)
    
    print(f"\nDemo using temporary file: {tmp_path}")
    
    # Write data
    print("\n1. Writing data...")
    with PickleBackend(tmp_path, mode='write') as backend:
        backend.write('position', [1.0, 2.0, 3.0], timestamp=0.0)
        backend.write('position', [1.5, 2.5, 3.5], timestamp=0.1)
        backend.write('velocity', [0.5, 0.5, 0.5], timestamp=0.0)
        backend.write('velocity', [0.6, 0.6, 0.6], timestamp=0.1)
        print(f"   Wrote {len(backend)} time steps")
    
    # Read data
    print("\n2. Reading data...")
    with PickleBackend(tmp_path, mode='read') as backend:
        print(f"   Keys: {backend.keys()}")
        print(f"   Length: {len(backend)}")
        print(f"   Position data: {backend.read('position')}")
        print(f"   Timestamps: {backend.get_timestamps()}")
    
    # Cleanup
    tmp_path.unlink()
    print(f"\n3. Cleaned up temporary file")
    
    print("\n" + "="*70)
    print("PickleBackend provides persistent storage for Python workflows")
    print("For cross-language compatibility, use MCAPBackend instead")
    print("="*70 + "\n")


