#!/usr/bin/env python3

"""
In-memory dataset backend.

Fast temporary storage using Python dictionaries.
Ideal for testing and short-lived datasets.
"""

# BAM
from .backend import DatasetBackend

# PYTHON
from typing import Any, Optional
from pathlib import Path
import time


class MemoryBackend(DatasetBackend):
    """
    In-memory storage backend using Python dictionaries.

    Fast and simple, but data is lost when process exits.
    Perfect for:
    - Testing and development
    - Temporary datasets
    - Small datasets that fit in RAM

    Data structure:
    {
        'key1': {'data': [val1, val2, ...], 'timestamps': [t1, t2, ...]},
        'key2': {'data': [val1, val2, ...], 'timestamps': [t1, t2, ...]},
    }
    """

    def __init__(self, path: Optional[Path] = None, **kwargs):
        """
        Initialize memory backend.

        Args:
            path: Ignored for memory backend
            **kwargs: Additional options (ignored)
        """
        super().__init__(path=path, **kwargs)
        self._data: dict[str, dict[str, list]] = {}

    def write(self, key: str, value: Any, timestamp: Optional[float] = None) -> None:
        """
        Write data to memory.

        Args:
            key: Data identifier
            value: Data to store
            timestamp: Optional timestamp (auto-generated if None)
        """
        if not self._is_open:
            raise RuntimeError("Backend not open. Call open() first.")

        if key not in self._data:
            self._data[key] = {"data": [], "timestamps": []}

        if timestamp is None:
            timestamp = time.time()

        self._data[key]["data"].append(value)
        self._data[key]["timestamps"].append(timestamp)

    def read(self, key: str, index: Optional[int] = None) -> Any:
        """
        Read data from memory.

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

        data = self._data[key]["data"]

        if index is not None:
            return data[index]
        return data

    def keys(self) -> list[str]:
        """
        Get all keys in the dataset.

        Returns:
            List of data identifiers
        """
        return list(self._data.keys())

    def __len__(self) -> int:
        """
        Get number of time steps.

        Returns length of first key's data.
        Assumes all keys have same number of samples.
        """
        if not self._data:
            return 0

        first_key = next(iter(self._data))
        return len(self._data[first_key]["data"])

    def get_timestamps(self, key: Optional[str] = None) -> list[float]:
        """
        Get timestamps.

        Args:
            key: Optional key (uses first key if None)

        Returns:
            List of timestamps
        """
        if not self._data:
            return []

        if key is None:
            key = next(iter(self._data))

        if key not in self._data:
            raise KeyError(f"Key '{key}' not found in dataset")

        return self._data[key]["timestamps"]

    def open(self) -> None:
        """Open memory backend (no-op, always ready)."""
        self._is_open = True

    def close(self) -> None:
        """Close memory backend (no-op)."""
        self._is_open = False

    def flush(self) -> None:
        """Flush memory backend (no-op, writes are immediate)."""
        pass

    def clear(self) -> None:
        """Clear all data from memory."""
        self._data.clear()

    def get_data_dict(self) -> dict[str, dict[str, list]]:
        """
        Get direct access to underlying data structure.

        Useful for debugging and serialization.

        Returns:
            Dictionary mapping keys to {data: [...], timestamps: [...]}
        """
        return self._data


if __name__ == "__main__":

    print("\n" + "=" * 70)
    print("MemoryBackend - In-Memory Dataset Storage")
    print("=" * 70)

    # Create and use backend
    backend = MemoryBackend()

    print("\nExample usage:")
    print("1. Opening backend...")
    backend.open()

    print("2. Writing data...")
    backend.write("sensor1", 42.0, timestamp=1.0)
    backend.write("sensor1", 43.5, timestamp=2.0)
    backend.write("sensor2", "hello", timestamp=1.0)
    backend.write("sensor2", "world", timestamp=2.0)

    print(f"3. Keys: {backend.keys()}")
    print(f"4. Length: {len(backend)}")
    print(f"5. Sensor1 data: {backend.read('sensor1')}")
    print(f"6. Sensor1[0]: {backend.read('sensor1', index=0)}")
    print(f"7. Timestamps: {backend.get_timestamps()}")

    print("\n8. Using context manager:")
    with MemoryBackend() as mem:
        mem.write("test", "data")
        print(f"   Wrote data, keys: {mem.keys()}")

    print("\n" + "=" * 70)
    print("MemoryBackend is fast and simple but data is lost on exit")
    print("Use PickleBackend or MCAPBackend for persistent storage")
    print("=" * 70 + "\n")
