#!/usr/bin/env python3

"""
DataRecorder component for recording time-series data.

Records data flowing through ports into a Dataset.
Can be used for:
- AI-compile verification datasets
- NN-compile training data collection
- System debugging and replay
- Regression testing
"""

# BAM
from lk.common.component import Component
from lk.common.port import InputPort
from lk.msgs.msg import Msg
from lk.common.dataset.dataset import Dataset

# PYTHON
from typing import Optional, Any
from dataclasses import dataclass, field
import time


class DataRecorder(Component):
    """
    Component that records input data to a dataset.

    Supports dynamic input ports - specify ports at initialization
    or add them dynamically during runtime.

    Usage:
        # Create recorder with dataset
        dataset = Dataset.create(backend='memory')
        recorder = DataRecorder(dataset=dataset)

        # Add input ports dynamically
        recorder.add_input('position', PoseStamped)
        recorder.add_input('velocity', Twist)

        # Record data
        recorder.record('position', pose_msg)
        recorder.record('velocity', twist_msg)
    """

    @dataclass
    class Config:
        """
        Configuration for DataRecorder.
        """

        # Recording options
        sample_every_n: int = 1  # Record every Nth call (1 = every time)
        max_samples: Optional[int] = None  # Max samples to record (None = unlimited)
        auto_timestamp: bool = True  # Auto-generate timestamps if not provided

        # Ring buffer options
        use_ring_buffer: bool = False  # Enable ring buffer (overwrites old data)
        ring_buffer_size: int = 1000  # Ring buffer capacity

        # Performance
        auto_flush: bool = False  # Flush after each write (slower but safer)
        flush_every_n: int = 100  # Flush every N samples (if auto_flush=False)

    def __init__(
        self,
        dataset: Dataset,
        name: Optional[str] = None,
        node: Optional[Any] = None,
        config: Optional[Config] = None,
    ):
        """
        Initialize data recorder.

        Args:
            dataset: Dataset to record into
            name: Component name
            node: Parent node
            config: Recorder configuration
        """
        super().__init__(name=name or "DataRecorder", node=node, config=config)

        self.dataset = dataset
        self._sample_count = 0
        self._write_count = 0
        self._port_types: dict[str, type] = {}

    def add_input(self, port_name: str, msg_type: type[Msg]) -> InputPort:
        """
        Add an input port to record.

        Args:
            port_name: Name of the port (will be dataset key)
            msg_type: Message type for this port

        Returns:
            Created InputPort instance
        """
        port = InputPort(port_name, msg_type, owner=self)
        self.inputs.add_port(port_name, port)
        self._port_types[port_name] = msg_type
        return port

    def record(self, key: str, value: Any, timestamp: Optional[float] = None) -> bool:
        """
        Record a data point.

        Args:
            key: Data identifier (port name)
            value: Data to record
            timestamp: Optional timestamp (auto-generated if None and auto_timestamp=True)

        Returns:
            True if data was recorded, False if skipped (due to sampling rate)
        """
        self._sample_count += 1

        # Check sampling rate
        if self._sample_count % self.config.sample_every_n != 0:
            return False

        # Check max samples
        if self.config.max_samples is not None:
            if self._write_count >= self.config.max_samples:
                return False

        # Ring buffer handling
        if self.config.use_ring_buffer:
            key_len = len(self.dataset.backend._data.get(key, {}).get("data", []))
            if key_len >= self.config.ring_buffer_size:
                # Remove oldest sample
                if key in self.dataset.backend._data:
                    self.dataset.backend._data[key]["data"].pop(0)
                    self.dataset.backend._data[key]["timestamps"].pop(0)

        # Auto-timestamp
        if timestamp is None and self.config.auto_timestamp:
            timestamp = time.time()

        # Write to dataset
        self.dataset.write(key, value, timestamp)
        self._write_count += 1

        # Flush if needed
        if self.config.auto_flush:
            self.dataset.flush()
        elif self._write_count % self.config.flush_every_n == 0:
            self.dataset.flush()

        return True

    def record_ports(self, timestamp: Optional[float] = None) -> dict[str, bool]:
        """
        Record all input ports at once.

        Reads current values from all input ports and records them.

        Args:
            timestamp: Optional timestamp for all recordings

        Returns:
            Dictionary mapping port names to recording status (True if recorded)
        """
        results = {}

        for port_name, port in self.inputs._ports.items():
            value = port.value
            if value is not None:
                results[port_name] = self.record(port_name, value, timestamp)
            else:
                results[port_name] = False

        return results

    def __call__(self, **kwargs) -> dict[str, bool]:
        """
        Record data passed as keyword arguments.

        Args:
            **kwargs: key=value pairs to record

        Returns:
            Dictionary of recording statuses

        Example:
            recorder(position=pose, velocity=twist)
        """
        results = {}
        timestamp = kwargs.pop("timestamp", None)

        for key, value in kwargs.items():
            results[key] = self.record(key, value, timestamp)

        return results

    def get_stats(self) -> dict[str, Any]:
        """
        Get recording statistics.

        Returns:
            Dictionary with stats (samples, writes, keys, etc.)
        """
        return {
            "total_samples": self._sample_count,
            "total_writes": self._write_count,
            "dataset_length": len(self.dataset),
            "keys": self.dataset.keys(),
            "recording_rate": self._write_count / max(self._sample_count, 1),
        }

    def flush(self) -> None:
        """Force flush dataset to storage."""
        self.dataset.flush()

    def close(self) -> None:
        """Close recorder and dataset."""
        self.dataset.close()

    def __repr__(self) -> str:
        return f"DataRecorder(name={self.name}, writes={self._write_count}, keys={len(self._port_types)})"


if __name__ == "__main__":

    print("\n" + "=" * 70)
    print("DataRecorder - Component for Recording Time-Series Data")
    print("=" * 70)

    # Example 1: Basic recording
    print("\nExample 1: Basic recording")
    dataset = Dataset.create(backend="memory")
    recorder = DataRecorder(dataset=dataset)

    # Add inputs
    from lk.msgs.msg import Action, Observation

    recorder.add_input("action", Action)
    recorder.add_input("obs", Observation)

    # Record some data
    recorder.record("action", Action(data=[1.0, 2.0]))
    recorder.record("action", Action(data=[1.5, 2.5]))
    recorder.record("obs", Observation(data="test"))

    print(f"  Recorded: {recorder.get_stats()}")
    print(f"  Dataset: {dataset}")

    # Example 2: Callable interface
    print("\nExample 2: Callable interface")
    recorder(action=Action(data=[2.0, 3.0]), obs=Observation(data="hello"))
    print(f"  After callable: {recorder.get_stats()}")

    # Example 3: Sampling rate
    print("\nExample 3: Sampling rate (every 2nd sample)")
    dataset2 = Dataset.create(backend="memory")
    recorder2 = DataRecorder(
        dataset=dataset2, config=DataRecorder.Config(sample_every_n=2)
    )
    recorder2.add_input("x", Action)

    for i in range(10):
        recorded = recorder2.record("x", Action(data=[i]))
        print(f"  Sample {i}: {'recorded' if recorded else 'skipped'}")

    print(f"\n  Final dataset length: {len(dataset2)} (expected 5)")

    # Example 4: Ring buffer
    print("\nExample 4: Ring buffer (size=3)")
    dataset3 = Dataset.create(backend="memory")
    recorder3 = DataRecorder(
        dataset=dataset3,
        config=DataRecorder.Config(use_ring_buffer=True, ring_buffer_size=3),
    )
    recorder3.add_input("y", Action)

    for i in range(5):
        recorder3.record("y", Action(data=[i]))

    data = dataset3["y"]
    print(f"  Recorded 5 samples, buffer kept last 3: {[d.data for d in data]}")

    print("\n" + "=" * 70)
    print("DataRecorder enables easy data collection for AI/NN workflows")
    print("=" * 70 + "\n")
