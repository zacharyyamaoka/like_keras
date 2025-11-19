#!/usr/bin/env python3

"""
    DataPlayback component for replaying recorded data.
    
    Plays back time-series data from a Dataset through output ports.
    Supports:
    - Real-time factor (RTF) control
    - Looping
    - Start offset and rate scaling
    - Time-accurate playback
"""

# BAM
from lk.common.component import Component
from lk.common.port import OutputPort
from lk.msgs.msg import Msg
from lk.common.dataset.dataset import Dataset

# PYTHON
from typing import Optional, Any
from dataclasses import dataclass
import time


class DataPlayback(Component):
    """
        Component that plays back recorded data from a dataset.
        
        Automatically creates output ports based on dataset keys.
        Supports time-accurate playback with configurable rate.
        
        Usage:
            # Load dataset
            dataset = Dataset.load('recording.pkl')
            
            # Create playback
            playback = DataPlayback(dataset=dataset)
            
            # Play back data
            while not playback.done:
                playback.step()  # Advance one time step
                pose = playback.outputs['position'].value
                
            # Or use real-time playback
            playback.play_realtime(rate=2.0)  # 2x speed
    """
    
    @dataclass
    class Config:
        """
            Configuration for DataPlayback.
        """
        # Playback control
        rate: float = 1.0  # Playback speed multiplier (1.0 = real-time, 2.0 = 2x speed)
        loop: bool = False  # Loop back to start when finished
        start_offset: float = 0.0  # Skip initial seconds
        
        # Time handling
        use_recorded_timestamps: bool = True  # Use dataset timestamps vs sequential
        rtf: Optional[float] = None  # Real-time factor (alias for rate, takes precedence)
        
        # Auto-play
        auto_play: bool = False  # Automatically start playback on init
    
    def __init__(self,
                 dataset: Dataset,
                 name: Optional[str] = None,
                 node: Optional[Any] = None,
                 config: Optional[Config] = None):
        """
            Initialize data playback.
            
            Args:
                dataset: Dataset to play back
                name: Component name
                node: Parent node
                config: Playback configuration
        """
        super().__init__(name=name or "DataPlayback", node=node, config=config)
        
        self.dataset = dataset
        self._current_index = 0
        self._is_playing = False
        self._start_time: Optional[float] = None
        self._playback_start_time: Optional[float] = None
        
        # Use RTF if specified (takes precedence over rate)
        if self.config.rtf is not None:
            self.config.rate = self.config.rtf
        
        # Create output ports for each key in dataset
        self._create_output_ports()
        
        # Apply start offset
        if self.config.start_offset > 0:
            self._skip_to_offset()
        
        # Auto-play if configured
        if self.config.auto_play:
            self.play()
    
    def _create_output_ports(self) -> None:
        """
            Create output ports based on dataset keys.
            
            Inspects first value to determine message type.
        """
        for key in self.dataset.keys():
            # Get first value to determine type
            try:
                first_value = self.dataset.read(key, index=0)
                msg_type = type(first_value)
                
                # Ensure it's a Msg subclass
                if not issubclass(msg_type, Msg):
                    print(f"Warning: {key} contains non-Msg data ({msg_type}), skipping port creation")
                    continue
                
                port = OutputPort(key, msg_type, owner=self)
                self.outputs.add_port(key, port)
                
            except (IndexError, KeyError):
                print(f"Warning: {key} has no data, skipping port creation")
    
    def _skip_to_offset(self) -> None:
        """Skip to start_offset time."""
        timestamps = self.dataset.get_timestamps()
        if not timestamps:
            return
        
        start_time = timestamps[0] + self.config.start_offset
        
        for i, ts in enumerate(timestamps):
            if ts >= start_time:
                self._current_index = i
                break
    
    def step(self) -> bool:
        """
            Advance playback by one time step.
            
            Reads data at current index and writes to output ports.
            
            Returns:
                True if step was successful, False if at end
        """
        if self._current_index >= len(self.dataset):
            if self.config.loop:
                self.reset()
            else:
                self._is_playing = False
                return False
        
        # Read data at current index
        for key in self.dataset.keys():
            try:
                value = self.dataset.read(key, index=self._current_index)
                port = self.outputs.get_port(key)
                if port is not None:
                    port.write(value)
            except (IndexError, KeyError):
                pass
        
        self._current_index += 1
        return True
    
    def play(self) -> None:
        """Start playback."""
        self._is_playing = True
        self._playback_start_time = time.time()
        
        timestamps = self.dataset.get_timestamps()
        if timestamps and self._current_index < len(timestamps):
            self._start_time = timestamps[self._current_index]
    
    def pause(self) -> None:
        """Pause playback."""
        self._is_playing = False
    
    def reset(self) -> None:
        """Reset playback to beginning."""
        self._current_index = 0
        self._start_time = None
        self._playback_start_time = None
        
        if self.config.start_offset > 0:
            self._skip_to_offset()
    
    def seek(self, index: int) -> None:
        """
            Seek to specific index.
            
            Args:
                index: Time step index to seek to
        """
        self._current_index = max(0, min(index, len(self.dataset)))
    
    def play_realtime(self, rate: Optional[float] = None, blocking: bool = True) -> None:
        """
            Play back data in real-time (or scaled time).
            
            Uses timestamps to maintain timing accuracy.
            
            Args:
                rate: Playback speed (overrides config.rate if provided)
                blocking: If True, blocks until playback complete
        """
        if rate is not None:
            original_rate = self.config.rate
            self.config.rate = rate
        
        self.play()
        
        if not blocking:
            return
        
        timestamps = self.dataset.get_timestamps()
        if not timestamps:
            # No timestamps, just step through
            while not self.done:
                self.step()
            return
        
        playback_start = time.time()
        data_start_time = timestamps[self._current_index] if self._current_index < len(timestamps) else timestamps[0]
        
        while not self.done:
            if self._current_index >= len(timestamps):
                break
            
            # Calculate when this sample should be played
            curr_data_time = timestamps[self._current_index]
            elapsed_data_time = curr_data_time - data_start_time
            target_playback_time = playback_start + (elapsed_data_time / self.config.rate)
            
            # Wait until it's time
            wait_time = target_playback_time - time.time()
            if wait_time > 0:
                time.sleep(wait_time)
            
            self.step()
        
        # Restore original rate if changed
        if rate is not None:
            self.config.rate = original_rate
    
    @property
    def done(self) -> bool:
        """Check if playback is finished."""
        return self._current_index >= len(self.dataset) and not self.config.loop
    
    @property
    def progress(self) -> float:
        """
            Get playback progress.
            
            Returns:
                Progress as float 0.0 to 1.0
        """
        if len(self.dataset) == 0:
            return 1.0
        return self._current_index / len(self.dataset)
    
    def __call__(self) -> dict[str, Any]:
        """
            Step and return current values.
            
            Returns:
                Dictionary mapping port names to current values
        """
        self.step()
        return {key: self.outputs.get_port(key).value for key in self.dataset.keys()}
    
    def get_current_values(self) -> dict[str, Any]:
        """
            Get current values at all output ports.
            
            Returns:
                Dictionary mapping port names to current values
        """
        return {key: self.outputs.get_port(key).value 
                for key in self.dataset.keys() 
                if self.outputs.get_port(key) is not None}
    
    def __repr__(self) -> str:
        return f"DataPlayback(name={self.name}, progress={self.progress:.1%}, done={self.done})"


if __name__ == '__main__':
    
    print("\n" + "="*70)
    print("DataPlayback - Component for Replaying Time-Series Data")
    print("="*70)
    
    # Create sample dataset
    print("\nCreating sample dataset...")
    dataset = Dataset.create(backend='memory')
    
    from lk.msgs.msg import Action, Observation
    
    for i in range(5):
        dataset.write('action', Action(data=[i, i+1]), timestamp=i * 0.1)
        dataset.write('obs', Observation(data=f'obs_{i}'), timestamp=i * 0.1)
    
    print(f"  Created dataset: {dataset}")
    
    # Example 1: Step-by-step playback
    print("\nExample 1: Step-by-step playback")
    playback = DataPlayback(dataset=dataset)
    
    print(f"  Output ports: {list(playback.outputs._ports.keys())}")
    
    for i in range(3):
        playback.step()
        values = playback.get_current_values()
        print(f"  Step {i}: action={values['action'].data}, obs={values['obs'].data}")
    
    # Example 2: Loop playback
    print("\nExample 2: Loop playback")
    playback2 = DataPlayback(
        dataset=dataset,
        config=DataPlayback.Config(loop=True)
    )
    
    print("  Playing 8 steps (5 dataset + 3 looped):")
    for i in range(8):
        playback2.step()
        print(f"    Step {i}: index={playback2._current_index-1}, done={playback2.done}")
    
    # Example 3: Real-time playback (fast)
    print("\nExample 3: Real-time playback at 10x speed")
    playback3 = DataPlayback(dataset=dataset)
    
    start = time.time()
    playback3.play_realtime(rate=10.0, blocking=True)
    elapsed = time.time() - start
    
    print(f"  Played {len(dataset)} samples in {elapsed:.3f}s")
    print(f"  Expected ~{0.4/10:.3f}s (dataset spans 0.4s, 10x speed)")
    
    # Example 4: Start offset
    print("\nExample 4: Start offset (skip first 0.2s)")
    playback4 = DataPlayback(
        dataset=dataset,
        config=DataPlayback.Config(start_offset=0.2)
    )
    
    print(f"  Started at index: {playback4._current_index} (expected 2)")
    
    print("\n" + "="*70)
    print("DataPlayback enables time-accurate replay of recorded data")
    print("="*70 + "\n")


