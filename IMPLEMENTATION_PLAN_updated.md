#!/usr/bin/env python3

"""
    Updated Implementation Plan Based on User Preferences
    
    This document outlines the changes to be made to align with user feedback
    and the Logger architecture pattern.
"""

# =============================================================================
# 1. DATASET ARCHITECTURE - Follow Logger Pattern
# =============================================================================

"""
Current Structure (Branch A):
    lk/common/dataset/
    ├── backends/
    │   ├── base.py          # DatasetBackend ABC
    │   ├── memory.py        # MemoryBackend
    │   └── pickle_backend.py
    ├── dataset.py           # High-level Dataset wrapper
    
New Structure (Logger-like):
    lk/common/dataset/
    ├── backend.py           # Backend ABC (like logger/backend.py)
    ├── memory_backend.py    # MemoryBackend implementation
    ├── pickle_backend.py    # PickleBackend implementation
    ├── mcap_backend.py      # Future: MCAP backend
    ├── dataset.py           # Dataset (like Logger) - routes to backends
    
Changes:
- Rename backends/base.py -> backend.py (flatten structure)
- Move backend implementations to dataset/ level
- Dataset becomes a lightweight router (like Logger)
- Backends handle their own setup and configuration
"""

# Example Dataset class (Logger-like pattern):
"""
class Dataset:
    '''Lightweight multi-backend dataset with context injection.
    
    Like Logger, Dataset is responsible for:
    1. Routing write/read calls to attached backends
    2. Managing multiple backends simultaneously
    3. Creating child datasets (if needed)
    
    Backends handle their own storage configuration.
    '''
    
    def __init__(self, **metadata):
        self.metadata = metadata
        self.backends: dict[str, Backend] = {}
    
    def get_memory_backend(self, **kwargs) -> MemoryBackend:
        '''Get in-memory storage backend.'''
        backend = MemoryBackend(**kwargs)
        self.backends[backend.name] = backend
        return backend
    
    def get_pickle_backend(self, path: Path, **kwargs) -> PickleBackend:
        '''Get pickle file backend.'''
        backend = PickleBackend(path=path, **kwargs)
        self.backends[backend.name] = backend
        return backend
    
    def write(self, key: str, value: Any, timestamp: Optional[float] = None):
        '''Route write to all backends.'''
        for backend in self.backends.values():
            backend.write(key, value, timestamp)
    
    def read(self, key: str, index: Optional[int] = None) -> Any:
        '''Read from primary backend (first added).'''
        if not self.backends:
            raise RuntimeError("No backends attached")
        primary = list(self.backends.values())[0]
        return primary.read(key, index)
    
    def compare(self, other: 'Dataset', tolerance: float = 1e-6) -> 'DatasetComparison':
        '''Compare this dataset with another.'''
        return DatasetComparison(self, other, tolerance=tolerance)
"""

# =============================================================================
# 2. DATARECORDER/DATAPLAYBACK LOCATION
# =============================================================================

"""
User Preference: Keep in dataset/ namespace, use re-exports

Structure:
    lk/common/dataset/
    ├── recorder.py          # DataRecorder component
    ├── playback.py          # DataPlayback component
    └── __init__.py          # Export both
    
    lk/common/__init__.py:
        # Re-export for convenience
        from .dataset import DataRecorder, DataPlayback, Dataset
        
This allows both:
    from lk.common.dataset import DataRecorder  # Direct
    from lk.common import DataRecorder          # Convenient re-export
"""

# =============================================================================
# 3. CONSENSUS RECORDER INHERITANCE
# =============================================================================

"""
ConsensusRecorder should inherit from DataRecorder:

class DataRecorder(Component):
    '''Base recorder that writes to a dataset.'''
    
    def __init__(self, dataset: Dataset, config: Config = None):
        super().__init__()
        self.dataset = dataset
    
    def record(self, key: str, value: Any, timestamp: float = None):
        '''Record a data point.'''
        self.dataset.write(key, value, timestamp)

class ConsensusRecorder(DataRecorder):
    '''Records only when components disagree (active learning).'''
    
    @dataclass
    class Config(DataRecorder.Config):
        tolerance: float = 1e-6
        record_mode: str = 'mismatches'  # 'matches', 'mismatches', 'all'
    
    def __init__(self, dataset: Dataset, components: list[Component], config: Config = None):
        super().__init__(dataset=dataset, config=config)
        self.components = components
    
    def should_record(self, outputs: list[Any]) -> bool:
        '''Check if outputs disagree enough to record.'''
        # Compare outputs, return True if disagreement > tolerance
        pass
"""

# =============================================================================
# 4. DIAGNOSTICS BASE CLASS
# =============================================================================

"""
Problem: Cannot have Diagnostics(Component) and Component.dx reference it
Solution: Diagnostics is NOT a Component, it's a mixin or helper class

Option 1 - Diagnostics as a separate class:
    class Diagnostics:
        '''Diagnostics helper attached to components.'''
        def __init__(self, component: Component):
            self.component = component
            self.stats: dict[str, Any] = {}
        
        def log_stat(self, key: str, value: Any):
            self.stats[key] = value
        
        def get_stats(self) -> dict[str, Any]:
            return self.stats.copy()
    
    class Component:
        def __init__(self, ...):
            self.dx = Diagnostics(self)  # Each component has diagnostics
            self.logger = None  # Set externally or via get_logger()

Option 2 - DiagnosticsMixin:
    class DiagnosticsMixin:
        '''Mixin that adds diagnostics capabilities.'''
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._stats: dict[str, Any] = {}
        
        def log_stat(self, key: str, value: Any):
            self._stats[key] = value
        
        def get_stats(self) -> dict[str, Any]:
            return self._stats.copy()
    
    class Component(DiagnosticsMixin):  # All components have diagnostics
        pass

Recommendation: Option 1 (separate Diagnostics class)
- Cleaner separation
- Can swap implementations
- Diagnostics can be a Component if needed for advanced use cases

ConsensusDx would then be:
    class ConsensusDx(Component):
        '''Component for monitoring consensus across multiple components.'''
        def __init__(self, components: list[Component]):
            super().__init__()
            self.components = components
            self.dx = Diagnostics(self)  # Has its own diagnostics too
"""

# =============================================================================
# 5. COMPILE UTILITIES STRUCTURE
# =============================================================================

"""
User Preference: Create lk.utils.compile package

Structure:
    lk/utils/compile/
    ├── __init__.py
    ├── ai_compile.py      # AI-compile utilities
    └── nn_compile.py      # NN-compile utilities
    
    lk/utils/compile/__init__.py:
        from .ai_compile import Prompt, create_prompt, send_prompt
        from .nn_compile import to_pytorch_dataset, to_dataloader

Usage:
    from lk.utils.compile import create_prompt  # AI compile
    from lk.utils.compile import to_dataloader  # NN compile
    
    # Or specific:
    from lk.utils.compile.ai_compile import Prompt
    from lk.utils.compile.nn_compile import to_pytorch_dataset
"""

# =============================================================================
# 6. DATASET COMPARISON
# =============================================================================

"""
User Preference: dataset.compare() method returning DatasetComparison

class DatasetComparison:
    '''Result of comparing two datasets.'''
    
    def __init__(self, dataset_a: Dataset, dataset_b: Dataset, tolerance: float = 1e-6):
        self.dataset_a = dataset_a
        self.dataset_b = dataset_b
        self.tolerance = tolerance
        
        # Computed on init
        self.keys_match: bool = ...
        self.timestamps_match: bool = ...
        self.values_match: bool = ...
        self.differences: dict[str, KeyDiff] = ...
    
    def to_str(self) -> str:
        '''Generate human-readable summary.'''
        return self._format_summary()
    
    def save_to_file(self, path: Path):
        '''Save comparison to file.'''
        path.write_text(self.to_str())
    
    def plot_histogram(self, key: str):
        '''Plot histogram comparison for a key.'''
        # Uses self.dataset_a and self.dataset_b references
        pass

Usage:
    comparison = dataset_a.compare(dataset_b, tolerance=1e-6)
    print(comparison.to_str())
    comparison.save_to_file('comparison.txt')
    comparison.plot_histogram('sensor_readings')
    
    # Shortcut for quick diff file generation:
    dataset_a.diffdiff(dataset_b, output_file='diff.txt')
    # Equivalent to:
    # comparison = dataset_a.compare(dataset_b)
    # comparison.save_to_file('diff.txt')
"""

# =============================================================================
# IMPLEMENTATION PRIORITY
# =============================================================================

"""
1. ✅ Dataset backend architecture (Logger pattern)
2. ✅ DatasetComparison class with dataset references
3. ✅ Diagnostics helper class
4. ✅ ConsensusRecorder(DataRecorder) inheritance
5. ✅ Compile utilities package structure
6. ✅ Update __init__.py files with re-exports
7. ✅ Update examples to use new APIs
8. ✅ Test everything works together
"""

print(__doc__)

