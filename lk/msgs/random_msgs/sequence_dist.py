#!/usr/bin/env python3

"""
    SequenceDist - Variable-length sequence distribution with DataDist interface.
    
    Supports sequences (lists) of varying length where each element is sampled
    from the same distribution. Similar to gymnasium.spaces.Sequence.
    
    Used for entity lists, action sequences, variable-length observations, etc.
"""

# BAM
from .data_dist import DataDist
from .int_dist import IntDist

# PYTHON
from typing import Optional, Callable, Any
from dataclasses import dataclass, field
import numpy as np


@dataclass
class SequenceDist(DataDist):
    """Sequence distribution with gymnasium Space compatibility.
    
    Samples variable-length sequences where:
    1. The length is sampled from a length distribution (IntDist)
    2. Each element is sampled from the same element distribution (DataDist)
    
    Use classmethods to construct distributions.
    """
    
    # Override sample_fn - we'll handle it differently for sequences
    length_dist: IntDist = field(default=None)
    element_dist: DataDist = field(default=None)
    
    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_value: Optional[list] = None
        
        # For sequences, sample_fn is not used directly
        # Instead, we use length_dist and element_dist
        if self.sample_fn is None:
            self.sample_fn = lambda rng: None  # Placeholder
    
    def with_seed(self, seed: int) -> 'SequenceDist':
        """Set the random seed (chainable).
        
        Args:
            seed: Random seed for reproducibility
            
        Returns:
            self (for chaining)
        """
        self._seed = seed
        self.rng = np.random.default_rng(seed)
        # Also seed the sub-distributions
        if self.length_dist is not None:
            self.length_dist.seed(seed)
        if self.element_dist is not None:
            self.element_dist.seed(seed + 1000)
        return self
    
    def sample(self) -> list:
        """Sample a sequence.
        
        First samples the length, then samples that many elements.
        """
        # Sample the length
        length = self.length_dist.sample()
        
        # Sample elements
        elements = [self.element_dist.sample() for _ in range(length)]
        
        self.curr_value = elements
        return elements
    
    def get_range(self) -> tuple[Optional[dict], Optional[dict]]:
        """Get the range of possible values.
        
        Returns a dict with 'length' and 'element' ranges.
        """
        length_range = self.length_dist.get_range()
        element_range = self.element_dist.get_range()
        
        min_dict = {
            'length': length_range[0],
            'element': element_range[0]
        }
        max_dict = {
            'length': length_range[1],
            'element': element_range[1]
        }
        
        return (min_dict, max_dict)
    
    def contains(self, x: list) -> bool:
        """Check if x is a valid sequence.
        
        Validates:
        1. x is a list
        2. Length is within bounds
        3. All elements are valid according to element_dist
        """
        if not isinstance(x, list):
            return False
        
        # Check length
        length_min, length_max = self.length_dist.get_range()
        if length_min is not None and len(x) < length_min:
            return False
        if length_max is not None and len(x) > length_max:
            return False
        
        # Check all elements
        for elem in x:
            if not self.element_dist.contains(elem):
                return False
        
        return True
    
    def save(self) -> list:
        """Save current value."""
        return self.curr_value
    
    def seed(self, seed: Optional[int] = None) -> int:
        """Seed the random number generator for this distribution.
        
        Args:
            seed: Seed value for RNG. If None, uses system entropy.
            
        Returns:
            The seed value that was used
        """
        if seed is None:
            seed = np.random.randint(0, 2**31)
        self._seed = seed
        self.rng = np.random.default_rng(seed)
        # Seed sub-distributions
        self.length_dist.seed(seed)
        self.element_dist.seed(seed + 1000)
        return seed
    
    @classmethod
    def create(cls, length_dist: IntDist, element_dist: DataDist) -> 'SequenceDist':
        """Create a sequence distribution.
        
        Args:
            length_dist: Distribution for sampling sequence length
            element_dist: Distribution for sampling each element
        
        Example:
            SequenceDist.create(
                length_dist=IntDist.uniform(1, 5),  # Length 1-4
                element_dist=FloatDist.uniform(0, 1)  # Elements in [0, 1]
            )
        """
        return cls(
            sample_fn=None,  # Not used for sequences
            min_val=None,
            max_val=None,
            length_dist=length_dist,
            element_dist=element_dist
        )
    
    @classmethod
    def fixed_length(cls, length: int, element_dist: DataDist) -> 'SequenceDist':
        """Create a fixed-length sequence distribution.
        
        Args:
            length: Fixed sequence length
            element_dist: Distribution for sampling each element
        """
        return cls.create(
            length_dist=IntDist.fixed(length),
            element_dist=element_dist
        )
    
    @classmethod
    def uniform_length(cls, min_len: int, max_len: int, element_dist: DataDist) -> 'SequenceDist':
        """Create a sequence with uniform length distribution.
        
        Args:
            min_len: Minimum sequence length (inclusive)
            max_len: Maximum sequence length (exclusive)
            element_dist: Distribution for sampling each element
        """
        return cls.create(
            length_dist=IntDist.uniform(min_len, max_len),
            element_dist=element_dist
        )


if __name__ == '__main__':
    from .float_dist import FloatDist
    from .text_dist import TextDist
    
    print("\n" + "="*70)
    print("SequenceDist - DataDist-based Sequence Distribution")
    print("="*70)
    
    # Fixed length sequence
    print("\n[1] Fixed length (n=5) with uniform floats")
    seq_fixed = SequenceDist.fixed_length(
        length=5,
        element_dist=FloatDist.uniform(0, 1)
    ).with_seed(42)
    
    print("  Samples:")
    for i in range(3):
        sample = seq_fixed.sample()
        print(f"    Length {len(sample)}: {[f'{x:.3f}' for x in sample]}")
    
    # Variable length sequence
    print("\n[2] Variable length (1-5) with normal floats")
    seq_var = SequenceDist.uniform_length(
        min_len=1,
        max_len=5,
        element_dist=FloatDist.normal(0.0, 0.1)
    ).with_seed(42)
    
    print("  Samples:")
    for i in range(5):
        sample = seq_var.sample()
        print(f"    Length {len(sample)}: {[f'{x:.3f}' for x in sample]}")
    
    # Sequence of text
    print("\n[3] Variable length (2-4) with discrete text")
    seq_text = SequenceDist.uniform_length(
        min_len=2,
        max_len=4,
        element_dist=TextDist.discrete(["red", "green", "blue"])
    ).with_seed(42)
    
    print("  Samples:")
    for i in range(5):
        sample = seq_text.sample()
        print(f"    Length {len(sample)}: {sample}")
    
    # Validation
    print("\n[4] Validation")
    seq_test = SequenceDist.uniform_length(
        min_len=2,
        max_len=5,
        element_dist=FloatDist.uniform(0, 1)
    )
    
    valid = [0.5, 0.3, 0.8]
    invalid_len = [0.5]  # Too short
    invalid_elem = [0.5, 2.0, 0.3]  # 2.0 out of range
    
    print(f"  contains([0.5, 0.3, 0.8]): {seq_test.contains(valid)}")
    print(f"  contains([0.5]) (too short): {seq_test.contains(invalid_len)}")
    print(f"  contains([0.5, 2.0, 0.3]) (elem out of range): {seq_test.contains(invalid_elem)}")
    
    # Range
    print("\n[5] Range information")
    seq_range = SequenceDist.uniform_length(
        min_len=3,
        max_len=7,
        element_dist=FloatDist.uniform(-1, 1)
    )
    min_dict, max_dict = seq_range.get_range()
    print(f"  Length range: [{min_dict['length']}, {max_dict['length']})")
    print(f"  Element range: [{min_dict['element']}, {max_dict['element']}]")
    
    # Generate dataset
    print("\n[6] Generate dataset")
    seq_ds = SequenceDist.uniform_length(
        min_len=2,
        max_len=4,
        element_dist=IntDist.uniform(0, 10)
    ).with_seed(42)
    
    dataset = seq_ds.generate_dataset(5)
    print("  Dataset:")
    for i, seq in enumerate(dataset):
        print(f"    {i+1}. Length {len(seq)}: {seq}")
    
    # Nested sequences (sequence of Points)
    print("\n[7] Sequence of composite messages (advanced)")
    print("  (Would need Point.Dist - see comprehensive examples)")
    
    print("\n" + "="*70)
    print("Clean API: Inherits from DataDist with gymnasium Space interface!")
    print("Variable-length sequences with configurable element distributions")
    print("="*70)




