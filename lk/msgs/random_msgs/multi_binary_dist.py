#!/usr/bin/env python3

"""
    MultiBinaryDist - Binary value distribution (0 or 1) with DataDist interface.
    
    Supports single binary values or arrays of binary values, similar to
    gymnasium.spaces.MultiBinary. Used for button states, object presence, etc.
"""

# BAM
from .data_dist import DataDist

# PYTHON
from typing import Optional, Callable
from dataclasses import dataclass, field
import numpy as np


@dataclass
class MultiBinaryDist(DataDist):
    """MultiBinary distribution with gymnasium Space compatibility.
    
    Samples binary values (0 or 1) either as single values or arrays.
    Use classmethods to construct distributions.
    """
    
    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_value: Optional[np.ndarray | int] = None
    
    def with_seed(self, seed: int) -> 'MultiBinaryDist':
        """Set the random seed (chainable).
        
        Args:
            seed: Random seed for reproducibility
            
        Returns:
            self (for chaining)
        """
        self._seed = seed
        self.rng = np.random.default_rng(seed)
        return self
    
    def sample(self) -> np.ndarray | int:
        """Sample binary value(s)."""
        self.curr_value = self.sample_fn(self.rng)
        return self.curr_value
    
    def get_range(self) -> tuple[Optional[int | np.ndarray], Optional[int | np.ndarray]]:
        """Get the (min, max) range."""
        return (self.min_val, self.max_val)
    
    def contains(self, x: np.ndarray | int) -> bool:
        """Check if x is valid binary value(s)."""
        # Check if all values are 0 or 1
        if isinstance(x, np.ndarray):
            return np.all((x == 0) | (x == 1))
        else:
            return x in [0, 1]
    
    def save(self) -> np.ndarray | int:
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
        return seed
    
    @classmethod
    def single(cls, p: float = 0.5) -> 'MultiBinaryDist':
        """Create a single binary value distribution.
        
        Args:
            p: Probability of sampling 1 (vs 0)
        """
        return cls(
            sample_fn=lambda rng: int(rng.random() < p),
            min_val=0,
            max_val=1
        )
    
    @classmethod
    def fixed(cls, value: int | np.ndarray) -> 'MultiBinaryDist':
        """Create a fixed binary value or array.
        
        Args:
            value: Fixed binary value(s) to return
        """
        if isinstance(value, np.ndarray):
            return cls(
                sample_fn=lambda rng: value.copy(),
                min_val=np.zeros_like(value),
                max_val=np.ones_like(value)
            )
        else:
            return cls(
                sample_fn=lambda rng: value,
                min_val=0,
                max_val=1
            )
    
    @classmethod
    def array(cls, n: int, p: float = 0.5) -> 'MultiBinaryDist':
        """Create a binary array distribution.
        
        Args:
            n: Size of the binary array
            p: Probability of each element being 1
        """
        return cls(
            sample_fn=lambda rng: (rng.random(n) < p).astype(np.int8),
            min_val=np.zeros(n, dtype=np.int8),
            max_val=np.ones(n, dtype=np.int8)
        )
    
    @classmethod
    def array_2d(cls, shape: tuple[int, int], p: float = 0.5) -> 'MultiBinaryDist':
        """Create a 2D binary array distribution.
        
        Args:
            shape: Shape of the 2D binary array
            p: Probability of each element being 1
        """
        return cls(
            sample_fn=lambda rng: (rng.random(shape) < p).astype(np.int8),
            min_val=np.zeros(shape, dtype=np.int8),
            max_val=np.ones(shape, dtype=np.int8)
        )
    
    @classmethod
    def custom(cls, sample_fn: Callable[[np.random.Generator], np.ndarray | int]) -> 'MultiBinaryDist':
        """Create custom binary distribution with arbitrary sampling function.
        
        Args:
            sample_fn: Function that takes RNG and returns binary value(s)
        """
        # Sample once to determine shape
        test_sample = sample_fn(np.random.default_rng(0))
        if isinstance(test_sample, np.ndarray):
            min_val = np.zeros_like(test_sample)
            max_val = np.ones_like(test_sample)
        else:
            min_val = 0
            max_val = 1
        
        return cls(
            sample_fn=sample_fn,
            min_val=min_val,
            max_val=max_val
        )


if __name__ == '__main__':
    print("\n" + "="*70)
    print("MultiBinaryDist - DataDist-based Binary Distribution")
    print("="*70)
    
    # Single binary value
    print("\n[1] Single binary (p=0.7)")
    mb_single = MultiBinaryDist.single(p=0.7).with_seed(42)
    samples = [mb_single.sample() for _ in range(10)]
    print(f"  Samples: {samples}")
    print(f"  Sum (should be ~7): {sum(samples)}")
    print(f"  Range: {mb_single.get_range()}")
    print(f"  contains(1): {mb_single.contains(1)}")
    print(f"  contains(2): {mb_single.contains(2)}")
    
    # Fixed binary value
    print("\n[2] Fixed binary")
    mb_fixed = MultiBinaryDist.fixed(1)
    print(f"  Samples: {[mb_fixed.sample() for _ in range(5)]}")
    
    # Binary array
    print("\n[3] Binary array (n=5, p=0.5)")
    mb_array = MultiBinaryDist.array(5, p=0.5).with_seed(42)
    print("  Samples:")
    for i in range(5):
        sample = mb_array.sample()
        print(f"    {sample}")
    print(f"  Range min: {mb_array.get_range()[0]}")
    print(f"  Range max: {mb_array.get_range()[1]}")
    
    # 2D binary array
    print("\n[4] 2D binary array (3x4, p=0.3)")
    mb_2d = MultiBinaryDist.array_2d((3, 4), p=0.3).with_seed(42)
    print("  Sample:")
    sample = mb_2d.sample()
    print(sample)
    
    # Validation
    print("\n[5] Validation")
    mb_test = MultiBinaryDist.array(3)
    valid = np.array([1, 0, 1])
    invalid = np.array([1, 2, 0])
    print(f"  contains([1, 0, 1]): {mb_test.contains(valid)}")
    print(f"  contains([1, 2, 0]): {mb_test.contains(invalid)}")
    
    # Generate dataset
    print("\n[6] Generate dataset")
    mb_ds = MultiBinaryDist.single(p=0.7).with_seed(42)
    dataset = mb_ds.generate_dataset(20)
    print(f"  Dataset: {dataset}")
    print(f"  Ones: {sum(dataset)}/20 (should be ~14)")
    
    print("\n" + "="*70)
    print("Clean API: Inherits from DataDist with gymnasium Space interface!")
    print("="*70)




