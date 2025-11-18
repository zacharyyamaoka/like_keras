#!/usr/bin/env python3

"""
    IntDist - Int distribution using numpy.random with DataDist interface.
    
    Provides various distribution types (uniform, poisson, etc.) compatible with
    gymnasium Space API patterns.
"""

# BAM
from .data_dist import DataDist

# PYTHON
from typing import Optional, Callable
from dataclasses import dataclass, field
import numpy as np


@dataclass
class IntDist(DataDist):
    """Int distribution with gymnasium Space compatibility.
    
    Use classmethods to construct distributions. Inherits from DataDist
    to provide standard interface for sampling, validation, and bounds.
    """
    
    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_value: Optional[int] = None
    
    def with_seed(self, seed: int) -> 'IntDist':
        """Set the random seed (chainable).
        
        Args:
            seed: Random seed for reproducibility
            
        Returns:
            self (for chaining)
        """
        self._seed = seed
        self.rng = np.random.default_rng(seed)
        return self
    
    def sample(self) -> int:
        """Sample a concrete int value."""
        self.curr_value = self.sample_fn(self.rng)
        return self.curr_value
    
    def get_range(self) -> tuple[Optional[int], Optional[int]]:
        """Get the (min, max) range."""
        return (self.min_val, self.max_val)
    
    def contains(self, x: int) -> bool:
        """Check if x is within bounds."""
        if self.min_val is not None and x < self.min_val:
            return False
        if self.max_val is not None and x > self.max_val:
            return False
        return True
    
    def save(self) -> int:
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
    def fixed(cls, value: int) -> 'IntDist':
        """Create a fixed int (always returns same value)."""
        return cls(
            sample_fn=lambda rng: value,
            min_val=value,
            max_val=value
        )
    
    @classmethod
    def discrete(cls, values: list[int]) -> 'IntDist':
        """Create discrete int (randomly choose from values)."""
        return cls(
            sample_fn=lambda rng: int(rng.choice(values)),
            min_val=min(values),
            max_val=max(values)
        )
    
    @classmethod
    def uniform(cls, low: int, high: int) -> 'IntDist':
        """Create uniformly distributed int in [low, high).
        
        Args:
            low: Lower bound (inclusive)
            high: Upper bound (exclusive)
        """
        return cls(
            sample_fn=lambda rng: rng.integers(low, high),
            min_val=low,
            max_val=high - 1
        )
    
    @classmethod
    def poisson(cls, lam: float, max_val: Optional[int] = None) -> 'IntDist':
        """Create Poisson distributed int.
        
        Args:
            lam: Expected number of events (lambda parameter)
            max_val: Optional maximum value to clip at
        """
        if max_val is not None:
            sample_fn = lambda rng: min(int(rng.poisson(lam)), max_val)
        else:
            sample_fn = lambda rng: int(rng.poisson(lam))
        
        return cls(
            sample_fn=sample_fn,
            min_val=0,
            max_val=max_val if max_val is not None else int(lam * 5)
        )
    
    @classmethod
    def binomial(cls, n: int, p: float) -> 'IntDist':
        """Create binomial distributed int.
        
        Args:
            n: Number of trials
            p: Probability of success per trial
        """
        return cls(
            sample_fn=lambda rng: int(rng.binomial(n, p)),
            min_val=0,
            max_val=n
        )
    
    @classmethod
    def geometric(cls, p: float, max_val: Optional[int] = None) -> 'IntDist':
        """Create geometric distributed int.
        
        Args:
            p: Probability of success
            max_val: Optional maximum value to clip at
        """
        if max_val is not None:
            sample_fn = lambda rng: min(int(rng.geometric(p)), max_val)
        else:
            sample_fn = lambda rng: int(rng.geometric(p))
        
        return cls(
            sample_fn=sample_fn,
            min_val=1,
            max_val=max_val if max_val is not None else int(10 / p)
        )
    
    @classmethod
    def pareto(cls, shape: float, min_val: int = 1, max_val: Optional[int] = None) -> 'IntDist':
        """Create Pareto distributed int (power-law distribution).
        
        Useful for sampling class IDs where few classes are common and many are rare.
        Lower shape values create more inequality (heavier tail).
        
        Args:
            shape: Shape parameter (alpha). Smaller values = more inequality.
                   Typical range: 1.0 to 3.0 (1.5 is a good default for class sampling)
            min_val: Minimum value (scale parameter), default 1
            max_val: Optional maximum value to clip at
        """
        if max_val is not None:
            sample_fn = lambda rng: min(int(rng.pareto(shape) + min_val), max_val)
        else:
            sample_fn = lambda rng: int(rng.pareto(shape) + min_val)
        
        return cls(
            sample_fn=sample_fn,
            min_val=min_val,
            max_val=max_val if max_val is not None else min_val * 100
        )
    
    @classmethod
    def custom(cls, sample_fn: Callable[[np.random.Generator], int],
              min_val: Optional[int] = None,
              max_val: Optional[int] = None) -> 'IntDist':
        """Create custom int with arbitrary sampling function."""
        return cls(
            sample_fn=sample_fn,
            min_val=min_val,
            max_val=max_val
        )


if __name__ == '__main__':
    print("\n" + "="*70)
    print("IntDist - DataDist-based Int Distribution")
    print("="*70)
    
    # Fixed value
    print("\n[1] Fixed")
    id_fixed = IntDist.fixed(5)
    print(f"  isinstance(DataDist): {isinstance(id_fixed, DataDist)}")
    print(f"  Samples: {[id_fixed.sample() for _ in range(3)]}")
    print(f"  Range: {id_fixed.get_range()}")
    
    # Discrete with seed (chainable)
    print("\n[2] Discrete (with seed)")
    id_discrete = IntDist.discrete([1, 2, 5, 10]).with_seed(42)
    print(f"  Samples: {[id_discrete.sample() for _ in range(10)]}")
    print(f"  Range: {id_discrete.get_range()}")
    
    # Uniform with seed
    print("\n[3] Uniform [0, 10) (with seed)")
    id_uniform = IntDist.uniform(0, 10).with_seed(42)
    print(f"  Samples: {[id_uniform.sample() for _ in range(10)]}")
    print(f"  Range: {id_uniform.get_range()}")
    print(f"  contains(5): {id_uniform.contains(5)}")
    print(f"  contains(10): {id_uniform.contains(10)}")
    
    # Poisson
    print("\n[4] Poisson (λ=3.0)")
    id_poisson = IntDist.poisson(3.0).with_seed(42)
    print(f"  Samples: {[id_poisson.sample() for _ in range(10)]}")
    print(f"  Range: {id_poisson.get_range()}")
    
    # Binomial
    print("\n[5] Binomial (n=10, p=0.3)")
    id_binomial = IntDist.binomial(10, 0.3).with_seed(42)
    print(f"  Samples: {[id_binomial.sample() for _ in range(10)]}")
    print(f"  Range: {id_binomial.get_range()}")
    
    # Geometric
    print("\n[6] Geometric (p=0.2)")
    id_geometric = IntDist.geometric(0.2).with_seed(42)
    print(f"  Samples: {[id_geometric.sample() for _ in range(10)]}")
    print(f"  Range: {id_geometric.get_range()}")
    
    # Pareto (for class IDs)
    print("\n[7] Pareto (shape=1.5, min=0, max=20)")
    id_pareto = IntDist.pareto(shape=1.5, min_val=0, max_val=20).with_seed(42)
    samples = [id_pareto.sample() for _ in range(30)]
    print(f"  Samples: {samples}")
    print(f"  Range: {id_pareto.get_range()}")
    print(f"  Distribution (few classes common, many rare):")
    from collections import Counter
    counts = Counter(samples)
    for cls_id in sorted(counts.keys())[:10]:  # Show first 10
        print(f"    Class {cls_id:2d}: {'█' * counts[cls_id]}")
    
    # Custom
    print("\n[8] Custom (Negative Binomial)")
    id_custom = IntDist.custom(
        lambda rng: int(rng.negative_binomial(n=5, p=0.5)),
        min_val=0,
        max_val=50
    ).with_seed(42)
    print(f"  Samples: {[id_custom.sample() for _ in range(10)]}")
    print(f"  Range: {id_custom.get_range()}")
    
    # Generate dataset
    print("\n[9] Generate dataset")
    id_test = IntDist.uniform(0, 10).with_seed(42)
    dataset = id_test.generate_dataset(10)
    print(f"  Dataset: {dataset}")
    
    print("\n" + "="*70)
    print("Clean API: Inherits from DataDist with gymnasium Space interface!")
    print("Integer-specific distributions: poisson, binomial, geometric, pareto")
    print("="*70)

