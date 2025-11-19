#!/usr/bin/env python3

"""
    RandomInt - Base random integer value using numpy.random directly.
    
    Stores a sampling function (lambda) that uses numpy.random.Generator.
"""

# BAM
from .random_type import RandomType

# PYTHON
from typing import Optional, Callable
from dataclasses import dataclass, field
import numpy as np


@dataclass
class RandomInt(RandomType):
    """Random int value using numpy.random directly.
    
    Stores a sampling function that takes an RNG and returns an int.
    """
    
    sample_fn: Callable[[np.random.Generator], int]
    min_val: Optional[int] = None
    max_val: Optional[int] = None
    _seed: Optional[int] = field(default=None, init=False, repr=False)
    
    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_value: Optional[int] = None
    
    def with_seed(self, seed: int) -> 'RandomInt':
        """Set the random seed (chainable).
        
        DescriptionArgs:
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
    
    def save(self) -> int:
        """Save current value."""
        return self.curr_value
    
    @classmethod
    def fixed(cls, value: int) -> 'RandomInt':
        """Create a fixed int (always returns same value)."""
        return cls(
            sample_fn=lambda rng: value,
            min_val=value,
            max_val=value
        )
    
    @classmethod
    def discrete(cls, values: list[int]) -> 'RandomInt':
        """Create discrete int (randomly choose from values)."""
        return cls(
            sample_fn=lambda rng: int(rng.choice(values)),
            min_val=min(values),
            max_val=max(values)
        )
    
    @classmethod
    def uniform(cls, low: int, high: int) -> 'RandomInt':
        """Create uniformly distributed int in [low, high).
        
        DescriptionArgs:
            low: Lower bound (inclusive)
            high: Upper bound (exclusive)
        """
        return cls(
            sample_fn=lambda rng: rng.integers(low, high),
            min_val=low,
            max_val=high - 1
        )
    
    @classmethod
    def poisson(cls, lam: float, max_val: Optional[int] = None) -> 'RandomInt':
        """Create Poisson distributed int.
        
        DescriptionArgs:
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
    def binomial(cls, n: int, p: float) -> 'RandomInt':
        """Create binomial distributed int.
        
        DescriptionArgs:
            n: Number of trials
            p: Probability of success per trial
        """
        return cls(
            sample_fn=lambda rng: int(rng.binomial(n, p)),
            min_val=0,
            max_val=n
        )
    
    @classmethod
    def geometric(cls, p: float, max_val: Optional[int] = None) -> 'RandomInt':
        """Create geometric distributed int.
        
        DescriptionArgs:
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
    def pareto(cls, shape: float, min_val: int = 1, max_val: Optional[int] = None) -> 'RandomInt':
        """Create Pareto distributed int (power-law distribution).
        
        Useful for sampling class IDs where few classes are common and many are rare.
        Lower shape values create more inequality (heavier tail).
        
        DescriptionArgs:
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
              max_val: Optional[int] = None) -> 'RandomInt':
        """Create custom int with arbitrary sampling function."""
        return cls(
            sample_fn=sample_fn,
            min_val=min_val,
            max_val=max_val
        )


if __name__ == '__main__':
    print("\n" + "="*70)
    print("RandomInt - Chainable .with_seed()")
    print("="*70)
    
    # Fixed value
    print("\n[1] Fixed")
    ri_fixed = RandomInt.fixed(5)
    print(f"  isinstance(RandomType): {isinstance(ri_fixed, RandomType)}")
    print(f"  Samples: {[ri_fixed.sample() for _ in range(3)]}")
    print(f"  Range: {ri_fixed.get_range()}")
    
    # Discrete with seed (chainable)
    print("\n[2] Discrete (with seed)")
    ri_discrete = RandomInt.discrete([1, 2, 5, 10]).with_seed(42)
    print(f"  Samples: {[ri_discrete.sample() for _ in range(10)]}")
    print(f"  Range: {ri_discrete.get_range()}")
    
    # Uniform with seed
    print("\n[3] Uniform [0, 10) (with seed)")
    ri_uniform = RandomInt.uniform(0, 10).with_seed(42)
    print(f"  Samples: {[ri_uniform.sample() for _ in range(10)]}")
    print(f"  Range: {ri_uniform.get_range()}")
    
    # Poisson
    print("\n[4] Poisson (λ=3.0)")
    ri_poisson = RandomInt.poisson(3.0).with_seed(42)
    print(f"  Samples: {[ri_poisson.sample() for _ in range(10)]}")
    print(f"  Range: {ri_poisson.get_range()}")
    
    # Binomial
    print("\n[5] Binomial (n=10, p=0.3)")
    ri_binomial = RandomInt.binomial(10, 0.3).with_seed(42)
    print(f"  Samples: {[ri_binomial.sample() for _ in range(10)]}")
    print(f"  Range: {ri_binomial.get_range()}")
    
    # Geometric
    print("\n[6] Geometric (p=0.2)")
    ri_geometric = RandomInt.geometric(0.2).with_seed(42)
    print(f"  Samples: {[ri_geometric.sample() for _ in range(10)]}")
    print(f"  Range: {ri_geometric.get_range()}")
    
    # Pareto (for class IDs)
    print("\n[7] Pareto (shape=1.5, min=0, max=20)")
    ri_pareto = RandomInt.pareto(shape=1.5, min_val=0, max_val=20).with_seed(42)
    samples = [ri_pareto.sample() for _ in range(30)]
    print(f"  Samples: {samples}")
    print(f"  Range: {ri_pareto.get_range()}")
    print(f"  Distribution (few classes common, many rare):")
    from collections import Counter
    counts = Counter(samples)
    for cls_id in sorted(counts.keys()):
        print(f"    Class {cls_id:2d}: {'█' * counts[cls_id]}")
    
    # Custom
    print("\n[8] Custom (Negative Binomial)")
    ri_custom = RandomInt.custom(
        lambda rng: int(rng.negative_binomial(n=5, p=0.5)),
        min_val=0,
        max_val=50
    ).with_seed(42)
    print(f"  Samples: {[ri_custom.sample() for _ in range(10)]}")
    print(f"  Range: {ri_custom.get_range()}")
    
    # Chaining demo
    print("\n[9] Chaining demo")
    ri_chain = RandomInt.uniform(1, 100).with_seed(999)
    print(f"  Created with fluent interface: {ri_chain.sample()}")
    
    print("\n" + "="*70)
    print("Clean API: .with_seed(42) is chainable and optional!")
    print("Integer-specific distributions: poisson, binomial, geometric, pareto")
    print("="*70)

