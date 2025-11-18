#!/usr/bin/env python3

"""
    FloatDist - Float distribution using numpy.random with DataDist interface.
    
    Provides various distribution types (uniform, normal, etc.) compatible with
    gymnasium Space API patterns.
"""

# BAM
from .data_dist import DataDist

# PYTHON
from typing import Optional, Callable
from dataclasses import dataclass, field
import numpy as np


@dataclass
class FloatDist(DataDist):
    """Float distribution with gymnasium Space compatibility.
    
    Use classmethods to construct distributions. Inherits from DataDist
    to provide standard interface for sampling, validation, and bounds.
    """
    
    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_value: Optional[float] = None
    
    def with_seed(self, seed: int) -> 'FloatDist':
        """Set the random seed (chainable).
        
        Args:
            seed: Random seed for reproducibility
            
        Returns:
            self (for chaining)
        """
        self._seed = seed
        self.rng = np.random.default_rng(seed)
        return self
    
    def sample(self) -> float:
        """Sample a concrete float value."""
        self.curr_value = self.sample_fn(self.rng)
        return self.curr_value
    
    def get_range(self) -> tuple[Optional[float], Optional[float]]:
        """Get the (min, max) range."""
        return (self.min_val, self.max_val)
    
    def contains(self, x: float) -> bool:
        """Check if x is within bounds."""
        if self.min_val is not None and x < self.min_val:
            return False
        if self.max_val is not None and x > self.max_val:
            return False
        return True
    
    def save(self) -> float:
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
    def fixed(cls, value: float) -> 'FloatDist':
        """Create a fixed float (always returns same value)."""
        return cls(
            sample_fn=lambda rng: value,
            min_val=value,
            max_val=value
        )
    
    @classmethod
    def discrete(cls, values: list[float]) -> 'FloatDist':
        """Create discrete float (randomly choose from values)."""
        return cls(
            sample_fn=lambda rng: rng.choice(values),
            min_val=min(values),
            max_val=max(values)
        )
    
    @classmethod
    def uniform(cls, low: float, high: float) -> 'FloatDist':
        """Create uniformly distributed float."""
        return cls(
            sample_fn=lambda rng: rng.uniform(low, high),
            min_val=low,
            max_val=high
        )
    
    @classmethod
    def normal(cls, mean: float, std: float) -> 'FloatDist':
        """Create normally distributed float."""
        return cls(
            sample_fn=lambda rng: rng.normal(mean, std),
            min_val=mean - 3*std,
            max_val=mean + 3*std
        )
    
    @classmethod
    def truncated_normal(cls, mean: float, std: float, low: float, high: float) -> 'FloatDist':
        """Create truncated normal distribution (clipped to [low, high])."""
        return cls(
            sample_fn=lambda rng: np.clip(rng.normal(mean, std), low, high),
            min_val=low,
            max_val=high
        )
    
    @classmethod
    def exponential(cls, scale: float, max_val: Optional[float] = None) -> 'FloatDist':
        """Create exponentially distributed float."""
        if max_val is not None:
            sample_fn = lambda rng: min(rng.exponential(scale), max_val)
        else:
            sample_fn = lambda rng: rng.exponential(scale)
        
        return cls(
            sample_fn=sample_fn,
            min_val=0.0,
            max_val=max_val if max_val is not None else scale * 10
        )
    
    @classmethod
    def beta(cls, alpha: float, beta: float, low: float = 0.0, high: float = 1.0) -> 'FloatDist':
        """Create beta distributed float scaled to [low, high]."""
        return cls(
            sample_fn=lambda rng: low + rng.beta(alpha, beta) * (high - low),
            min_val=low,
            max_val=high
        )
    
    @classmethod
    def log_uniform(cls, low: float, high: float) -> 'FloatDist':
        """Create log-uniformly distributed float (spans orders of magnitude)."""
        log_low = np.log10(low)
        log_high = np.log10(high)
        return cls(
            sample_fn=lambda rng: 10 ** rng.uniform(log_low, log_high),
            min_val=low,
            max_val=high
        )
    
    @classmethod
    def custom(cls, sample_fn: Callable[[np.random.Generator], float],
              min_val: Optional[float] = None,
              max_val: Optional[float] = None) -> 'FloatDist':
        """Create custom float with arbitrary sampling function."""
        return cls(
            sample_fn=sample_fn,
            min_val=min_val,
            max_val=max_val
        )


if __name__ == '__main__':
    print("\n" + "="*70)
    print("FloatDist - DataDist-based Float Distribution")
    print("="*70)
    
    # Fixed value
    print("\n[1] Fixed")
    fd_fixed = FloatDist.fixed(5.0)
    print(f"  isinstance(DataDist): {isinstance(fd_fixed, DataDist)}")
    print(f"  Samples: {[fd_fixed.sample() for _ in range(3)]}")
    print(f"  Range: {fd_fixed.get_range()}")
    print(f"  contains(5.0): {fd_fixed.contains(5.0)}")
    print(f"  contains(4.0): {fd_fixed.contains(4.0)}")
    
    # Discrete with seed (chainable)
    print("\n[2] Discrete (with seed)")
    fd_discrete = FloatDist.discrete([1.0, 2.0, 5.0, 10.0]).with_seed(42)
    print(f"  Samples: {[fd_discrete.sample() for _ in range(10)]}")
    print(f"  Range: {fd_discrete.get_range()}")
    
    # Uniform with seed
    print("\n[3] Uniform (with seed)")
    fd_uniform = FloatDist.uniform(0.0, 1.0).with_seed(42)
    print(f"  Samples: {[f'{fd_uniform.sample():.3f}' for _ in range(5)]}")
    print(f"  contains(0.5): {fd_uniform.contains(0.5)}")
    print(f"  contains(2.0): {fd_uniform.contains(2.0)}")
    
    # Normal without seed
    print("\n[4] Normal (no seed - random)")
    fd_normal = FloatDist.normal(0.0, 0.1)
    print(f"  Samples: {[f'{fd_normal.sample():.3f}' for _ in range(5)]}")
    print(f"  Range: {fd_normal.get_range()}")
    
    # Truncated Normal
    print("\n[5] Truncated Normal")
    fd_trunc = FloatDist.truncated_normal(0.5, 0.2, 0.0, 1.0).with_seed(42)
    print(f"  Samples: {[f'{fd_trunc.sample():.3f}' for _ in range(5)]}")
    
    # Exponential
    print("\n[6] Exponential")
    fd_exp = FloatDist.exponential(2.0, max_val=10.0).with_seed(42)
    print(f"  Samples: {[f'{fd_exp.sample():.3f}' for _ in range(5)]}")
    
    # Beta
    print("\n[7] Beta")
    fd_beta = FloatDist.beta(2.0, 5.0, low=0.01, high=0.1).with_seed(42)
    print(f"  Samples: {[f'{fd_beta.sample():.4f}' for _ in range(5)]}")
    
    # Log-uniform
    print("\n[8] Log-uniform")
    fd_log = FloatDist.log_uniform(0.001, 1.0).with_seed(42)
    print(f"  Samples: {[f'{fd_log.sample():.4f}' for _ in range(5)]}")
    
    # Custom
    print("\n[9] Custom (Gamma distribution)")
    fd_custom = FloatDist.custom(
        lambda rng: rng.gamma(shape=2.0, scale=2.0),
        min_val=0.0,
        max_val=20.0
    ).with_seed(42)
    print(f"  Samples: {[f'{fd_custom.sample():.3f}' for _ in range(5)]}")
    
    # Generate dataset
    print("\n[10] Generate dataset")
    fd_test = FloatDist.uniform(-1, 1).with_seed(42)
    dataset = fd_test.generate_dataset(5)
    print(f"  Dataset: {[f'{v:.3f}' for v in dataset]}")
    
    print("\n" + "="*70)
    print("Clean API: Inherits from DataDist with gymnasium Space interface!")
    print("="*70)

