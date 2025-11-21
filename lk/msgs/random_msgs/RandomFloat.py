#!/usr/bin/env python3

"""
RandomFloat - Base random scalar value using numpy.random directly.

Stores a sampling function (lambda) that uses numpy.random.Generator.
"""

# BAM
from .RandomType import RandomType

# PYTHON
from typing import Optional, Callable
from dataclasses import dataclass, field
import numpy as np


@dataclass
class RandomFloat(RandomType):
    """Random float value using numpy.random directly.

    Stores a sampling function that takes an RNG and returns a float.
    """

    sample_fn: Callable[[np.random.Generator], float]
    min_val: Optional[float] = None
    max_val: Optional[float] = None
    _seed: Optional[int] = field(default=None, init=False, repr=False)

    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_value: Optional[float] = None

    def with_seed(self, seed: int) -> "RandomFloat":
        """Set the random seed (chainable).

        DescriptionArgs:
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

    def save(self) -> float:
        """Save current value."""
        return self.curr_value

    @classmethod
    def fixed(cls, value: float) -> "RandomFloat":
        """Create a fixed float (always returns same value)."""
        return cls(sample_fn=lambda rng: value, min_val=value, max_val=value)

    @classmethod
    def discrete(cls, values: list[float]) -> "RandomFloat":
        """Create discrete float (randomly choose from values)."""
        return cls(
            sample_fn=lambda rng: rng.choice(values),
            min_val=min(values),
            max_val=max(values),
        )

    @classmethod
    def uniform(cls, low: float, high: float) -> "RandomFloat":
        """Create uniformly distributed float."""
        return cls(
            sample_fn=lambda rng: rng.uniform(low, high), min_val=low, max_val=high
        )

    @classmethod
    def normal(cls, mean: float, std: float) -> "RandomFloat":
        """Create normally distributed float."""
        return cls(
            sample_fn=lambda rng: rng.normal(mean, std),
            min_val=mean - 3 * std,
            max_val=mean + 3 * std,
        )

    @classmethod
    def truncated_normal(
        cls, mean: float, std: float, low: float, high: float
    ) -> "RandomFloat":
        """Create truncated normal distribution (clipped to [low, high])."""
        return cls(
            sample_fn=lambda rng: np.clip(rng.normal(mean, std), low, high),
            min_val=low,
            max_val=high,
        )

    @classmethod
    def exponential(
        cls, scale: float, max_val: Optional[float] = None
    ) -> "RandomFloat":
        """Create exponentially distributed float."""
        if max_val is not None:
            sample_fn = lambda rng: min(rng.exponential(scale), max_val)
        else:
            sample_fn = lambda rng: rng.exponential(scale)

        return cls(
            sample_fn=sample_fn,
            min_val=0.0,
            max_val=max_val if max_val is not None else scale * 10,
        )

    @classmethod
    def beta(
        cls, alpha: float, beta: float, low: float = 0.0, high: float = 1.0
    ) -> "RandomFloat":
        """Create beta distributed float scaled to [low, high]."""
        return cls(
            sample_fn=lambda rng: low + rng.beta(alpha, beta) * (high - low),
            min_val=low,
            max_val=high,
        )

    @classmethod
    def log_uniform(cls, low: float, high: float) -> "RandomFloat":
        """Create log-uniformly distributed float (spans orders of magnitude)."""
        log_low = np.log10(low)
        log_high = np.log10(high)
        return cls(
            sample_fn=lambda rng: 10 ** rng.uniform(log_low, log_high),
            min_val=low,
            max_val=high,
        )

    @classmethod
    def custom(
        cls,
        sample_fn: Callable[[np.random.Generator], float],
        min_val: Optional[float] = None,
        max_val: Optional[float] = None,
    ) -> "RandomFloat":
        """Create custom float with arbitrary sampling function."""
        return cls(sample_fn=sample_fn, min_val=min_val, max_val=max_val)


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("RandomFloat - Chainable .with_seed()")
    print("=" * 70)

    # Fixed value
    print("\n[1] Fixed")
    rf_fixed = RandomFloat.fixed(5.0)
    print(f"  isinstance(RandomType): {isinstance(rf_fixed, RandomType)}")
    print(f"  Samples: {[rf_fixed.sample() for _ in range(3)]}")

    # Discrete with seed (chainable)
    print("\n[2] Discrete (with seed)")
    rf_discrete = RandomFloat.discrete([1.0, 2.0, 5.0, 10.0]).with_seed(42)
    print(f"  Samples: {[rf_discrete.sample() for _ in range(10)]}")

    # Uniform with seed
    print("\n[3] Uniform (with seed)")
    rf_uniform = RandomFloat.uniform(0.0, 1.0).with_seed(42)
    print(f"  Samples: {[f'{rf_uniform.sample():.3f}' for _ in range(5)]}")

    # Normal without seed
    print("\n[4] Normal (no seed - random)")
    rf_normal = RandomFloat.normal(0.0, 0.1)
    print(f"  Samples: {[f'{rf_normal.sample():.3f}' for _ in range(5)]}")

    # Truncated Normal
    print("\n[5] Truncated Normal")
    rf_trunc = RandomFloat.truncated_normal(0.5, 0.2, 0.0, 1.0).with_seed(42)
    print(f"  Samples: {[f'{rf_trunc.sample():.3f}' for _ in range(5)]}")

    # Exponential
    print("\n[6] Exponential")
    rf_exp = RandomFloat.exponential(2.0, max_val=10.0).with_seed(42)
    print(f"  Samples: {[f'{rf_exp.sample():.3f}' for _ in range(5)]}")

    # Beta
    print("\n[7] Beta")
    rf_beta = RandomFloat.beta(2.0, 5.0, low=0.01, high=0.1).with_seed(42)
    print(f"  Samples: {[f'{rf_beta.sample():.4f}' for _ in range(5)]}")

    # Log-uniform
    print("\n[8] Log-uniform")
    rf_log = RandomFloat.log_uniform(0.001, 1.0).with_seed(42)
    print(f"  Samples: {[f'{rf_log.sample():.4f}' for _ in range(5)]}")

    # Custom
    print("\n[9] Custom (Gamma distribution)")
    rf_custom = RandomFloat.custom(
        lambda rng: rng.gamma(shape=2.0, scale=2.0), min_val=0.0, max_val=20.0
    ).with_seed(42)
    print(f"  Samples: {[f'{rf_custom.sample():.3f}' for _ in range(5)]}")

    # Chaining demo
    print("\n[10] Chaining demo")
    rf_chain = RandomFloat.uniform(0, 1).with_seed(999)
    print(f"  Created with fluent interface: {rf_chain.sample():.3f}")

    print("\n" + "=" * 70)
    print("Clean API: .with_seed(42) is chainable and optional!")
    print("=" * 70)
