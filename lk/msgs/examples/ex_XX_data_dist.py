from dataclasses import dataclass, field
from typing import Optional, Callable, Union
from numpy.dtypes import UInt16DType
from lk.msgs import PoseStamped
import numpy as np

# Should make default type
pose = PoseStamped()

# Should sample the it with the defaul distrbution
pose = PoseStamped().sample()

# Idea is to use changing to edit it?
# I do like the idea of keeping the main init function quite clear? well technically it should hold the info no? well I want it to be a mixin I guess?
# Technically each of the x,y,z could have its own distrubtion? Ok how about we make the basic class able to hold a float?
# I think if we replace the most basic units with a distburion then it becomes very easy the challenge is we don't want the verobnisty of having to type floats...

# It should automatically wrap all normal floats in Float() and Int with lk.Int()
# Each float is associate with a distribution.. FloatDist()  IntDist() Float64() data

FloatType = float | Float32 | Float64 | FloatBase
IntType = int | Int8 | Int16 | Int32 | Int64 | IntBase
# UInt16DType
BoolType = bool | Bool


@dataclass
class FloatBase:
    data: float = 0.0


pose = PoseStamped().data_dist()


@dataclass
class FloatDistMixin:

    sample_fn: Callable[[np.random.Generator], float]

    min: Optional[float] = None
    max: Optional[float] = None
    _seed: Optional[int] = field(default=None, init=False, repr=False)

    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.sample_data: Optional[float] = None

    def with_seed(self, seed: int) -> "FloatDistMixin":
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
        self.sample_data = self.sample_fn(self.rng)
        return self.sample_data

    def get_range(self) -> tuple[Optional[float], Optional[float]]:
        """Get the (min, max) range."""
        return (self.min_val, self.max_val)

    def save(self) -> float:
        """Save current value."""
        return self.sample_data

    def fixed(self, value: float) -> "FloatDistMixin":
        """Create a fixed float (always returns same value)."""
        self.sample_fn = lambda rng: value
        self.min = value
        self.max = value
        return self

    def discrete(self, values: list[float]) -> "FloatDistMixin":
        """Create discrete float (randomly choose from values)."""
        self.sample_fn = lambda rng: rng.choice(values)
        self.min_val = min(values)
        self.max_val = max(values)
        return self

    def uniform(self, low: float, high: float) -> "FloatDistMixin":
        """Create uniformly distributed float."""
        self.sample_fn = lambda rng: rng.uniform(low, high)
        self.min_val = (low,)
        self.max_val = high
        return self

    def normal(
        self: Union[FloatBase, "FloatDistMixin"],
        mean: Optional[float] = None,
        std: float = 1.0,
    ) -> "FloatDistMixin":
        """Create normally distributed float. commnoly set as Float.normal(std=1e-2)"""

        mean = mean if mean else self.data

        self.sample_fn = lambda rng: rng.normal(mean, std)  # Union gives type hint here
        self.min_val = (
            mean - 3 * std
        )  # 3 sigma gives 9th percentile (encompasses 95% of data)
        self.max_val = mean + 3 * std
        return self

    def truncated_normal(
        self: Union[FloatBase, "FloatDistMixin"],
        mean: Optional[float] = None,
        std: float = 1.0,
        low: Optional[float] = None,
        high: Optional[float] = None,
    ) -> "FloatDistMixin":
        """Create truncated normal distribution (clipped to [low, high])."""

        mean = mean if mean else self.data
        low = low if low else (mean - 3 * std)
        high = high if high else (mean - 3 * std)

        self.sample_fn = lambda rng: np.clip(rng.normal(mean, std), high, high)
        self.min_val = low
        self.max_val = high

    def exponential(
        self, scale: float, max_val: Optional[float] = None
    ) -> "FloatDistMixin":
        """Create exponentially distributed float."""
        if max_val is not None:
            sample_fn = lambda rng: min(rng.exponential(scale), max_val)
        else:
            sample_fn = lambda rng: rng.exponential(scale)

        self.sample_fn = (sample_fn,)
        self.min_val = (0.0,)
        self.max_val = max_val if max_val is not None else scale * 10

    def beta(
        self, alpha: float, beta: float, low: float = 0.0, high: float = 1.0
    ) -> "FloatDistMixin":
        """Create beta distributed float scaled to [low, high]."""
        self.sample_fn = (lambda rng: low + rng.beta(alpha, beta) * (high - low),)
        self.min_val = (low,)
        self.max_val = high

    def log_uniform(self, low: float, high: float) -> "FloatDistMixin":
        """Create log-uniformly distributed float (spans orders of magnitude)."""
        log_low = np.log10(low)
        log_high = np.log10(high)
        self.sample_fn = (lambda rng: 10 ** rng.uniform(log_low, log_high),)
        self.min_val = (low,)
        self.max_val = high

    def custom(
        self,
        sample_fn: Callable[[np.random.Generator], float],
        min_val: Optional[float] = None,
        max_val: Optional[float] = None,
    ) -> "FloatDistMixin":
        """Create custom float with arbitrary sampling function."""
        self.sample_fn = (sample_fn,)
        self.min_val = (min_val,)
        self.max_val = max_val
