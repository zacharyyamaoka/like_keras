#!/usr/bin/env python3

"""
Base class for all data distributions.

Provides common interface for sampling, permutation, and configuration.

Greatly inspired by: https://gymnasium.farama.org/api/spaces/
"""

# PYTHON
from abc import ABC, abstractmethod
from typing import Any, Optional, Callable
from dataclasses import dataclass, field
import numpy as np


@dataclass
class DataDist(ABC):
    """Abstract base class for data distributions.

    All data distributions (RandomPoseStamped, RandomMaterial, etc.) inherit from this.
    Provides common interface for:
    - Sampling random values
    - Permuting through discrete options
    - Querying current state
    - Seeding random number generators
    - Configuration serialization

    Design Philosophy:
    - DataDist is a GENERATOR/DISTRIBUTION, not a dataset container
    - Use dist.sample() to generate individual values
    - Use dist.generate_dataset(n) to create datasets for testing
    - The distribution defines HOW to generate data, not the data itself

    Benefits:
    - Easy to check: isinstance(obj, DataDist)
    - Consistent API across all distribution types
    - Type safety and documentation
    - Compatible with Gymnasium's Space API patterns
    """

    sample_fn: Callable[[np.random.Generator], Any]
    min_val: Optional[Any] = None
    max_val: Optional[Any] = None
    _seed: Optional[int] = field(default=None, init=False, repr=False)

    @abstractmethod
    def sample(self, **kwargs) -> Any:
        """Sample a concrete value from the distribution.

        Returns:
            Concrete instance of the base type (e.g., PoseStamped, Material)
        """
        pass

    @abstractmethod
    def get_range(self) -> tuple[Optional[Any], Optional[Any]]:
        """Get the (min, max) range of possible values.

        Returns:
            Tuple of (min, max) values, or (None, None) if unbounded
        """
        pass

    @abstractmethod
    def contains(self, x: Any) -> bool:
        """Check if x is a valid member of this distribution.

        Args:
            x: Value to check

        Returns:
            True if x is valid for this distribution
        """
        pass

    def seed(self, seed: Optional[int] = None) -> int:
        """Seed the random number generator for this distribution.

        Args:
            seed: Seed value for RNG. If None, uses system entropy.

        Returns:
            The seed value that was used

        Raises:
            NotImplementedError: If this distribution doesn't use an RNG
        """
        raise NotImplementedError(f"{self.__class__.__name__} does not support seeding")

    def to_config(self) -> dict[str, Any]:
        """Serialize this distribution to a configuration dictionary.

        Returns:
            Dictionary representation that can be saved to JSON/YAML

        Raises:
            NotImplementedError: If this distribution doesn't support serialization
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} does not support to_config"
        )

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "DataDist":
        """Create a distribution from a configuration dictionary.

        Args:
            config: Dictionary representation from to_config()

        Returns:
            New instance of this distribution

        Raises:
            NotImplementedError: If this distribution doesn't support deserialization
        """
        raise NotImplementedError(f"{cls.__name__} does not support from_config")

    @property
    def shape(self) -> Optional[tuple[int, ...]]:
        """Return the shape of sampled values (for array-like distributions).

        Returns:
            Shape tuple, or None if not applicable
        """
        return None

    @property
    def is_flattenable(self) -> bool:
        """Check if this distribution can be flattened to a vector.

        Returns:
            True if distribution can be flattened
        """
        return False

    def permute(self) -> tuple[Any, bool]:
        """Permute to next value if discrete or random sample if continuous. I think if we want to grid sampling,
        we can also permute? so yes can return a float/int its about wether continous or not.

        1.0,2.0,3.0 is discrete float

        Helpful if you want 100% space coverage.

        Returns:
            Tuple of (value, done_flag)
            - value: The permuted concrete value
            - done_flag: True if we've cycled through all permutations

        Raises:
            NotImplementedError: If this distribution doesn't support permutation
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} does not support permutation"
        )

    def shuffle(self) -> tuple[Any, bool]:
        """Shuffle the distribution, most applicable for when permuting.

        Returns:
            The shuffled distribution
        """
        return self.sample()

    def save(self) -> Any:
        """Save current sampled value (optional).

        Returns:
            The most recently sampled concrete value

        Raises:
            NotImplementedError: If this distribution doesn't support saving
        """
        raise NotImplementedError(f"{self.__class__.__name__} does not support saving")

    @property
    def permutation_total(self) -> int:
        """Get total number of permutations (optional).

        Returns:
            Total number of discrete permutations

        Raises:
            NotImplementedError: If this distribution doesn't support permutation
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} does not have discrete permutations"
        )

    def generate_dataset(self, n: int, permute: bool = False) -> list[Any]:
        """Generate a dataset of n samples from this distribution.

        This is a helper method for testing and data generation.
        The distribution itself doesn't hold data - it generates it.

        Args:
            n: Number of samples to generate
            permute: If True and distribution supports permutation, use permute()
                    instead of sample() for exhaustive coverage

        Returns:
            List of n sampled values

        Example:
            >>> dist = PoseStamped.Dist(...)
            >>> dataset = dist.generate_dataset(100)  # 100 random samples
            >>> test_set = dist.generate_dataset(10, permute=True)  # Exhaustive if discrete
        """
        dataset = []
        if permute:
            try:
                for _ in range(n):
                    value, done = self.permute()
                    dataset.append(value)
                    if done:
                        break
            except NotImplementedError:
                # Fall back to sampling if permutation not supported
                dataset = [self.sample() for _ in range(n)]
        else:
            dataset = [self.sample() for _ in range(n)]
        return dataset


if __name__ == "__main__":

    print("\n" + "=" * 70)
    print("DataDist Base Class")
    print("=" * 70)

    print(
        "\nDataDist is an abstract base class that all data distributions inherit from."
    )
    print("Inspired by Gymnasium's Space API: https://gymnasium.farama.org/api/spaces/")

    print("\n" + "Required methods:")
    print("  - sample() -> Concrete value")
    print("  - get_range() -> (min, max)")
    print("  - contains(x) -> bool")

    print("\n" + "Optional methods with default implementations:")
    print("  - seed(seed) -> int")
    print("  - to_config() -> dict")
    print("  - from_config(config) -> DataDist (classmethod)")
    print("  - permute() -> (value, done)")
    print("  - randomize() -> (value, done)")
    print("  - save() -> value")

    print("\n" + "Properties:")
    print("  - shape -> tuple[int, ...] | None")
    print("  - is_flattenable -> bool")
    print("  - permutation_total -> int")

    print("\n" + "Usage:")
    print("  if isinstance(obj, DataDist):")
    print("      concrete_value = obj.sample()")
    print("      is_valid = obj.contains(some_value)")
    print("      obj.seed(42)  # For reproducibility")
    print("")
    print("  # Generate datasets for testing:")
    print("  dataset = dist.generate_dataset(100)  # 100 random samples")
    print("  test_set = dist.generate_dataset(10, permute=True)  # Exhaustive coverage")

    print("\n" + "=" * 70)
