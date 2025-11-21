#!/usr/bin/env python3

"""
TextDist - Text/string distribution with DataDist interface.

Supports various text generation patterns, similar to gymnasium.spaces.Text.
Used for messages, commands, labels, etc.
"""

# BAM
from .data_dist import DataDist

# PYTHON
from typing import Optional, Callable
from dataclasses import dataclass, field
import numpy as np
import string


@dataclass
class TextDist(DataDist):
    """Text distribution with gymnasium Space compatibility.

    Samples text strings using various patterns.
    Use classmethods to construct distributions.
    """

    def __post_init__(self):
        """Initialize the random number generator."""
        self.rng = np.random.default_rng(self._seed)
        self.curr_value: Optional[str] = None

    def with_seed(self, seed: int) -> "TextDist":
        """Set the random seed (chainable).

        Args:
            seed: Random seed for reproducibility

        Returns:
            self (for chaining)
        """
        self._seed = seed
        self.rng = np.random.default_rng(seed)
        return self

    def sample(self) -> str:
        """Sample a text string."""
        self.curr_value = self.sample_fn(self.rng)
        return self.curr_value

    def get_range(self) -> tuple[Optional[list[str]], Optional[list[str]]]:
        """Get the range of possible values.

        For discrete text, returns (possible_values, possible_values).
        For generated text, returns (None, None).
        """
        return (self.min_val, self.max_val)

    def contains(self, x: str) -> bool:
        """Check if x is a valid text value.

        For discrete text, checks if x is in the list of possible values.
        For generated text, always returns True (any string is valid).
        """
        if self.min_val is not None:
            return x in self.min_val
        return isinstance(x, str)

    def save(self) -> str:
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
    def fixed(cls, text: str) -> "TextDist":
        """Create a fixed text (always returns same string).

        Args:
            text: Fixed string to return
        """
        return cls(sample_fn=lambda rng: text, min_val=[text], max_val=[text])

    @classmethod
    def discrete(cls, texts: list[str]) -> "TextDist":
        """Create discrete text (randomly choose from list).

        Args:
            texts: List of possible strings
        """
        return cls(
            sample_fn=lambda rng: rng.choice(texts), min_val=texts, max_val=texts
        )

    @classmethod
    def random_alphanumeric(cls, min_len: int = 1, max_len: int = 10) -> "TextDist":
        """Create random alphanumeric strings.

        Args:
            min_len: Minimum string length
            max_len: Maximum string length
        """
        chars = string.ascii_letters + string.digits

        def sample_fn(rng):
            length = rng.integers(min_len, max_len + 1)
            return "".join(rng.choice(list(chars)) for _ in range(length))

        return cls(sample_fn=sample_fn, min_val=None, max_val=None)

    @classmethod
    def random_alphabetic(
        cls,
        min_len: int = 1,
        max_len: int = 10,
        lowercase: bool = True,
        uppercase: bool = True,
    ) -> "TextDist":
        """Create random alphabetic strings.

        Args:
            min_len: Minimum string length
            max_len: Maximum string length
            lowercase: Include lowercase letters
            uppercase: Include uppercase letters
        """
        chars = ""
        if lowercase:
            chars += string.ascii_lowercase
        if uppercase:
            chars += string.ascii_uppercase

        if not chars:
            raise ValueError("At least one of lowercase or uppercase must be True")

        def sample_fn(rng):
            length = rng.integers(min_len, max_len + 1)
            return "".join(rng.choice(list(chars)) for _ in range(length))

        return cls(sample_fn=sample_fn, min_val=None, max_val=None)

    @classmethod
    def random_numeric(cls, min_len: int = 1, max_len: int = 10) -> "TextDist":
        """Create random numeric strings.

        Args:
            min_len: Minimum string length
            max_len: Maximum string length
        """

        def sample_fn(rng):
            length = rng.integers(min_len, max_len + 1)
            return "".join(rng.choice(list(string.digits)) for _ in range(length))

        return cls(sample_fn=sample_fn, min_val=None, max_val=None)

    @classmethod
    def template(
        cls, template: str, variables: dict[str, "TextDist | list[str]"]
    ) -> "TextDist":
        """Create text from a template with variable substitution.

        Args:
            template: Template string with {variable_name} placeholders
            variables: Dict of variable names to TextDist or list of strings

        Example:
            TextDist.template(
                "Hello {name}, you are {age} years old",
                {
                    "name": TextDist.discrete(["Alice", "Bob"]),
                    "age": ["20", "25", "30"]
                }
            )
        """
        # Convert list of strings to TextDist.discrete
        var_dists = {}
        for key, val in variables.items():
            if isinstance(val, list):
                var_dists[key] = cls.discrete(val)
            else:
                var_dists[key] = val

        def sample_fn(rng):
            substitutions = {key: dist.sample() for key, dist in var_dists.items()}
            return template.format(**substitutions)

        return cls(sample_fn=sample_fn, min_val=None, max_val=None)

    @classmethod
    def custom(
        cls,
        sample_fn: Callable[[np.random.Generator], str],
        possible_values: Optional[list[str]] = None,
    ) -> "TextDist":
        """Create custom text with arbitrary sampling function.

        Args:
            sample_fn: Function that takes RNG and returns a string
            possible_values: Optional list of all possible values (for discrete distributions)
        """
        return cls(
            sample_fn=sample_fn, min_val=possible_values, max_val=possible_values
        )


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("TextDist - DataDist-based Text Distribution")
    print("=" * 70)

    # Fixed text
    print("\n[1] Fixed")
    td_fixed = TextDist.fixed("Hello World")
    print(f"  Samples: {[td_fixed.sample() for _ in range(3)]}")
    print(f"  contains('Hello World'): {td_fixed.contains('Hello World')}")
    print(f"  contains('Goodbye'): {td_fixed.contains('Goodbye')}")

    # Discrete text
    print("\n[2] Discrete")
    td_discrete = TextDist.discrete(["apple", "banana", "cherry"]).with_seed(42)
    print(f"  Samples: {[td_discrete.sample() for _ in range(10)]}")
    print(f"  contains('apple'): {td_discrete.contains('apple')}")
    print(f"  contains('orange'): {td_discrete.contains('orange')}")

    # Random alphanumeric
    print("\n[3] Random alphanumeric (len 5-8)")
    td_alphanum = TextDist.random_alphanumeric(5, 8).with_seed(42)
    print(f"  Samples: {[td_alphanum.sample() for _ in range(5)]}")

    # Random alphabetic
    print("\n[4] Random alphabetic lowercase (len 3-6)")
    td_alpha = TextDist.random_alphabetic(
        3, 6, lowercase=True, uppercase=False
    ).with_seed(42)
    print(f"  Samples: {[td_alpha.sample() for _ in range(5)]}")

    # Random numeric
    print("\n[5] Random numeric (len 4-6)")
    td_numeric = TextDist.random_numeric(4, 6).with_seed(42)
    print(f"  Samples: {[td_numeric.sample() for _ in range(5)]}")

    # Template
    print("\n[6] Template")
    td_template = TextDist.template(
        "Hello {name}, you scored {score} points!",
        {
            "name": TextDist.discrete(["Alice", "Bob", "Charlie"]),
            "score": ["100", "200", "300"],
        },
    ).with_seed(42)
    print("  Samples:")
    for i in range(5):
        print(f"    {td_template.sample()}")

    # Generate dataset
    print("\n[7] Generate dataset")
    td_ds = TextDist.discrete(["red", "green", "blue"]).with_seed(42)
    dataset = td_ds.generate_dataset(10)
    print(f"  Dataset: {dataset}")

    print("\n" + "=" * 70)
    print("Clean API: Inherits from DataDist with gymnasium Space interface!")
    print("=" * 70)
