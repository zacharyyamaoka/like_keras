#!/usr/bin/env python3

"""
RandomRGBA - Random version of visual_objects/RGBA.
"""

# BAM
from ..random_type import RandomType
from ..random_float import RandomFloat
from visual_objects import RGBA

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class RandomRGBA(RandomType):
    """Random RGBA color with r, g, b, a components."""

    r: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.5))
    g: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.5))
    b: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.5))
    a: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(1.0))

    def with_seed(self, seed: int) -> "RandomRGBA":
        """Set seed for all components (chainable)."""
        self.r.with_seed(seed)
        self.g.with_seed(seed + 1)
        self.b.with_seed(seed + 2)
        self.a.with_seed(seed + 3)
        return self

    def sample(self) -> RGBA:
        """Sample a concrete RGBA."""
        return RGBA(
            r=self.r.sample(), g=self.g.sample(), b=self.b.sample(), a=self.a.sample()
        )

    def get_range(self) -> tuple:
        """Get range information for r, g, b, a."""
        return (
            self.r.get_range(),
            self.g.get_range(),
            self.b.get_range(),
            self.a.get_range(),
        )

    @classmethod
    def fixed(cls, r: float, g: float, b: float, a: float = 1.0) -> "RandomRGBA":
        """Create fixed RGBA color."""
        return cls(
            r=RandomFloat.fixed(r),
            g=RandomFloat.fixed(g),
            b=RandomFloat.fixed(b),
            a=RandomFloat.fixed(a),
        )

    @classmethod
    def from_list(cls, rgba: list[float]) -> "RandomRGBA":
        """Create fixed RGBA from list [r, g, b, a]."""
        return cls.fixed(rgba[0], rgba[1], rgba[2], rgba[3] if len(rgba) > 3 else 1.0)

    @classmethod
    def uniform(cls, rgba_lower: list[float], rgba_upper: list[float]) -> "RandomRGBA":
        """Create with uniformly distributed r, g, b, a."""
        return cls(
            r=RandomFloat.uniform(rgba_lower[0], rgba_upper[0]),
            g=RandomFloat.uniform(rgba_lower[1], rgba_upper[1]),
            b=RandomFloat.uniform(rgba_lower[2], rgba_upper[2]),
            a=RandomFloat.uniform(
                rgba_lower[3] if len(rgba_lower) > 3 else 1.0,
                rgba_upper[3] if len(rgba_upper) > 3 else 1.0,
            ),
        )

    @classmethod
    def red(cls, s: float = 1.0, alpha: float = 1.0) -> "RandomRGBA":
        """Create fixed red color."""
        return cls.fixed(1.0 * s, 0.0, 0.0, alpha)

    @classmethod
    def green(cls, s: float = 1.0, alpha: float = 1.0) -> "RandomRGBA":
        """Create fixed green color."""
        return cls.fixed(0.0, 1.0 * s, 0.0, alpha)

    @classmethod
    def blue(cls, s: float = 1.0, alpha: float = 1.0) -> "RandomRGBA":
        """Create fixed blue color."""
        return cls.fixed(0.0, 0.0, 1.0 * s, alpha)

    @classmethod
    def grey(cls, s: float = 0.5, alpha: float = 1.0) -> "RandomRGBA":
        """Create fixed grey color."""
        return cls.fixed(s, s, s, alpha)


if __name__ == "__main__":
    print("\n[TEST] RandomRGBA")

    # Fixed colors
    rgba = RandomRGBA.red().sample()
    print(f"Red: {rgba.to_tuple()}")

    rgba = RandomRGBA.green().sample()
    print(f"Green: {rgba.to_tuple()}")

    rgba = RandomRGBA.blue().sample()
    print(f"Blue: {rgba.to_tuple()}")

    rgba = RandomRGBA.grey(0.5).sample()
    print(f"Grey: {rgba.to_tuple()}")

    # From list
    rgba = RandomRGBA.from_list([1.0, 0.5, 0.0, 1.0]).sample()
    print(f"From list: {rgba.to_tuple()}")

    # Uniform
    rrgba = RandomRGBA.uniform([0.0, 0.0, 0.0, 1.0], [1.0, 1.0, 1.0, 1.0]).with_seed(42)
    print(f"\nUniform distribution:")
    for i in range(3):
        rgba = rrgba.sample()
        print(f"  Sample {i}: {rgba.to_tuple()}")

    print("\n✓ RandomRGBA working correctly!")
