#!/usr/bin/env python3

"""
RandomMaterial - Random version of visual_objects/Material.
"""

# BAM
from ..random_type import RandomType
from ..random_float import RandomFloat
from .random_rgba import RandomRGBA
from visual_objects import Material, RGBA

# PYTHON
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class RandomMaterial(RandomType):
    """Random Material with color (RandomRGBA), metallic, and roughness."""

    color: RandomRGBA = field(default_factory=RandomRGBA.grey)
    metallic: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(0.0))
    roughness: RandomFloat = field(default_factory=lambda: RandomFloat.fixed(1.0))

    def with_seed(self, seed: int) -> "RandomMaterial":
        """Set seed for all components (chainable)."""
        self.color.with_seed(seed)
        self.metallic.with_seed(seed + 10)
        self.roughness.with_seed(seed + 11)
        return self

    def sample(self) -> Material:
        """Sample a concrete Material."""
        return Material(
            color=self.color.sample(),
            metallic=self.metallic.sample(),
            roughness=self.roughness.sample(),
        )

    def get_range(self) -> tuple:
        """Get range information for color, metallic, roughness."""
        return (
            self.color.get_range(),
            self.metallic.get_range(),
            self.roughness.get_range(),
        )

    @classmethod
    def fixed(
        cls, rgba: list[float], metallic: float = 0.0, roughness: float = 1.0
    ) -> "RandomMaterial":
        """Create fixed Material from RGBA list."""
        return cls(
            color=RandomRGBA.from_list(rgba),
            metallic=RandomFloat.fixed(metallic),
            roughness=RandomFloat.fixed(roughness),
        )

    @classmethod
    def from_rgba(
        cls, rgba: RGBA, metallic: float = 0.0, roughness: float = 1.0
    ) -> "RandomMaterial":
        """Create fixed Material from RGBA object."""
        return cls(
            color=RandomRGBA.fixed(rgba.r, rgba.g, rgba.b, rgba.a),
            metallic=RandomFloat.fixed(metallic),
            roughness=RandomFloat.fixed(roughness),
        )

    @classmethod
    def red(cls, metallic: float = 0.0, roughness: float = 1.0) -> "RandomMaterial":
        """Create fixed red material."""
        return cls(
            color=RandomRGBA.red(),
            metallic=RandomFloat.fixed(metallic),
            roughness=RandomFloat.fixed(roughness),
        )

    @classmethod
    def green(cls, metallic: float = 0.0, roughness: float = 1.0) -> "RandomMaterial":
        """Create fixed green material."""
        return cls(
            color=RandomRGBA.green(),
            metallic=RandomFloat.fixed(metallic),
            roughness=RandomFloat.fixed(roughness),
        )

    @classmethod
    def blue(cls, metallic: float = 0.0, roughness: float = 1.0) -> "RandomMaterial":
        """Create fixed blue material."""
        return cls(
            color=RandomRGBA.blue(),
            metallic=RandomFloat.fixed(metallic),
            roughness=RandomFloat.fixed(roughness),
        )

    @classmethod
    def grey(
        cls, s: float = 0.5, metallic: float = 0.0, roughness: float = 1.0
    ) -> "RandomMaterial":
        """Create fixed grey material."""
        return cls(
            color=RandomRGBA.grey(s),
            metallic=RandomFloat.fixed(metallic),
            roughness=RandomFloat.fixed(roughness),
        )

    @classmethod
    def uniform(
        cls,
        rgba_lower: list[float],
        rgba_upper: list[float],
        metallic_lower: float = 0.0,
        metallic_upper: float = 1.0,
        roughness_lower: float = 0.0,
        roughness_upper: float = 1.0,
    ) -> "RandomMaterial":
        """Create with uniformly distributed color, metallic, and roughness."""
        return cls(
            color=RandomRGBA.uniform(rgba_lower, rgba_upper),
            metallic=RandomFloat.uniform(metallic_lower, metallic_upper),
            roughness=RandomFloat.uniform(roughness_lower, roughness_upper),
        )


if __name__ == "__main__":
    print("\n[TEST] RandomMaterial")

    # Fixed colors
    mat = RandomMaterial.red().sample()
    print(
        f"Red: rgba={mat.color.to_tuple()}, metallic={mat.metallic}, roughness={mat.roughness}"
    )

    mat = RandomMaterial.green(metallic=0.5).sample()
    print(f"Green (metallic): rgba={mat.color.to_tuple()}, metallic={mat.metallic}")

    mat = RandomMaterial.blue(roughness=0.2).sample()
    print(f"Blue (smooth): rgba={mat.color.to_tuple()}, roughness={mat.roughness}")

    mat = RandomMaterial.grey(0.5).sample()
    print(f"Grey: rgba={mat.color.to_tuple()}")

    # Fixed from list
    mat = RandomMaterial.fixed(
        [1.0, 0.5, 0.0, 1.0], metallic=0.3, roughness=0.7
    ).sample()
    print(
        f"From list: rgba={mat.color.to_tuple()}, metallic={mat.metallic}, roughness={mat.roughness}"
    )

    # Uniform
    rmat = RandomMaterial.uniform(
        rgba_lower=[0.0, 0.0, 0.0, 1.0],
        rgba_upper=[1.0, 1.0, 1.0, 1.0],
        metallic_lower=0.0,
        metallic_upper=1.0,
        roughness_lower=0.0,
        roughness_upper=1.0,
    ).with_seed(42)

    print(f"\nUniform distribution:")
    for i in range(3):
        mat = rmat.sample()
        print(f"  Sample {i}:")
        print(f"    rgba: {mat.color.to_tuple()}")
        print(f"    metallic: {mat.metallic:.3f}, roughness: {mat.roughness:.3f}")

    print("\n✓ RandomMaterial working correctly!")
