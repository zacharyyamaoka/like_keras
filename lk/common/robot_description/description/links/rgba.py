"""
Internally we hold as RGBA floats between 0 and 1.
"""

# PYTHON
from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    plt = None


@dataclass
class RGBA:
    r: float
    g: float
    b: float
    a: float = 1.0

    @classmethod
    def from_value(cls, value: Any) -> "RGBA":
        if isinstance(value, RGBA):
            return value
        elif isinstance(value, tuple):
            return cls.from_tuple(value)
        elif isinstance(value, list):
            return cls.from_list(value)
        elif isinstance(value, np.ndarray):
            return cls.from_numpy(value)
        else:
            raise ValueError(f"Invalid value type: {type(value)}")

    @classmethod
    def from_numpy(
        cls, array: np.ndarray, base8: bool = False, bgr: bool = False
    ) -> "RGBA":

        scale = 255.0 if base8 else 1.0
        array = array / scale

        if len(array) == 3:
            if bgr:
                return cls(array[2], array[1], array[0], 1.0)
            else:
                return cls(array[0], array[1], array[2], 1.0)
        elif len(array) == 4:
            if bgr:
                return cls(array[2], array[1], array[0], array[3])
            else:
                return cls(array[0], array[1], array[2], array[3])
        else:
            raise ValueError(f"Invalid array length: {len(array)}")

    @classmethod
    def from_list(
        cls, values: list[float], base8: bool = False, bgr: bool = False
    ) -> "RGBA":
        return cls.from_numpy(np.array(values), base8, bgr)

    @classmethod
    def from_tuple(
        cls, value: tuple[float], base8: bool = False, bgr: bool = False
    ) -> "RGBA":
        return cls.from_numpy(np.array(list(value)), base8, bgr)

    def to_numpy(
        self, alpha: bool = True, base8: bool = False, bgr: bool = False
    ) -> np.ndarray:

        scale = 255.0 if base8 else 1.0

        if alpha:
            if bgr:
                color = np.array([self.b, self.g, self.r, self.a]) * scale
            else:
                color = np.array([self.r, self.g, self.b, self.a]) * scale
        else:
            if bgr:
                color = np.array([self.b, self.g, self.r]) * scale
            else:
                color = np.array([self.r, self.g, self.b]) * scale

        if base8:
            return color.astype(int)
        else:
            return color

    def to_list(
        self, alpha: bool = True, base8: bool = False, bgr: bool = False
    ) -> list[float]:
        return self.to_numpy(alpha, base8, bgr).tolist()

    def to_tuple(
        self, alpha: bool = True, base8: bool = False, bgr: bool = False
    ) -> tuple:
        return tuple(self.to_list(alpha, base8, bgr))

    def scale(self, factor: float, scale_alpha: bool = False) -> "RGBA":
        """Scale color values by a factor and clip to valid range [0, 1].

        DescriptionArgs:
            factor: Multiplier for color values (< 1 darkens, > 1 brightens)
            scale_alpha: If True, also scales the alpha channel (default: False)

        Returns:
            New RGBA instance with scaled and clipped values
        """
        r = np.clip(self.r * factor, 0.0, 1.0)
        g = np.clip(self.g * factor, 0.0, 1.0)
        b = np.clip(self.b * factor, 0.0, 1.0)
        a = np.clip(self.a * factor, 0.0, 1.0) if scale_alpha else self.a
        return RGBA(r, g, b, a)

    @classmethod
    def to_gradient_list(
        cls,
        start_color: "RGBA" = None,
        end_color: "RGBA" = None,
        num_points: int = 10,
        cmap=None,
    ) -> list["RGBA"]:
        """Generate a gradient list by interpolating between two colors or using a matplotlib colormap.

        DescriptionArgs:
            start_color: The starting RGBA color (required if cmap is None)
            end_color: The ending RGBA color (required if cmap is None)
            num_points: The number of color points in the gradient
            cmap: Optional matplotlib colormap (string name or colormap object)

        Returns:
            List of RGBA colors interpolated between start and end, or sampled from cmap
        """
        if num_points < 1:
            raise ValueError("num_points must be at least 1")

        # If cmap is provided, use matplotlib colormap
        if cmap is not None:
            if plt is None:
                raise ModuleNotFoundError(
                    "matplotlib is required to sample colormaps. Install it or pass explicit start/end colors."
                )
            # Get the colormap
            if isinstance(cmap, str):
                cmap = plt.get_cmap(cmap)

            # Sample the colormap
            gradient = []
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0.5
                rgba = cmap(t)
                gradient.append(cls(rgba[0], rgba[1], rgba[2], rgba[3]))

            return gradient

        # Otherwise, interpolate between start and end colors
        if start_color is None or end_color is None:
            raise ValueError(
                "start_color and end_color are required when cmap is not provided"
            )

        if num_points < 2:
            raise ValueError(
                "num_points must be at least 2 when using start_color and end_color"
            )

        gradient = []
        for i in range(num_points):
            t = i / (num_points - 1)
            r = start_color.r + t * (end_color.r - start_color.r)
            g = start_color.g + t * (end_color.g - start_color.g)
            b = start_color.b + t * (end_color.b - start_color.b)
            a = start_color.a + t * (end_color.a - start_color.a)
            gradient.append(cls(r, g, b, a))

        return gradient

    @classmethod
    def random(cls) -> "RGBA":
        rgba = np.random.uniform(0.5, 1.0, 4).tolist()
        return cls(*rgba)

    @classmethod
    def grey(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(0.5 * s, 0.5 * s, 0.5 * s, alpha)

    @classmethod
    def purple(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(0.5 * s, 0.0 * s, 0.5 * s, alpha)

    @classmethod
    def red(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(1.0 * s, 0.0 * s, 0.0 * s, alpha)

    @classmethod
    def green(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(0.0 * s, 1.0 * s, 0.0 * s, alpha)

    @classmethod
    def blue(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(0.0 * s, 0.0 * s, 1.0 * s, alpha)

    @classmethod
    def yellow(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(1.0 * s, 1.0 * s, 0.0 * s, alpha)

    @classmethod
    def cyan(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(0.0 * s, 1.0 * s, 1.0 * s, alpha)

    @classmethod
    def magenta(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(1.0 * s, 0.0 * s, 1.0 * s, alpha)

    @classmethod
    def white(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(1.0 * s, 1.0 * s, 1.0 * s, alpha)

    @classmethod
    def black(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(0.0 * s, 0.0 * s, 0.0 * s, alpha)

    @classmethod
    def orange(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(1.0 * s, 0.5 * s, 0.0 * s, alpha)

    @classmethod
    def origin_gold(cls, s: float = 1.0, alpha: float = 1.0) -> "RGBA":
        return cls(236 / 255 * s, 236 / 255 * s, 0 / 255 * s, alpha)


if __name__ == "__main__":
    rgba = RGBA.random()
    print(rgba)
    # Assert round-trip for base8, bgr, and both combined

    arr = np.array([0.25, 0.5, 0.75, 1.0])  # example RGBA

    # base8
    rgba = RGBA.from_numpy(arr, base8=False, bgr=False)
    base8 = rgba.to_numpy(base8=True, bgr=False)
    round_trip = RGBA.from_numpy(base8, base8=True, bgr=False)
    assert np.allclose(
        round_trip.to_numpy(base8=True, bgr=False), base8
    ), "base8 round-trip failed"

    # bgr
    rgba = RGBA.from_numpy(arr, base8=False, bgr=False)
    bgr = rgba.to_numpy(base8=False, bgr=True)
    round_trip = RGBA.from_numpy(bgr, base8=False, bgr=True)
    assert np.allclose(
        round_trip.to_numpy(base8=False, bgr=True), bgr
    ), "bgr round-trip failed"

    # base8 + bgr
    rgba = RGBA.from_numpy(arr, base8=False, bgr=False)
    base8_bgr = rgba.to_numpy(base8=True, bgr=True)
    round_trip = RGBA.from_numpy(base8_bgr, base8=True, bgr=True)
    assert np.allclose(
        round_trip.to_numpy(base8=True, bgr=True), base8_bgr
    ), "base8+bgr round-trip failed"

    arr = np.array([0.25, 0.5, 0.75])  # example RGBA

    # base8
    rgba = RGBA.from_numpy(arr, base8=False, bgr=False)
    base8 = rgba.to_numpy(alpha=False, base8=True, bgr=False)
    round_trip = RGBA.from_numpy(base8, base8=True, bgr=False)
    assert np.allclose(
        round_trip.to_numpy(alpha=False, base8=True, bgr=False), base8
    ), "base8 round-trip failed"

    # bgr
    rgba = RGBA.from_numpy(arr, base8=False, bgr=False)
    bgr = rgba.to_numpy(alpha=False, base8=False, bgr=True)
    round_trip = RGBA.from_numpy(bgr, base8=False, bgr=True)
    assert np.allclose(
        round_trip.to_numpy(alpha=False, base8=False, bgr=True), bgr
    ), "bgr round-trip failed"

    # base8 + bgr
    rgba = RGBA.from_numpy(arr, base8=False, bgr=False)
    base8_bgr = rgba.to_numpy(alpha=False, base8=True, bgr=True)
    round_trip = RGBA.from_numpy(base8_bgr, base8=True, bgr=True)
    assert np.allclose(
        round_trip.to_numpy(alpha=False, base8=True, bgr=True), base8_bgr
    ), "base8+bgr round-trip failed"

    # Test gradient
    gradient = RGBA.to_gradient_list(
        RGBA.from_list([1.0, 0.0, 0.0, 1.0]), RGBA.from_list([0.0, 0.0, 1.0, 1.0]), 5
    )
    # Test that the gradient produces correct colors at the ends and length matches
    assert len(gradient) == 5, "Gradient should have 5 colors"
    assert gradient[0].to_tuple() == (
        1.0,
        0.0,
        0.0,
        1.0,
    ), "Gradient should start with the start color"
    assert gradient[-1].to_tuple() == (
        0.0,
        0.0,
        1.0,
        1.0,
    ), "Gradient should end with the end color"
    # Also check that intermediate values are interpolated
    expected_middle = RGBA(0.5, 0.0, 0.5, 1.0).to_tuple()
    assert np.allclose(
        gradient[2].to_tuple(), expected_middle
    ), "Gradient midpoint color is not as expected"
    print(gradient)

    # Test gradient with matplotlib colormap
    gradient_cmap = RGBA.to_gradient_list(num_points=5, cmap="viridis")
    assert len(gradient_cmap) == 5, "Colormap gradient should have 5 colors"
    print(gradient_cmap)

    # Test scale method
    color = RGBA.red(s=1.0, alpha=0.8)
    darkened = color.scale(0.5)
    brightened = color.scale(1.5)
    assert (
        darkened.r == 0.5 and darkened.g == 0.0 and darkened.b == 0.0
    ), "Darkening failed"
    assert darkened.a == 0.8, "Alpha should not change by default"
    assert brightened.r == 1.0, "Should clip to 1.0"
    # Test with alpha scaling
    darkened_with_alpha = color.scale(0.5, scale_alpha=True)
    assert darkened_with_alpha.a == 0.4, "Alpha should scale when requested"
    print(f"Original: {color}, Darkened: {darkened}, Brightened: {brightened}")

    print("All tests passed! ✅")
