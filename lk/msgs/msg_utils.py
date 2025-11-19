#!/usr/bin/env python3

"""
    Lightweight helpers shared across BAM message modules.
"""

# BAM

# PYTHON
from collections.abc import Sequence


# Reimplemented here to avoid importing bam.utils; keeps message modules lightweight.
def lerp_value(start: float | int, end: float | int, fraction: float) -> float:
    return float(start) + (float(end) - float(start)) * fraction


def lerp_list(
    start: Sequence[float] | None,
    end: Sequence[float] | None,
    fraction: float,
) -> Sequence[float] | None:
    if start is None or len(start) == 0:
        return end

    if end is None or len(end) == 0:
        return start

    if len(start) != len(end):  # type: ignore[arg-type]
        raise ValueError("lerp_list inputs must have the same length")

    return [lerp_value(float(s), float(e), fraction) for s, e in zip(start, end)]  # type: ignore[arg-type]


if __name__ == "__main__":
    print(lerp_value(0.0, 10.0, 0.5))
    print(lerp_list([0.0, 0.0], [10.0, 100.0], 0.25))
