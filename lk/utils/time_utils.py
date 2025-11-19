#!/usr/bin/env python3

"""
    Simple stopwatch for wall-clock profiling.
"""

# PYTHON
import time


class StopWatch:
    """Lightweight stopwatch supporting multiple named timers."""

    def __init__(self, verbose: bool = True) -> None:
        self.verbose = verbose
        self.start_times: dict[str, float] = {}

    def start(self, label: str = "default") -> None:
        self.start_times[label] = time.time()
        if self.verbose:
            print(f"[START] {label}")

    def stop(self, label: str = "default") -> float:
        if label not in self.start_times:
            raise ValueError(f"No start time recorded for label '{label}'")
        elapsed = time.time() - self.start_times[label]
        if self.verbose:
            print(f"[STOP] {label}: {elapsed:.3f}s")
        del self.start_times[label]
        return elapsed


__all__ = ["StopWatch"]

