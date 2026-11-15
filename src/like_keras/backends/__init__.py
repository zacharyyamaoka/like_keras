"""Backends (CONTRACT §9): `inproc` (one process, many threads) and the transport backends beside it.

A backend module declares what it can honour as data (NAME, VERSION, CAPABILITIES) and runs a checked plan
(`start(plan, until)` -> a run with `stop()`); the core refuses what it cannot do, it never degrades silently.

WHY backends load on first use: a deployment file names its backend at the top (`from like_keras.backends import
<name>`), and the all-Python fallback one line away in the same file must run where that backend's transport is
not installed (the lean core: robot_msgs is the one runtime dependency).
"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import os
import sys


def __getattr__(name: str):
    if name.startswith("_"):
        raise AttributeError(name)
    spec = importlib.machinery.PathFinder.find_spec(f"{__name__}.{name}", __path__)
    if spec is None:
        raise AttributeError(f"like_keras has no backend {name!r}")
    spec.loader = importlib.util.LazyLoader(spec.loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)  # lazy: the module body runs at its first attribute access
    globals()[name] = module
    return module


def address(system: str) -> str:
    """The attach socket of a running deployment (CONTRACT §9.3): explicit, printed by describe(), never discovered."""
    return f"unix:{os.environ.get('XDG_RUNTIME_DIR') or '/tmp'}/like_keras/{system}.sock"
