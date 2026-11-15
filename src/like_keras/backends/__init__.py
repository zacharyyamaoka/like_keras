"""Backends (CONTRACT §9): `inproc` (one process, many threads) and the transport backends beside it.

A backend module declares what it can honour as data (NAME, VERSION, CAPABILITIES) and runs a checked plan
(`start(plan, until)` -> a run with `stop()`); the core refuses what it cannot do, it never degrades silently.

WHY backends load on first use: a deployment file names its backend at the top (`from like_keras.backends import
<name>`), and the all-Python fallback one line away in the same file must run where that backend's transport is
not installed (the lean core: robot_msgs is the one runtime dependency).
"""

from __future__ import annotations

import importlib.util
import os
import sys


def __getattr__(name: str):
    if name.startswith("_"):
        raise AttributeError(name)
    qualified = f"{__name__}.{name}"
    module = sys.modules.get(qualified)  # imported already: the same module object, never a second copy of its state
    if module is None:
        # WHY importlib.util.find_spec, not PathFinder: it walks sys.meta_path, so an import hook or blocker sees
        # the backend like any other import (kpi/blackbox blocks one; a blocker's ImportError propagates as is).
        spec = importlib.util.find_spec(qualified)
        if spec is None:
            raise AttributeError(f"like_keras has no backend {name!r}")
        spec.loader = importlib.util.LazyLoader(spec.loader)
        module = importlib.util.module_from_spec(spec)
        sys.modules[qualified] = module
        spec.loader.exec_module(module)  # lazy: the module body runs at its first attribute access
    globals()[name] = module
    return module


def address(system: str) -> str:
    """The attach socket of a running deployment (CONTRACT §9.3): explicit, printed by describe(), never discovered."""
    return f"unix:{os.environ.get('XDG_RUNTIME_DIR') or '/tmp'}/like_keras/{system}.sock"
