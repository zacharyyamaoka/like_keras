"""pytest wiring for every kpi suite: the private dora fixture, the timing tier and the backend switch.

Run a suite through kpi/run.py (it puts the suite in a private network and PID namespace, where the dora suites
may start their own coordinator and daemon), or the whole ruler through kpi/score.py.

`--backend NAME` (or LK_BACKEND=NAME; a comma list, or the flag repeated) runs only the tests of those backends;
the default is every backend the ruler scores. What a test names, and the rule: kpi/support/backends.py.
"""

from __future__ import annotations

import os

import pytest

from kpi.support import backends
from kpi.support.dora_private import private_dora  # noqa: F401  (a session fixture for the dora suites)
from kpi.support.lk import TIMING

# WHY a guard: a parameter on one of these axes, or one whose value is a backend or a placement, must say its backend
# (kpi/support/backends.py's helpers), or a new backend would silently skip past the switch.
AXES = {"backend", "backend_name", "placement", "combination"}
PLACEMENTS = {"target", "same_box", "fallback", "attached"}


def pytest_addoption(parser):
    parser.addoption("--backend", action="append", default=[], metavar="NAME",
                     help=f"test only these like_keras backends (comma list or repeated; default: every backend the "
                          f"ruler scores, {','.join(backends.RULER)}; else ${backends.ENVIRONMENT})")


def pytest_configure(config):
    config.addinivalue_line("markers", "backend(*names): the like_keras backends this test or parametrization runs "
                                       "on; --backend / LK_BACKEND keeps it only when every one is selected")
    flag = config.getoption("--backend")
    try:
        backends.select(flag or [os.environ.get(backends.ENVIRONMENT, "")])
    except ValueError as error:
        raise pytest.UsageError(f"{'--backend' if flag else backends.ENVIRONMENT}: {error}") from None


def _untagged(item) -> bool:
    params = getattr(getattr(item, "callspec", None), "params", {})
    axis = any(name in AXES or (isinstance(value, str) and (value in backends.known() or value in PLACEMENTS))
               for name, value in params.items())
    return axis and not backends.names_of(item)


def pytest_collection_modifyitems(config, items):
    untagged = [item.nodeid for item in items if _untagged(item)]
    if untagged:
        raise pytest.UsageError("a backend or placement parametrization without its backend (use "
                                "kpi/support/backends.py's helpers): " + ", ".join(untagged))
    chosen = set(backends.selected())
    dropped = [item for item in items if not backends.names_of(item) <= chosen]
    if dropped:
        config.hook.pytest_deselected(items=dropped)
        items[:] = [item for item in items if backends.names_of(item) <= chosen]
    skip_timing = pytest.mark.skip(reason="timing assertion: KPI_TIER=timing (rack unpowered or the fixed cloud VM)")
    for item in items:
        if "timing" in item.keywords and not TIMING:
            item.add_marker(skip_timing)
