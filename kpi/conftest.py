"""pytest wiring for every kpi suite: the private dora fixture and the timing tier.

Run a suite through kpi/run.py (it puts the suite in a private network and PID namespace, where the dora suites
may start their own coordinator and daemon), or the whole ruler through kpi/score.py.
"""

from __future__ import annotations

import pytest

from kpi.support.dora_private import private_dora  # noqa: F401  (a session fixture for the dora suites)
from kpi.support.lk import TIMING


def pytest_collection_modifyitems(config, items):
    skip_timing = pytest.mark.skip(reason="timing assertion: KPI_TIER=timing (rack unpowered or the fixed cloud VM)")
    for item in items:
        if "timing" in item.keywords and not TIMING:
            item.add_marker(skip_timing)
