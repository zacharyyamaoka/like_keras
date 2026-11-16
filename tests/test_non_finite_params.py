"""Lane core's test of CONTRACT §15 A33's Python side (consumer-016): a Python component whose Params hold NaN and
+-inf, the natural "NaN = off" and unbounded-limit defaults, crosses a process as its C++ twin does.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_non_finite_params.py

The ruler's test (facilities::test_non_finite_params_defaults) covers a native box's defaults. WHY this one: the
Python twin of that box was refused by describe() under dora ("cannot be rebuilt in another process"), because the
check that its params round-trip through JSON compared NaN with NaN, which is never equal. The dora case needs a
private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites).
"""

from __future__ import annotations

import dataclasses
import json
import math

import numpy as np
import pytest
from robot_msgs.sensor import JointState
from robot_msgs.std import Header

import like_keras as lk
from kpi.polyglot.friction_parts import JointStateLog
from kpi.support.lk import backend

NAMES = ("off", "upper", "lower", "gain")


class NonFinitePy(lk.Component):
    """kpi/facilities/non_finite.cpp in Python: its params, once, as a JointState (off, upper, lower, gain)."""

    echo = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        off: float = math.nan
        upper: float = math.inf
        lower: float = -math.inf
        gain: float = 0.5

    def forward(self):
        if getattr(self, "_sent", False):
            return None
        self._sent, params = True, self.params
        position = np.array([params.off, params.upper, params.lower, params.gain])
        return JointState(header=Header(stamp=self.now()), name=NAMES, position=position)


def same(found, expected) -> bool:
    return (found is None or math.isnan(found)) if math.isnan(expected) else found == expected


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_python_params_with_non_finite_values_cross_a_process(backend_name, tmp_path, request):
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    name = f"non_finite_py_{backend_name}"
    s = lk.System(name)
    box = s.add(NonFinitePy(gain=2.0), "box")
    s.add(JointStateLog(path=str(tmp_path / "echo.jsonl")), "sink", value=box.echo)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    shown = dep.describe()["components"][f"{name}.box"]["params"]
    expected = [math.nan, math.inf, -math.inf, 2.0]
    assert all(same(shown[key], want) for key, want in zip(NAMES, expected, strict=True)), shown
    try:
        dep.start(until=0.25)
    finally:
        dep.stop()
    rows = [json.loads(line) for line in (tmp_path / "echo.jsonl").read_text().splitlines()]
    assert len(rows) == 1, rows
    assert all(same(found, want) for found, want in zip(rows[0]["position"], expected, strict=True)), rows[0]
