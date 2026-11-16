"""Lane cpp's own tests of a native box's facilities through inproc (CONTRACT §5.5, §5.6, §8), beside the ruler's
kpi/facilities, which deploys only in real time (docs/friction/ruler-005.md proposes its lockstep half).

    cmake -S /home/bam/like_keras -B /home/bam/like_keras/build -DCMAKE_BUILD_TYPE=Release
    nice -n 19 cmake --build /home/bam/like_keras/build -j 2
    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests

WHY (fix round 1, M2): under lockstep a native box sent no health at all. Its uptime subtracted a wall-clock
activation stamp (epoch s) from the logical step time, so every record said uptime_s ~ -1.79e9 s, robot_msgs refused
it and inproc dropped it. Every stamp the box takes now comes from one clock, its own step clock (Box::clock()).
"""

from __future__ import annotations

import logging
import math
import pathlib

import pytest
from robot_msgs.std import DiagnosticRecord, Metrics

import like_keras as lk

LIBRARY = pathlib.Path(__file__).resolve().parents[1] / "build" / "kpi" / "libempty.so"
UNTIL_S = 3.5  # logical seconds: the facilities publish at 0, 1, 2 and 3


class Keep(lk.Component):
    """A sink that keeps every record it is sent, in memory (inproc runs it in this process)."""

    value = lk.In(DiagnosticRecord, queue=1000)

    def activate(self) -> None:
        self.rows: list = []

    def forward(self, value):
        self.rows.append(value)


class KeepMetrics(Keep):
    value = lk.In(Metrics, queue=1000)


@pytest.fixture
def refusals():
    """What like_keras logs at error level while the test runs (inproc reports a refused record there once)."""
    seen: list[str] = []

    class Catch(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            seen.append(record.getMessage())

    handler = Catch(level=logging.ERROR)
    logging.getLogger("like_keras").addHandler(handler)
    yield seen
    logging.getLogger("like_keras").removeHandler(handler)


def deploy(language: str, realtime: bool):
    if not LIBRARY.exists():
        raise FileNotFoundError(f"{LIBRARY} is not built: run the cmake commands in this file's docstring")
    from kpi.tasks.empty.empty import Empty

    s = lk.System(f"native_facilities_{language}")
    subject = s.add(lk.native(LIBRARY, config={"gain": 3.0}) if language == "cpp" else Empty(gain=3.0), "box")
    health = s.add(Keep(), "health_sink", value=subject.health)
    metrics = s.add(KeepMetrics(), "metrics_sink", value=subject.metrics)
    dep = lk.Deployment(s, realtime=realtime)
    dep.place(s.box, process="subject", hz=50)
    dep.place(s.health_sink, s.metrics_sink, process="sinks")
    return dep, health, metrics


def run(language: str, realtime: bool):
    dep, health, metrics = deploy(language, realtime)
    dep.start(until=2.6 if realtime else UNTIL_S)  # real time: wall seconds, as kpi/facilities runs it
    dep.stop()
    return health.rows, metrics.rows


@pytest.mark.parametrize("language", ["python", "cpp"])
def test_lockstep_health_is_delivered(language, refusals):
    """Under lockstep a native box delivers a health record per component per logical second, as a Python component
    does, and robot_msgs refuses none of them."""
    health, _ = run(language, realtime=False)
    per_component = 4  # publishes at logical 0, 1, 2 and 3 (UNTIL_S = 3.5)
    components = 2 if language == "cpp" else 1  # the box's empty and talker; the Python Empty alone
    assert not [text for text in refusals if "invalid" in text], refusals
    assert len(health) == per_component * components, [(row.path, row.header.stamp) for row in health]
    assert all(row.mode == "operational" for row in health)


def test_lockstep_uptime_is_on_the_step_clock(refusals):
    """§5.5: logical time starts at 0.0, where the box activated; so uptime_s IS the record's logical stamp, and it
    never goes negative. The facilities publish before the box's step at the same time, so the stamp is the last
    step's time (0.98 at the publish of 1.0, at 50 Hz)."""
    health, metrics = run("cpp", realtime=False)
    assert health, refusals
    stamps = sorted({row.header.stamp for row in health})
    assert stamps == pytest.approx([0.0, 0.98, 1.98, 2.98]), stamps
    for row in health:
        assert math.isfinite(row.uptime_s) and row.uptime_s >= 0.0, (row.path, row.uptime_s)
        assert row.uptime_s == pytest.approx(row.header.stamp, abs=1e-9), (row.path, row.uptime_s, row.header.stamp)
    assert all(0.0 <= row.header.stamp < UNTIL_S for row in metrics), [row.header.stamp for row in metrics]


def test_realtime_uptime_is_on_the_box_clock(refusals):
    """In real time the box's clock is epoch seconds: uptime counts from activation, small and non-negative."""
    health, _ = run("cpp", realtime=True)
    assert not [text for text in refusals if "invalid" in text], refusals
    operational = [row for row in health if row.mode == "operational"]
    assert len(operational) >= 4, [(row.path, row.mode) for row in health]  # two components, at least two seconds
    assert all(0.0 <= row.uptime_s < 10.0 for row in operational), [row.uptime_s for row in operational]
    assert all(abs(row.header.stamp - operational[0].header.stamp) < 10.0 for row in operational)
