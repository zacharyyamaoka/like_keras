"""Lane cpp's own tests of a native box's facilities through inproc (CONTRACT §5.5, §5.6, §8), beside the ruler's
kpi/facilities, which deploys only in real time (docs/friction/ruler-005.md proposes its lockstep half).

    cmake -S /home/bam/like_keras -B /home/bam/like_keras/build -DCMAKE_BUILD_TYPE=Release
    nice -n 19 cmake --build /home/bam/like_keras/build -j 2
    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests

WHY (fix round 1, M2): under lockstep a native box sent no health at all. Its uptime subtracted a wall-clock
activation stamp (epoch s) from the logical step time, so every record said uptime_s ~ -1.79e9 s, robot_msgs refused
it and inproc dropped it. Every stamp the box takes now comes from one clock, its own step clock (Box::clock()).

WHY (CONTRACT §15 A10): that clock is the last step's time (0.98 s at the facility tick 1.0, at 50 Hz), where a
Python component is stamped at the tick, so a Python <-> C++ swap shifted every facility stamp by one period. Under
lockstep the host now stamps a native box's health and metrics at the facility tick, uptime_s shifted with it: inproc
here, lk-dora-host (cpp/hosts) under dora, which the dora half of the swap test runs (private dora: run it as
`python kpi/run.py`'s namespace does, `bwrap --unshare-net --unshare-pid`, or it skips).
"""

from __future__ import annotations

import json
import logging
import math
import pathlib

import pytest
from robot_msgs.std import DiagnosticRecord, Metrics

import like_keras as lk
from kpi.support.dora_private import private_dora  # noqa: F401  (the dora half's session fixture)

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


def test_lockstep_stamps_are_the_facility_tick(refusals):
    """§5.5 and A10: logical time starts at 0.0, where the box activated, and every health and metrics record is
    stamped at its facility tick (0, 1, 2, 3), never at the box's last step before it (0.98 at 50 Hz); uptime_s IS
    the record's stamp, and it never goes negative."""
    health, metrics = run("cpp", realtime=False)
    assert health, refusals
    assert sorted({row.header.stamp for row in health}) == [0.0, 1.0, 2.0, 3.0], [row.header.stamp for row in health]
    for row in health:
        assert math.isfinite(row.uptime_s) and row.uptime_s >= 0.0, (row.path, row.uptime_s)
        assert row.uptime_s == pytest.approx(row.header.stamp, abs=1e-9), (row.path, row.uptime_s, row.header.stamp)
    assert sorted({row.header.stamp for row in metrics}) == [0.0, 1.0, 2.0, 3.0], [row.header.stamp for row in metrics]


def stamps(health, metrics) -> dict:
    """What a swap must not change, per facility: each record's stamp and uptime, whatever component it names (the
    C++ box has two, Empty and Talker, where the Python subject is one Empty)."""
    return {
        "health": sorted({(row.header.stamp, row.uptime_s, row.mode, row.level) for row in health}),
        "metrics": sorted({row.header.stamp for row in metrics}),
    }


def test_a_python_cpp_swap_changes_no_facility_stamp_inproc(refusals):
    python, cpp = stamps(*run("python", realtime=False)), stamps(*run("cpp", realtime=False))
    assert python["health"] == [(float(tick), float(tick), "operational", "ok") for tick in range(4)], python
    assert cpp == python


def dora_stamps(language: str, directory: pathlib.Path) -> dict:
    """The same deployment as `deploy`, on dora under lockstep: the subject in its own process (lk-dora-host for the
    .so), the sinks writing JSON lines in theirs."""
    from kpi.facilities.parts import HealthLog, MetricsLog
    from kpi.tasks.empty.empty import Empty
    from like_keras.backends import dora

    s = lk.System(f"native_stamps_{language}")
    subject = s.add(lk.native(LIBRARY, config={"gain": 3.0}) if language == "cpp" else Empty(gain=3.0), "box")
    s.add(HealthLog(path=str(directory / f"{language}.health.jsonl")), "health_sink", value=subject.health)
    s.add(MetricsLog(path=str(directory / f"{language}.metrics.jsonl")), "metrics_sink", value=subject.metrics)
    dep = lk.Deployment(s, backend=dora, realtime=False)
    dep.place(s.box, process="subject", hz=50)
    dep.place(s.health_sink, s.metrics_sink, process="sinks")
    dep.start(until=UNTIL_S)
    dep.stop()
    health, metrics = (
        [json.loads(line) for line in (directory / f"{language}.{port}.jsonl").read_text().splitlines()]
        for port in ("health", "metrics")
    )
    return {
        "health": sorted({(row["stamp"], row["uptime_s"], row["mode"], row["level"]) for row in health}),
        "metrics": sorted({row["stamp"] for row in metrics}),
    }


def test_a_python_cpp_swap_changes_no_facility_stamp_dora(request, tmp_path):
    """A10 in lk-dora-host: the C++ host re-stamps as inproc does, so the dora swap matches the inproc one."""
    host = LIBRARY.parents[1] / "hosts" / "lk-dora-host"
    if not host.exists():
        pytest.skip(f"{host} is not built: cpp/hosts/build_dora_cxx.sh")
    request.getfixturevalue("private_dora")
    python, cpp = dora_stamps("python", tmp_path), dora_stamps("cpp", tmp_path)
    assert python["health"] == [(float(tick), float(tick), "operational", "ok") for tick in range(4)], python
    assert python["metrics"] == [0.0, 1.0, 2.0, 3.0], python
    assert cpp == python


def test_realtime_uptime_is_on_the_box_clock(refusals):
    """In real time the box's clock is epoch seconds: uptime counts from activation, small and non-negative."""
    health, _ = run("cpp", realtime=True)
    assert not [text for text in refusals if "invalid" in text], refusals
    operational = [row for row in health if row.mode == "operational"]
    assert len(operational) >= 4, [(row.path, row.mode) for row in health]  # two components, at least two seconds
    assert all(0.0 <= row.uptime_s < 10.0 for row in operational), [row.uptime_s for row in operational]
    assert all(abs(row.header.stamp - operational[0].header.stamp) < 10.0 for row in operational)
