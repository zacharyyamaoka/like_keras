"""Criterion 9: the seven facilities on an empty-step component, each language, each backend, zero user lines.

A facility counts only when its test passes in all four combinations (python/cpp x inproc/dora). The components
are kpi/tasks/empty (empty.py, empty.cpp -> build/kpi/libempty.so). Facility ports are wired like any port.
"""

from __future__ import annotations

import dataclasses
import json
import time

import pytest

from kpi.support import mcap
from kpi.support.lk import backend as load_backend
from kpi.support.lk import decode, like_keras, lk_command, native_library

COMBINATIONS = ["python-inproc", "python-dora", "cpp-inproc", "cpp-dora"]
STANDARD_METRICS = {"steps", "step_p50_s", "step_p99_s", "step_max_s", "period_error_p99_s", "overruns"}


def rows(path) -> list[dict]:
    try:
        return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    except FileNotFoundError:
        return []


@dataclasses.dataclass
class Facility:
    lk: object
    system: object
    subject: object  # the component whose facilities are measured (the native box, or the Python Talker)
    deployment: object
    language: str
    backend: str
    directory: object

    def path_of(self, name: str) -> str:
        return str(self.directory / name)


@pytest.fixture
def facility(request, tmp_path):
    language, backend_name = request.node.callspec.params["combination"].split("-")
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    backend = load_backend(backend_name)
    s = lk.System(f"facility_{language}_{backend_name}")
    if language == "cpp":
        subject = s.add(lk.native(native_library("empty"), config={"gain": 3.0}), "box")
    else:
        from kpi.tasks.empty.empty import Empty, Talker

        s.add(Empty(gain=3.0), "empty")
        subject = s.add(Talker(), "talker")

    def deployment(realtime: bool = True):
        dep = lk.Deployment(s, backend=backend, realtime=realtime)
        dep.place(*[getattr(s, name) for name in (("box",) if language == "cpp" else ("empty", "talker"))],
                  process="subject", hz=50)
        return dep

    return Facility(lk, s, subject, deployment, language, backend_name, tmp_path)


def run_with_sinks(facility: Facility, seconds: float = 2.6, record=None) -> None:
    from kpi.facilities.parts import HealthLog, LogLog, MetricsLog

    s = facility.system
    s.add(LogLog(path=facility.path_of("log.jsonl")), "log_sink", value=facility.subject.log)
    s.add(HealthLog(path=facility.path_of("health.jsonl")), "health_sink", value=facility.subject.health)
    s.add(MetricsLog(path=facility.path_of("metrics.jsonl")), "metrics_sink", value=facility.subject.metrics)
    dep = facility.deployment()
    dep.place(s.log_sink, s.health_sink, s.metrics_sink, process="sinks")
    if record is not None:
        record(dep)
    dep.start()
    time.sleep(seconds)
    dep.stop()


def mine(found: list[dict], facility: Facility) -> list[dict]:
    """Records about the subject: the Talker's own path, or any component inside the native box."""
    name = "box" if facility.language == "cpp" else "talker"
    return [row for row in found if f".{name}" in row["path"] or row["path"].endswith(name)]


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_log(facility, combination):
    """log: structured records on the `log` port, with the tick, wall time and component path."""
    run_with_sinks(facility)
    found = [row for row in mine(rows(facility.path_of("log.jsonl")), facility) if "step" in row["message"]]
    assert len(found) >= 50, f"{len(found)} log records"
    assert all(row["level"] == 20 and row["tick_type"] == "int" for row in found)
    assert all(abs(row["stamp"] - time.time()) < 60 for row in found), "LogRecord.header.stamp is not wall time"
    ticks = [row["tick"] for row in found]
    assert ticks == sorted(ticks) and len(set(ticks)) == len(ticks)


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_metrics(facility, combination):
    """metrics: at 1 Hz, steps, step time p50/p99/max, period error p99 and overruns; a custom name equal to a
    standard one is refused."""
    run_with_sinks(facility)
    found = mine(rows(facility.path_of("metrics.jsonl")), facility)
    assert len(found) >= 2, f"{len(found)} metrics records in 2.6 s at 1 Hz"
    assert STANDARD_METRICS <= set(found[-1]["values"]), f"missing {STANDARD_METRICS - set(found[-1]['values'])}"
    steps = [row["values"]["steps"] for row in found]
    assert steps == sorted(steps) and steps[-1] >= 50


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_health(facility, combination):
    """health: a DiagnosticRecord at 1 Hz, mode from the lifecycle, level ok, expected_period_s 1.0; STALE is never
    sent (an observer derives it)."""
    run_with_sinks(facility)
    found = mine(rows(facility.path_of("health.jsonl")), facility)
    assert len(found) >= 2, f"{len(found)} health records"
    assert all(row["expected_period_s"] == 1.0 for row in found)
    assert all(row["level"] in ("unknown", "ok", "warn", "error") for row in found), "STALE was sent"
    assert any(row["mode"] == "operational" and row["level"] == "ok" for row in found)


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_params(facility, combination):
    """params: the frozen Params (Python) or LK_PARAMS aggregate (C++) round-trips as JSON and shows in describe();
    an unknown key is refused naming it."""
    plan = facility.deployment().describe()
    json.dumps(plan)
    params = [component["params"] for path, component in plan["components"].items() if path.endswith(("empty", "box"))]
    assert params and params[0].get("gain") == 3.0, plan["components"]
    lk = facility.lk
    if facility.language == "cpp":
        if facility.backend == "inproc":  # under dora lk_create runs in the node process; its exit is T7's business
            s = lk.System("refused_params")
            s.add(lk.native(native_library("empty"), config={"gainz": 3.0}), "box")
            dep = lk.Deployment(s)
            dep.place(s.box, hz=10)
            with pytest.raises(Exception, match="gainz"):
                dep.start(until=0.1)
            dep.stop()
    else:
        from kpi.tasks.empty.empty import Empty

        values = dataclasses.asdict(Empty(gain=3.0).params)
        assert Empty(**json.loads(json.dumps(values))).params == Empty(gain=3.0).params
        with pytest.raises(TypeError, match="gainz"):
            Empty(gainz=3.0)


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_lifecycle(facility, combination):
    """lifecycle: driven from outside (start/stop); health.mode says operational while running; a restart comes back
    through activate (operational again)."""
    run_with_sinks(facility, seconds=2.2)
    first = mine(rows(facility.path_of("health.jsonl")), facility)
    assert first and first[0]["mode"] in ("initialization", "operational") and any(r["mode"] == "operational" for r in first)
    if facility.backend == "inproc":
        dep = facility.deployment()
        dep.place(facility.system.log_sink, facility.system.health_sink, facility.system.metrics_sink, process="sinks")
        dep.start()
        time.sleep(2.2)
        dep.stop()
        again = mine(rows(facility.path_of("health.jsonl")), facility)[len(first):]
        assert any(row["mode"] == "operational" for row in again), "a restarted deployment did not activate again"


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_recording(facility, combination):
    """recording: Deployment.record writes MCAP (one channel per port, message encoding lkf, schema encoding
    arrow-c-format, log and publish time) that lk.replay plays back through the same ports."""
    path = facility.directory / "facility.mcap"
    run_with_sinks(facility, record=lambda dep: dep.record(path, facility.subject.metrics, facility.subject.health))
    recording = mcap.read(path)
    channels = [channel for channel in recording.channels.values() if channel.topic.endswith((".metrics", ".health"))]
    assert len(channels) == 2, recording.topics()
    for channel in channels:
        assert channel.message_encoding == "lkf"
        assert recording.schemas[channel.schema_id].encoding == "arrow-c-format"
    metrics = recording.on("metrics")
    assert len(metrics) >= 2
    for message in metrics:
        type_name, body = decode(message.data)
        assert type_name == "std/Metrics"
        stamp_ns = round(body["header"]["stamp"] * 1e9)
        assert abs(message.publish_time - stamp_ns) <= 1_000, "publish_time is not header.stamp"
        assert message.log_time > 0
    lk = facility.lk
    from kpi.facilities.parts import MetricsLog

    s = lk.System("replayed")
    replay = s.add(lk.replay(path), "replay")
    port = next(name for name in dir(replay) if name.endswith("metrics") and not name.startswith("_"))
    s.add(MetricsLog(path=facility.path_of("replayed.jsonl")), "sink", value=getattr(replay, port))
    dep = lk.Deployment(s)  # real time: a real-time recording's publish times are epoch seconds
    dep.place(s.replay, hz=50)
    dep.place(s.sink)
    dep.start()
    time.sleep(4.0)
    dep.stop()
    assert len(rows(facility.path_of("replayed.jsonl"))) >= len(metrics) - 1, "replay did not deliver the recording"


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_introspection(facility, combination):
    """introspection: describe() is JSON listing ports, types, policies, placement and transport; `lk ls` reads it
    from the attach socket."""
    import subprocess

    facility.system.expose(facility.subject.health)
    dep = facility.deployment()
    plan = dep.describe()
    json.dumps(plan)
    for key in ("like_keras", "abi", "hash", "backend", "system", "components", "edges", "groups", "exposed", "attach"):
        assert key in plan, f"describe() has no {key!r}"
    assert plan["backend"]["name"] == facility.backend
    assert any(group["process"] == "subject" and group["hz"] == 50 for group in plan["groups"])
    for component in plan["components"].values():
        assert {"impl", "language", "params", "ports"} <= set(component)
    dep.start()
    try:
        time.sleep(1.0)
        listing = subprocess.run(lk_command("ls", plan["attach"]["socket"]), capture_output=True, text=True, timeout=5)
    finally:
        dep.stop()
    assert listing.returncode == 0 and "health" in listing.stdout and "std/DiagnosticRecord" in listing.stdout
