"""Criterion 9 (facilities), ruler M0.3 (CONTRACT §15 A27): the first consumer's friction with the facilities.

Each test judges one decision of §15 and counts with the facility kpi/score.py already scores (a facility counts only
when every one of its tests passes):
  A18 test_sigterm_runs_deactivate       lifecycle: SIGTERM and SIGHUP end a deployment the way SIGINT does
  A21 test_system_facility_ports         metrics: a Python System's log/health/metrics carry every inner component
  A22 test_health_set_by_the_component   health: health(level, code), held; log.error -> error / ErrorLogged
  A24 test_vector_params                 params: LK_PARAMS reflects vectors and vectors of rows
Ruler M0.4 adds two, from the library verifier's round 1:
  A28 test_dora_stop_drains_to_live_readers      lifecycle: a plain real-time dora stop() delivers the final drain
  A22 test_health_tie_keeps_the_facility_message health: on a tie the component's code, the facility's message
The C++ boxes (hand_back.cpp, self_health.cpp, table_params.cpp, tie_health.cpp) are built on demand
(kpi/support/lk.feature_library): until a decision lands, its box does not compile, or the library does not do it,
and its test FAILS, never skips.
"""

from __future__ import annotations

import ctypes
import json
import os
import signal
import subprocess
import threading
import time

import pytest

from kpi.support import cells, procs
from kpi.support.lk import (backend, dora_backend, feature_library, kpi_cpus, like_keras, native_library, recorded,
                            subprocess_env)


def rows(path) -> list[dict]:
    try:
        return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    except FileNotFoundError:
        return []


# ---------------------------------------------------------------- A18 (consumer-003): lifecycle

SIGNAL_CASES = ["py-inproc-SIGTERM", "py-inproc-SIGHUP", "cpp-inproc-SIGTERM", "cpp-inproc-SIGHUP",
                "cpp-dora-SIGTERM", "cpp-dora-node-SIGTERM"]


@pytest.mark.parametrize("case", SIGNAL_CASES)
def test_sigterm_runs_deactivate(case, tmp_path, request):
    """CONTRACT §15 A18: SIGTERM and SIGHUP end a deployment the way SIGINT does. A process written as the contract's
    file 3 (§4.6: start, signal.pause() in try/finally: stop; no handler of its own, kpi/facilities/hand_back_child.py)
    runs a box whose deactivate writes a file. The signal goes to that process (same_box: Python and C++; target:
    the launcher, whose stop() stops the dataflow), or straight to the box's lk-dora-host (`node`, taken on its
    sigwait thread): the file appears. In one process, deactivate's log line also reaches the recording (the
    facility ports are drained once more after the last deactivate). WHY (consumer-003): systemctl stop, kill and a
    dora stop of a Python node send SIGTERM, which ended a same_box process with no deactivate: no hand-back."""
    language, backend_name, *rest = case.split("-")
    signal_number, to_node = getattr(signal, rest[-1]), "node" in rest
    like_keras()
    dora = request.getfixturevalue("private_dora") if backend_name == "dora" else None
    library = ["--library", str(feature_library("hand_back"))] if language == "cpp" else []
    name = f"hand_back_{SIGNAL_CASES.index(case)}_{os.getpid()}"
    handed_back = tmp_path / "handed_back"
    child = procs.Child("kpi.facilities.hand_back_child", "--language", language, "--backend", backend_name,
                        "--directory", str(tmp_path), "--name", name, *library, log=tmp_path / "child.log")
    try:
        child.wait_ready(timeout_s=60.0)
        time.sleep(1.0)  # the box steps a while before the signal
        if to_node:
            node = procs.wait_for(lambda: next(iter(procs.pids_with("lk-dora-host", name)), None), 10.0,
                                  "the box's lk-dora-host")
            os.kill(node, signal_number)
            procs.wait_for(handed_back.exists, 15.0, "deactivate's file after a SIGTERM to lk-dora-host")
        else:
            child.signal(signal_number)
            child.process.wait(timeout=30)
            procs.wait_for(handed_back.exists, 15.0, "deactivate's file after the signal")
    finally:
        child.signal(signal.SIGINT)  # what ends it today; a no-op once it has exited
        try:
            child.process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            child.stop()
        if dora is not None:  # a launcher killed by the signal leaves its dataflow running: stop it by name
            subprocess.run([dora.cli, "stop", "--name", name], cwd=dora.workdir, env=subprocess_env(),
                           capture_output=True, timeout=60)
    assert handed_back.read_text() == "handed back\n"
    if backend_name == "inproc":
        lines = [row["message"] for row in recorded(tmp_path / "run.mcap", "log")]
        assert any("handed back" in line for line in lines), f"deactivate's log line was not drained: {lines}"


# ---------------------------------------------------------------- A21 (consumer-006): metrics

CHILDREN = {"cell.arm.bus", "cell.arm.jtc", "cell.arm.guard"}


def _arm_facilities(placement: str, directory) -> dict[str, list[dict]]:
    """The reference cell with kpi/consumer/parts.py's StallingPlanner (its third plan runs away, so the guard logs
    a runaway hold), lockstep for 4 s, with the arm's own log, metrics and health ports wired to sinks the same way
    in both placements: the C++ chain (same_box) and the Python one (fallback)."""
    lk = like_keras()
    from kpi.consumer.parts import StallingPlanner
    from kpi.facilities.parts import HealthLog, LogLog, MetricsLog
    from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy
    from kpi.tasks.arm_cell.targets import TargetFile

    directory.mkdir(parents=True, exist_ok=True)
    config = cells.config(directory, lockstep=True)
    if placement == "fallback":
        arm = ArmControllerPy(config=config)
    else:
        arm = lk.native(native_library("arm_controller"), config=config)
    s = lk.System("cell")
    targets = s.add(TargetFile(), "targets")
    s.add(arm, "arm")
    planner = s.add(StallingPlanner(stall_on=2), "planner", target=targets.target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    # WHY no expose: the System's own facility ports, read the same way whatever the arm is made of (A21)
    s.add(LogLog(path=str(directory / "log.jsonl")), "log_sink", value=arm.log)
    s.add(MetricsLog(path=str(directory / "metrics.jsonl")), "metrics_sink", value=arm.metrics)
    s.add(HealthLog(path=str(directory / "health.jsonl")), "health_sink", value=arm.health)
    dep = lk.Deployment(s, realtime=False)
    dep.place(s.arm, process="arm-controller", hz=100, cpus=kpi_cpus(), fifo=80)
    dep.place(s.targets, hz=2)
    dep.place(s.planner)
    dep.place(s.log_sink, s.metrics_sink, s.health_sink, process="sinks")
    try:
        dep.start(until=4.0)
    finally:
        dep.stop()
    return {port: rows(directory / f"{port}.jsonl") for port in ("log", "metrics", "health")}


def test_system_facility_ports(tmp_path):
    """CONTRACT §15 A21: a Python lk.System has log, health and metrics ports that carry every inner component's
    records with their full paths, as a native box's do. The reference arm, wired by `arm.log`, `arm.metrics` and
    `arm.health` in same_box (lk.native) and fallback (ArmControllerPy): the same inner paths on metrics and on
    health (cell.arm.bus, .jtc, .guard), and the guard's runaway line on log, in both. WHY (consumer-006): the swap
    changed how the chain's facilities were read; every reader branched on the placement (`expose(guard_log=...)`)."""
    runs = {placement: _arm_facilities(placement, tmp_path / placement) for placement in ("same_box", "fallback")}
    for port in ("metrics", "health"):
        inner = {placement: {row["path"] for row in run[port] if row["path"].startswith("cell.arm.")}
                 for placement, run in runs.items()}
        assert inner["fallback"] == inner["same_box"] == CHILDREN, f"arm.{port} paths: {inner}"
    for placement, run in runs.items():
        runaway = [row for row in run["log"] if row["path"] == "cell.arm.guard" and row["level"] >= 40
                   and "runaway" in row["message"]]
        assert runaway, f"{placement}: arm.log did not carry the guard's runaway line: {run['log'][:5]}"


# ---------------------------------------------------------------- A22 (consumer-007): health

HEALTH_ROUNDS = {1.0: ("error", "Refused"), 2.0: ("error", "Refused"), 3.0: ("warn", "Degraded"),
                 4.0: ("warn", "Degraded"), 5.0: ("warn", "Degraded"), 6.0: ("ok", ""), 7.0: ("ok", ""),
                 8.0: ("error", "ErrorLogged")}


@pytest.mark.parametrize("combination", ["python-inproc", "cpp-inproc"])
def test_health_set_by_the_component(combination, tmp_path):
    """CONTRACT §15 A22, in both languages, lockstep (health rounds at 1 s, 2 s, ...): a component's
    health(level, code) is held until it is called again (rounds 1-2 error / Refused, 3-5 warn / Degraded, 6-7 ok);
    log.error sets health error / ErrorLogged (round 8: the worse of the component's ok and the log's error); the
    C++ Log takes a string-literal %s argument. WHY (consumer-007): a refused or braked chain read warn for one
    second, then ok, and a C++ component could neither say it was in error nor name why in its log."""
    lk = like_keras()
    from kpi.facilities.parts import HealthLog, LogLog

    s = lk.System("self_health")
    if combination.startswith("cpp"):
        subject = s.add(lk.native(feature_library("self_health"), config={"unused": 0}), "box")
    else:
        from kpi.facilities.friction_parts import SelfHealth

        subject = s.add(SelfHealth(), "subject")
    s.add(HealthLog(path=str(tmp_path / "health.jsonl")), "health_sink", value=subject.health)
    s.add(LogLog(path=str(tmp_path / "log.jsonl")), "log_sink", value=subject.log)
    dep = lk.Deployment(s, realtime=False)
    dep.place(subject, process="subject", hz=50)
    dep.place(s.health_sink, s.log_sink, process="sinks")
    try:
        dep.start(until=8.6)
    finally:
        dep.stop()
    found = {round(row["stamp"], 6): (row["level"], row["code"]) for row in rows(tmp_path / "health.jsonl")
             if row["path"].endswith("subject") and row["mode"] == "operational"}
    wrong = {stamp: (found.get(stamp), want) for stamp, want in HEALTH_ROUNDS.items() if found.get(stamp) != want}
    assert not wrong, f"health round: (sent, expected): {wrong}"
    errors = [row["message"] for row in rows(tmp_path / "log.jsonl") if row["path"].endswith("subject")
              and row["level"] == 40]
    assert errors == ["refused: StartTooFar"], errors


# ---------------------------------------------------------------- A24 (consumer-009): params

ROWS = [{"name": "shoulder", "value": 1.5}, {"name": "elbow", "value": -2.0}, {"name": "wrist", "value": 0.25}]
GAINS = [10.0, 20.0, 30.0]


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_vector_params(backend_name, tmp_path, request):
    """CONTRACT §15 A24: LK_PARAMS reflects std::vector<double> and std::vector<Row> of an LK_PARAMS aggregate,
    filled at lk_create: lk_describe() shows their defaults as [], describe() shows every row, the box receives every
    row and gain, and an unknown key inside a row is refused naming rows[2].<key>. WHY (consumer-009): the
    controller's per-joint table could only be a file path, so the datasheet showed a path, not the joint table."""
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    chosen = backend(backend_name)
    library = feature_library("table_params")
    from kpi.polyglot.friction_parts import JointStateLog

    so = ctypes.CDLL(str(library))
    so.lk_describe.restype = ctypes.c_char_p
    defaults = json.loads(so.lk_describe())["params"]
    assert defaults == {"rows": [], "gains": []}, defaults
    s = lk.System(f"table_{backend_name}")
    box = s.add(lk.native(library, config={"rows": ROWS, "gains": GAINS}), "box")
    s.add(JointStateLog(path=str(tmp_path / "echo.jsonl")), "sink", value=box.echo)
    dep = lk.Deployment(s, backend=chosen, realtime=False)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    shown = next(component["params"] for path, component in dep.describe()["components"].items()
                 if path.endswith(".box"))
    assert shown == {"rows": ROWS, "gains": GAINS}, shown
    try:
        dep.start(until=0.45)
    finally:
        dep.stop()
    echoed = rows(tmp_path / "echo.jsonl")
    assert len(echoed) == 1, echoed
    assert echoed[0]["name"] == [row["name"] for row in ROWS], echoed
    assert echoed[0]["position"] == [row["value"] for row in ROWS] and echoed[0]["velocity"] == GAINS, echoed
    if backend_name == "inproc":  # under dora lk_create runs in the node process; its exit is T7's business
        typo = [*ROWS[:2], {"name": "wrist", "valu": 0.25}]
        s = lk.System("table_refused")
        s.add(lk.native(library, config={"rows": typo, "gains": GAINS}), "box")
        dep = lk.Deployment(s, realtime=False)
        dep.place(s.box, hz=10)
        with pytest.raises(Exception) as refused:
            dep.start(until=0.1)
        dep.stop()
        assert "rows[2].valu" in str(refused.value), str(refused.value)


# ---------------------------------------------------------------- A28 under dora (M0.4; library verifier, round 1)


def test_dora_stop_drains_to_live_readers(tmp_path, request):
    """CONTRACT §15 A28 as A30 orders it: every stop of a native box drains its log, health and metrics once more
    after the last deactivate(), and a real-time dora stop() signals the native nodes first, so that drain reaches
    live readers. hand_back.cpp (its deactivate logs "handed back") in its own process at 20 Hz, real time, with its
    log and health wired to reader components in another process and recorded; once the readers hold a health record
    of the running box, a plain dep.stop() (no signal from the test). Then: deactivate ran once; the reader and the
    recording each hold the "handed back" line once, and the box's last health record says offline, with nothing
    operational after it; and the box's lk-dora-host kept one pid from start to stop (dora never restarted it).
    WHY (library verifier, round 1): `dora stop` ended the readers and the gateway together with lk-dora-host, so
    over 3 of 3 runs the drain reached nobody: no "handed back" line, the last health operational."""
    lk = like_keras()
    dora = dora_backend()
    request.getfixturevalue("private_dora")
    from kpi.facilities.parts import HealthLog, LogLog

    library = feature_library("hand_back")
    name = f"drain_{os.getpid()}"
    handed_back = tmp_path / "handed_back"
    s = lk.System(name)
    box = s.add(lk.native(library, config={"path": str(handed_back)}), "box")
    s.add(LogLog(path=str(tmp_path / "log.jsonl")), "log_reader", value=box.log)
    s.add(HealthLog(path=str(tmp_path / "health.jsonl")), "health_reader", value=box.health)
    dep = lk.Deployment(s, backend=dora)
    dep.record(tmp_path / "run.mcap", box.log, box.health)
    dep.place(s.box, process="box", hz=20)
    dep.place(s.log_reader, s.health_reader, process="readers")
    node_pids: set[int] = set()
    watching = threading.Event()

    def watch() -> None:  # every pid the box's node had, from start() until stop() returned
        while not watching.is_set():
            node_pids.update(procs.pids_with("lk-dora-host", name))
            time.sleep(0.02)

    watcher = threading.Thread(target=watch, daemon=True)
    dep.start()
    watcher.start()
    try:
        procs.wait_for(lambda: any(row["path"].endswith("hand_back") for row in rows(tmp_path / "health.jsonl")),
                       30.0, "a health record of the running box at the reader")
    finally:
        dep.stop()
        watching.set()
        watcher.join(timeout=5)
    assert handed_back.read_text() == "handed back\n"
    read_lines = [row["message"] for row in rows(tmp_path / "log.jsonl")]
    assert read_lines.count("handed back") == 1, f"the live reader got {read_lines}"
    read_health = [row for row in rows(tmp_path / "health.jsonl") if row["path"].endswith("hand_back")]
    _assert_ends_offline([row["mode"] for row in read_health], "the live reader")
    recorded_lines = [row["message"] for row in recorded(tmp_path / "run.mcap", "log")]
    assert recorded_lines.count("handed back") == 1, f"the recording holds {recorded_lines}"
    recorded_health = [row for row in recorded(tmp_path / "run.mcap", "health") if row["path"].endswith("hand_back")]
    _assert_ends_offline([row["mode"] for row in recorded_health], "the recording")
    assert len(node_pids) == 1, f"the box's lk-dora-host had pids {sorted(node_pids)}: dora restarted it during the stop"


def _assert_ends_offline(modes: list[str], where: str) -> None:
    assert modes and modes[-1] == "offline", f"{where}: the box's health modes were {modes}"
    first = modes.index("offline")
    assert all(mode == "offline" for mode in modes[first:]), f"{where}: the box ran again after offline: {modes}"


# ---------------------------------------------------------------- A22 with §4.5 (M0.4; library verifier, round 1)

TIE_ROUNDS = {1.0: ("error", "Braked", ""), 2.0: ("error", "Braked", "brake held: J2"),
              3.0: ("warn", "Slipping", "slipping: J3"), 4.0: ("warn", "Slipping", "")}


@pytest.mark.parametrize("combination", ["python-inproc", "cpp-inproc"])
def test_health_tie_keeps_the_facility_message(combination, tmp_path):
    """CONTRACT §15 A22 with §4.5: health reports the worse of the component's own health(level, code) and the
    facility's; on a tie the component's code wins and the facility's message is kept. In both languages, lockstep
    (rounds at 1 s, 2 s, ...): round 1 error / Braked (the component alone); round 2, a log.error in the period:
    error / Braked with the logged text; round 3, health("warn", "Slipping") and a warning in the period: warn /
    Slipping with the warning's text; round 4 warn / Slipping, nothing logged, no message. WHY (library verifier,
    round 1): A22's tie rule cleared the message, so the attach proxy's LeaseHeld health, whose holder travels only
    in its log.error line, no longer named the holder (§4.5; T8 checks that one)."""
    lk = like_keras()
    from kpi.facilities.parts import HealthLog

    s = lk.System("tie_health")
    if combination.startswith("cpp"):
        subject = s.add(lk.native(feature_library("tie_health"), config={"unused": 0}), "box")
    else:
        from kpi.facilities.friction_parts import TieHealth

        subject = s.add(TieHealth(), "subject")
    s.add(HealthLog(path=str(tmp_path / "health.jsonl")), "health_sink", value=subject.health)
    dep = lk.Deployment(s, realtime=False)
    dep.place(subject, process="subject", hz=50)
    dep.place(s.health_sink, process="sinks")
    try:
        dep.start(until=4.6)
    finally:
        dep.stop()
    found = {round(row["stamp"], 6): (row["level"], row["code"], row["message"])
             for row in rows(tmp_path / "health.jsonl") if row["path"].endswith("subject") and row["mode"] == "operational"}
    wrong = {stamp: (found.get(stamp), want) for stamp, want in TIE_ROUNDS.items() if found.get(stamp) != want}
    assert not wrong, f"health round: (sent, expected): {wrong}"
