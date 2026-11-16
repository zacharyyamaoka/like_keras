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
Ruler M0.5 (CONTRACT §15 A38) adds three:
  A33 test_non_finite_params_defaults              params: NaN and +-inf defaults round-trip
  A35 test_dora_boundary_accounting                metrics: io_sched_policy, drops and send gaps per exposed Out
  A37 test_dora_stop_drains_python_to_live_readers lifecycle: A28's drain for a Python node under dora
The C++ boxes (hand_back.cpp, self_health.cpp, table_params.cpp, tie_health.cpp, non_finite.cpp) are built on demand
(kpi/support/lk.feature_library): until a decision lands, its box does not compile, or the library does not do it,
and its test FAILS, never skips.
"""

from __future__ import annotations

import ctypes
import json
import math
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


# ================================================================ ruler M0.5 (CONTRACT §15 A38)

# ---------------------------------------------------------------- A33 (consumer-016): params

NON_FINITE = {"off": math.nan, "upper": math.inf, "lower": -math.inf}


def _same_number(found, expected) -> bool:
    if expected is None or (isinstance(expected, float) and math.isnan(expected)):
        return found is None or (isinstance(found, float) and math.isnan(found))
    return found == expected


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_non_finite_params_defaults(backend_name, tmp_path, request):
    """CONTRACT §15 A33: lk_describe() prints a non-finite params default as NaN, Infinity or -Infinity (what Python's
    json and strtod read), and lk.native() reads them back. non_finite.cpp's defaults are off = NaN, upper = inf,
    lower = -inf, gain = 0.5: its lk_describe() is JSON Python reads, with those values; lk.native() accepts the
    .so; and the values round-trip into the running box, in one process and across processes (lockstep): with no
    config the box runs its defaults, and with a config that carries non-finite values (Python's json writes them
    the same way) describe() shows them and the box runs them. WHY (consumer-016): `nan` and `inf` (std::to_chars)
    made lk.native() fail with a JSONDecodeError, so a component's natural "NaN = off" default broke it."""
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    chosen = backend(backend_name)
    library = feature_library("non_finite")
    from kpi.polyglot.friction_parts import JointStateLog

    so = ctypes.CDLL(str(library))
    so.lk_describe.restype = ctypes.c_char_p
    text = so.lk_describe().decode()
    try:
        defaults = json.loads(text)["params"]
    except ValueError as error:
        pytest.fail(f"lk_describe() is not JSON Python reads ({error}): {text[:400]}")
    assert set(defaults) == {"off", "upper", "lower", "gain"} and defaults["gain"] == 0.5, defaults
    assert all(_same_number(defaults[key], value) for key, value in NON_FINITE.items()), defaults
    for case, config in (("defaults", None), ("config", {**NON_FINITE, "gain": 2.0})):
        s = lk.System(f"non_finite_{backend_name}_{case}")
        box = s.add(lk.native(library, config=config), "box")
        s.add(JointStateLog(path=str(tmp_path / f"{case}.jsonl")), "sink", value=box.echo)
        dep = lk.Deployment(s, backend=chosen, realtime=False)
        dep.place(s.box, process="box", hz=10)
        dep.place(s.sink, process="sink")
        plan = dep.describe()
        json.dumps(plan)  # the datasheet stays JSON-serialisable data (§4.4)
        if config is not None:
            shown = next(component["params"] for path, component in plan["components"].items() if path.endswith(".box"))
            assert set(shown) == set(config) and all(_same_number(shown[key], config[key]) for key in config), shown
        try:
            dep.start(until=0.25)
        finally:
            dep.stop()
        echoed = rows(tmp_path / f"{case}.jsonl")
        assert len(echoed) == 1, f"{case}: {echoed}"
        expected = [math.nan, math.inf, -math.inf, 0.5 if config is None else 2.0]
        assert all(_same_number(found, want) for found, want in zip(echoed[0]["position"], expected)), \
            f"{case}: the box ran (off, upper, lower, gain) = {echoed[0]['position']}, expected {expected}"


# ---------------------------------------------------------------- A35 (consumer-018): metrics at the dora boundary

SLOW_SUBSCRIBER_S = 1.5
DAEMON_STALL_S = 4.0  # T4's stall
BOX_TICK_S = 0.01  # the reference arm at 100 Hz
IN_FLIGHT_S = 0.1  # a step older than this at a metrics record has been taken by TX (sent or superseded) by then


def _missing_runs(states: list[dict], before: float) -> tuple[int, int]:
    """(how many seqs are missing between the subscriber's states stamped before `before`, the longest run of them)."""
    stamped = sorted((row["seq"], row["stamp"]) for row in states if row["stamp"] < before)
    runs = [later - earlier - 1 for (earlier, _), (later, _) in zip(stamped, stamped[1:]) if later - earlier > 1]
    return sum(runs), max(runs, default=0)


def test_dora_boundary_accounting(tmp_path, request):
    """CONTRACT §15 A35, default tier: lk-dora-host reports io_sched_policy in the box record (Linux's numbers, as
    A19's sched_policy) and counts per exposed Out the steps that never went out and the worst gap between sends.
    The ruler reads them where consumer-018 proposed and A9 put the box's own counters: the box record (path = the
    box's own path), as drops.<exposed name> and send_gap_max_s.<exposed name> (seconds).
    The reference arm (target, real time, 100 Hz, its planner on the fake bus; exposed outputs state and command)
    with two subscribers of `state` in processes of their own and a reader of its metrics. One subscriber is made
    deliberately slow (SIGSTOP for SLOW_SUBSCRIBER_S), then the boundary's own consumer, the private dora daemon,
    is stopped for DAEMON_STALL_S (T4's lever). Then: io_sched_policy is reported, realtime whenever the RT thread's
    sched_policy is (A19's ladder), and a realtime report is true of lk-dora-host's threads; drops.state,
    drops.command and both send gaps are reported; and the count is exact against the fast subscriber (queue 1000:
    dora drops nothing for it, so a seq it misses never left the box): at every box record, the states it missed
    before the record <= drops.state <= the box's steps - the states it received stamped up to the record; and the
    worst send gap spans the longest run of states it missed.
    WHY exactness rather than "non-zero": dora 0.5 never makes a sender wait on a subscriber, and buffers a stopped
    daemon's sends (ruler lane, measured on today's library: a subscriber stopped 1.5 s cost the fast one 0-3
    states, as quiet windows do; a daemon stopped 3 s, 0; stopped 12 s, one run of 26 states at 0.77 s); what loses
    states is a starved TX thread, which no test can starve on demand. On this host some are lost in every run.
    WHY (consumer-018): over dora the controller's boundary ran SCHED_OTHER while only the RT thread was raised,
    and the box record carried drops.log only, so 2-3 % of states lost at a subscriber showed up nowhere."""
    lk = like_keras()
    dora = dora_backend()
    private = request.getfixturevalue("private_dora")
    from kpi.consumer.boundary_ab import thread_policies
    from kpi.consumer.boundary_parts import SeqLog
    from kpi.facilities.parts import MetricsLog
    from kpi.tasks.arm_cell.planner import PickPlanner
    from kpi.tasks.arm_cell.targets import TargetFile

    native_library("arm_controller")
    name = f"boundary_{os.getpid()}"
    s = lk.System(name)
    targets = s.add(TargetFile(repeat=True), "targets")
    arm = s.add(lk.native(cells.cell_module().LIBRARY, config=cells.config(tmp_path, lockstep=False)), "arm")
    planner = s.add(PickPlanner(), "planner", target=targets.target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    s.add(SeqLog(path=str(tmp_path / "fast.jsonl")), "fast", value=arm.state)
    s.add(SeqLog(path=str(tmp_path / "slow.jsonl")), "slow", value=arm.state)
    s.add(MetricsLog(path=str(tmp_path / "metrics.jsonl")), "metrics_reader", value=arm.metrics)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.arm, process="arm-controller", hz=1 / BOX_TICK_S, cpus=kpi_cpus(), fifo=80)
    dep.place(s.targets, hz=2)
    dep.place(s.planner)
    dep.place(s.fast, process="fast")
    dep.place(s.slow, process="slow")
    dep.place(s.metrics_reader, process="metrics_reader")
    stopped: list[int] = []
    dep.start()
    try:
        procs.wait_for(lambda: len(rows(tmp_path / "fast.jsonl")) >= 100, 60.0, "the fast subscriber's first states")
        host = procs.wait_for(lambda: next(iter(procs.pids_with("lk-dora-host", name)), None), 10.0, "lk-dora-host")
        threads = thread_policies(host)
        slow = procs.wait_for(lambda: next((pid for pid in procs.pids_with("like_keras.backends.dora.host", name)
                                            if open(f"/proc/{pid}/cmdline", "rb").read().rstrip(b"\0")
                                            .endswith(b"slow")), None), 10.0, "the slow subscriber's node")
        for victim, seconds in ((slow, SLOW_SUBSCRIBER_S), (private.daemon_pid, DAEMON_STALL_S)):
            os.kill(victim, signal.SIGSTOP)
            stopped.append(victim)
            time.sleep(seconds)
            os.kill(victim, signal.SIGCONT)
            stopped.remove(victim)
            time.sleep(1.0)
        time.sleep(2.0)  # two more metrics rounds after the stall
    finally:
        for victim in stopped:
            os.kill(victim, signal.SIGCONT)
        dep.stop()
    box = [row for row in rows(tmp_path / "metrics.jsonl") if row["path"] == f"{name}.arm"]
    assert box, f"no box record (path {name}.arm) reached the metrics reader"
    reported = [row["values"] for row in box if "io_sched_policy" in row["values"]]
    assert reported, f"the box record does not report io_sched_policy; it has {sorted(box[-1]['values'])}"
    realtime = {os.SCHED_FIFO, os.SCHED_RR}
    for values in reported:
        policy = int(values["io_sched_policy"])
        assert policy in {os.SCHED_OTHER, *realtime}, values
        if int(values.get("sched_policy", os.SCHED_OTHER)) in realtime:
            assert policy in realtime, f"the RT thread runs realtime, the I/O threads {policy} (A19's ladder): {values}"
    if int(reported[0]["io_sched_policy"]) in realtime:
        raised = sum(count for key, count in threads.items() if key.split()[0] in ("RR", "FIFO"))
        assert raised >= 3, f"io_sched_policy says realtime; lk-dora-host's threads are {threads}"
    for port in ("state", "command"):
        for metric in (f"drops.{port}", f"send_gap_max_s.{port}"):
            assert any(metric in row["values"] for row in box), \
                f"the box record has no {metric}: {sorted(box[-1]['values'])}"
    states = rows(tmp_path / "fast.jsonl")
    checked, longest_run = 0, 0
    for row in box:
        values, at = row["values"], row["stamp"]
        if "drops.state" not in values or "steps" not in values:
            continue
        missed, run = _missing_runs(states, at - IN_FLIGHT_S)
        received = sum(1 for state in states if state["stamp"] <= at)
        assert missed <= values["drops.state"] <= values["steps"] - received + 2, (
            f"the box record at {at:.2f} counts drops.state {values['drops.state']}: the fast subscriber missed "
            f"{missed} states before it, and the box sent at least {received} of its {values['steps']} steps")
        checked += 1
        longest_run = max(longest_run, run)
    assert checked >= 5, f"{checked} box records with drops.state and steps"
    gap = max(row["values"].get("send_gap_max_s.state", 0.0) for row in box)
    assert gap >= max(0.5 * BOX_TICK_S, longest_run * BOX_TICK_S - 2e-3), (
        f"send_gap_max_s.state {gap} s; the fast subscriber missed a run of {longest_run} consecutive states")


# ---------------------------------------------------------------- A37 (ruler lane, M0.4): the Python drain


def test_dora_stop_drains_python_to_live_readers(tmp_path, request):
    """CONTRACT §15 A37, the Python twin of test_dora_stop_drains_to_live_readers: under a plain real-time dora
    stop(), a Python node's deactivate log line and offline health reach live readers and the recording as a native
    node's do (A30's stop order includes the Python hosts). HandBackPy (its deactivate writes a file and logs
    "handed back") in its own process at 20 Hz, real time, its log and health wired to readers in another process
    and recorded; once the readers hold a health record of the running box, a plain dep.stop(). Then deactivate
    ran once; the reader and the recording each hold "handed back" once; the box's last health record says offline,
    with nothing operational after it; and the box's node kept one pid (dora never restarted it).
    WHY (ruler lane, M0.4's probe): a Python box's line reached neither the reader nor the recording (3 of 3 runs),
    while a C++ box's reached both."""
    lk = like_keras()
    dora = dora_backend()
    request.getfixturevalue("private_dora")
    from kpi.facilities.friction_parts import HandBackPy
    from kpi.facilities.parts import HealthLog, LogLog

    name = f"drain_py_{os.getpid()}"
    handed_back = tmp_path / "handed_back"
    s = lk.System(name)
    box = s.add(HandBackPy(path=str(handed_back)), "box")
    s.add(LogLog(path=str(tmp_path / "log.jsonl")), "log_reader", value=box.log)
    s.add(HealthLog(path=str(tmp_path / "health.jsonl")), "health_reader", value=box.health)
    dep = lk.Deployment(s, backend=dora)
    dep.record(tmp_path / "run.mcap", box.log, box.health)
    dep.place(s.box, process="box", hz=20)
    dep.place(s.log_reader, s.health_reader, process="readers")
    node_pids: set[int] = set()
    watching = threading.Event()

    def box_node() -> list[int]:
        return [pid for pid in procs.pids_with("like_keras.backends.dora.host", name)
                if open(f"/proc/{pid}/cmdline", "rb").read().rstrip(b"\0").endswith(b"box")]

    def watch() -> None:  # every pid the box's node had, from start() until stop() returned
        while not watching.is_set():
            try:
                node_pids.update(box_node())
            except OSError:  # a node that ended while it was being read
                pass
            time.sleep(0.02)

    watcher = threading.Thread(target=watch, daemon=True)
    dep.start()
    watcher.start()
    try:
        procs.wait_for(lambda: any(row["path"] == f"{name}.box" for row in rows(tmp_path / "health.jsonl")),
                       30.0, "a health record of the running box at the reader")
    finally:
        dep.stop()
        watching.set()
        watcher.join(timeout=5)
    assert handed_back.read_text() == "handed back\n"
    read_lines = [row["message"] for row in rows(tmp_path / "log.jsonl")]
    assert read_lines.count("handed back") == 1, f"the live reader got {read_lines}"
    read_health = [row for row in rows(tmp_path / "health.jsonl") if row["path"] == f"{name}.box"]
    _assert_ends_offline([row["mode"] for row in read_health], "the live reader")
    recorded_lines = [row["message"] for row in recorded(tmp_path / "run.mcap", "log")]
    assert recorded_lines.count("handed back") == 1, f"the recording holds {recorded_lines}"
    recorded_health = [row for row in recorded(tmp_path / "run.mcap", "health") if row["path"] == f"{name}.box"]
    _assert_ends_offline([row["mode"] for row in recorded_health], "the recording")
    assert len(node_pids) == 1, f"the box's node had pids {sorted(node_pids)}: dora restarted it during the stop"
