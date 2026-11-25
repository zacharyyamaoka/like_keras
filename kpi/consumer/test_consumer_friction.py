"""Criterion 1 (consumer fit), ruler M0.3 (CONTRACT §15 A27): the first consumer's friction with the controller's
life cycle and the swap.

Each test judges one decision of §15 and counts as a companion of the consumer test it extends (kpi/score.py: that
test counts only when its companion passes too):
  A20 test_period_in_activate            T7: refused in activate before any step, when one exchange cannot fit a tick
  A25 test_fresh_input_counts_arrivals   T1: Python and C++ count the same arrivals (the swap changes nothing)
  A26 test_T2_export_and_push_apart      T2 (timing tier): the send split into the export and lk_push
  A30 test_log_reaches_readers_at_its_step (M0.4)  T1: a native log line reaches a reader as a Python twin's does
Ruler M0.5 (CONTRACT §15 A38) adds four, on the second consumer round (consumer-015, 017, 018, 019):
  A32 test_optional_input_in_python              T1: lk.In(T, optional=True), the Python peer of an optional C++ In
  A34 test_refused_activate_is_final_under_dora  T7: a refusal under dora is final; start() raises naming it
  A35 test_A35_boundary_within_noise_of_raw_dora T4 (timing tier): target's boundary within noise of raw dora
  A36 test_attach_lease_per_input                T8: the attach write lease is per exposed input
The C++ boxes (period_probe.cpp, arrivals.cpp, log_steps.cpp, refuser.cpp) are built on demand
(kpi/support/lk.feature_library): until a decision lands, its box does not compile, or the library does not do it,
and its test FAILS, never skips.
"""

from __future__ import annotations

import ctypes
import json
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from kpi.support import backends, cells, procs
from kpi.support.lk import (BUILD, KPI, backend, dora_backend, feature_library, kpi_cpus, like_keras, native_library,
                            recorded, subprocess_env, timing_tier)

COMBINATIONS = backends.combinations("python", "cpp")  # ruler v2: python-<backend>, cpp-<backend>


def rows(path) -> list[dict]:
    try:
        return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    except FileNotFoundError:
        return []


# ---------------------------------------------------------------- A20 (consumer-005): T7's shape


def _probe(lk, language: str, min_period_s: float):
    if language == "cpp":
        return lk.native(feature_library("period_probe"), config={"min_period_s": min_period_s})
    from kpi.consumer.friction_parts import PeriodProbe

    return PeriodProbe(min_period_s=min_period_s)


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_period_in_activate(combination, tmp_path, request):
    """CONTRACT §15 A20: period(), the group's period in seconds, from activate() on, in both languages. A probe
    placed at 50 Hz reads 0.02 in activate and in forward (lockstep, both backends); in one process, a Python probe
    in a rate-less group reads 0.0, and a probe whose min_period_s is above its period refuses in activate:
    start() fails naming the reason, and no step ran (T7's shape). WHY (consumer-005): the raw node's open-time gate
    (20 query exchanges, refused when their p95 is over 0.8 x the tick) could not be ported, so a placement the bus
    cannot serve showed up only as overruns, after the arm had been commanded."""
    language, backend_name = combination.split("-")
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    chosen = backend(backend_name)
    from kpi.consumer.friction_parts import RatelessPeriodProbe
    from kpi.polyglot.friction_parts import JointStateLog

    s = lk.System(f"period_{language}_{backend_name}")
    probe = s.add(_probe(lk, language, 0.0), "probe")
    s.add(JointStateLog(path=str(tmp_path / "probe.jsonl")), "sink", value=probe.report)
    rateless = language == "python" and backend_name == "inproc"
    if rateless:
        follower = s.add(RatelessPeriodProbe(), "rateless", trigger=probe.report)
        s.add(JointStateLog(path=str(tmp_path / "rateless.jsonl")), "rateless_sink", value=follower.report)
    dep = lk.Deployment(s, backend=chosen, realtime=False)
    dep.place(s.probe, process="probe", hz=50)
    dep.place(s.sink, process="sink")
    if rateless:
        dep.place(s.rateless, s.rateless_sink)
    try:
        dep.start(until=0.3)
    finally:
        dep.stop()
    reports = rows(tmp_path / "probe.jsonl")
    assert len(reports) >= 10, f"{len(reports)} steps reported"
    assert all(row["position"] == pytest.approx([0.02, 0.02], abs=1e-12) for row in reports), reports[:3]
    if rateless:
        followed = rows(tmp_path / "rateless.jsonl")
        assert followed and all(row["position"] == [0.0, 0.0] for row in followed), followed[:3]
    if backend_name == "inproc":  # under dora a refused activate is a node exiting non-zero (T7 itself)
        s = lk.System(f"period_refused_{language}")
        probe = s.add(_probe(lk, language, 0.05), "probe")
        s.add(JointStateLog(path=str(tmp_path / "refused.jsonl")), "sink", value=probe.report)
        dep = lk.Deployment(s, realtime=False)
        dep.place(s.probe, hz=50)
        dep.place(s.sink)
        with pytest.raises(Exception) as refused:
            dep.start(until=0.3)
        dep.stop()
        assert "does not fit in a tick" in str(refused.value), str(refused.value)
        assert rows(tmp_path / "refused.jsonl") == [], "a probe that refused in activate stepped"


# ---------------------------------------------------------------- A25 (consumer-010): T1's parity


@pytest.mark.parametrize("combination", COMBINATIONS)
def test_fresh_input_counts_arrivals(combination, tmp_path, request):
    """CONTRACT §15 A25: lk.In(T, fresh=True) gives forward None for that input unless a value arrived since the
    last step. A source sends one plan object twice, at 0.1 s and 0.6 s (10 Hz, lockstep); a Python counter
    (fresh=True) and a C++ counter (take()) at 10 Hz in another group each see it arrive on the steps at 0.2 s and
    0.7 s (§5.4: the first step strictly after the send) and on no other, and count 2, in one process and across
    processes. WHY (consumer-010): a Python component could not tell a new plan from the one it saw last step, so
    it guessed by identity and value, and a plan sent again counted twice in C++ and once in Python."""
    language, backend_name = combination.split("-")
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    chosen = backend(backend_name)
    from kpi.consumer.friction_parts import SameObjectSource
    from kpi.polyglot.friction_parts import JointStateLog

    s = lk.System(f"arrivals_{language}_{backend_name}")
    source = s.add(SameObjectSource(), "source")
    if language == "cpp":
        counter = s.add(lk.native(feature_library("arrivals"), config={"unused": 0}), "counter", plan=source.plan)
    else:
        from kpi.consumer.fresh_parts import Arrivals

        counter = s.add(Arrivals(), "counter", plan=source.plan)
    s.add(JointStateLog(path=str(tmp_path / "count.jsonl")), "sink", value=counter.count)
    dep = lk.Deployment(s, backend=chosen, realtime=False)
    dep.place(s.source, process="source", hz=10)
    dep.place(s.counter, process="counter", hz=10)
    dep.place(s.sink, process="sink")
    try:
        dep.start(until=1.05)
    finally:
        dep.stop()
    counts = sorted(rows(tmp_path / "count.jsonl"), key=lambda row: row["stamp"])
    assert len(counts) >= 9, f"{len(counts)} steps counted"
    arrived_at = [round(row["stamp"], 6) for row in counts if row["position"][0] == 1.0]
    assert arrived_at == [0.2, 0.7], f"arrivals seen at {arrived_at}"
    assert counts[-1]["position"][1] == 2.0, counts[-1]


# ---------------------------------------------------------------- A26 (consumer-011): T2 split, timing tier


class _TimedPush:
    """lk_push as like_keras calls it (through ctypes.PyDLL, CONTRACT §6.3), timed: the hand-off into the box, apart
    from everything the send did before the call (the export of the message)."""

    def __init__(self, function, durations: list[float]) -> None:
        object.__setattr__(self, "_function", function)
        object.__setattr__(self, "_durations", durations)

    def __setattr__(self, name, value) -> None:  # argtypes, restype
        setattr(self._function, name, value)

    def __getattr__(self, name):
        return getattr(self._function, name)

    def __call__(self, *arguments):
        started = time.perf_counter_ns()
        result = self._function(*arguments)
        self._durations.append((time.perf_counter_ns() - started) * 1e-9)
        return result


def _timed_pydll(durations: list[float]):
    class TimedPyDLL(ctypes.PyDLL):
        def __getattr__(self, name):
            function = super().__getattr__(name)
            if name == "lk_push":
                function = _TimedPush(function, durations)
                setattr(self, name, function)
            return function

    return TimedPyDLL


@pytest.mark.timing
@pytest.mark.parametrize("placement", backends.placements("same_box"))
def test_T2_export_and_push_apart(placement, tmp_path, monkeypatch):
    """CONTRACT §15 A26, timing tier: T2's 1000 sends of a 7 x 1000 plan in one process, while the node ticks, with
    the send split in two: the time inside lk_push (the C ABI hand-off) and the rest of the send on the planner's
    thread (the Python export of the message). Each half, and the whole send, must stay within T2's in-process
    0.1 ms at p99; the split is written to build/kpi/t2_split.json so a regression in either half is visible.
    WHY (consumer-011): A2's push budget was spent in the export (0.17 ms p50), not in lk_push (0.013 ms)."""
    lk = like_keras()
    from kpi.consumer.parts import HandoffProbe, TimedPlanner
    from kpi.tasks.arm_cell.targets import TargetFile

    pushes: list[float] = []
    monkeypatch.setattr(ctypes, "PyDLL", _timed_pydll(pushes))
    targets = tmp_path / "targets.jsonl"
    lines = open(KPI / "tasks" / "arm_cell" / "targets.jsonl", encoding="utf-8").read().splitlines()
    targets.write_text("\n".join(lines[index % len(lines)] for index in range(1000)) + "\n")
    config = cells.config(tmp_path, lockstep=False)
    s = lk.System("pick_cell")
    source = s.add(TargetFile(path=str(targets)), "targets")
    arm = s.add(lk.native(native_library("arm_controller"), config=config), "arm")
    planner = s.add(TimedPlanner(), "planner", target=source.target, state=arm.state)
    s.add(HandoffProbe(path=str(tmp_path / "handoff.jsonl")), "probe", trajectory=planner.trajectory)
    s.wire(arm, trajectory=planner.trajectory)
    dep = lk.Deployment(s)
    dep.place(s.arm, process="arm-controller", hz=100, cpus=kpi_cpus(), fifo=80)
    dep.place(s.targets, hz=50)
    dep.place(s.planner, s.probe)
    dep.start()
    try:
        time.sleep(22.0)
    finally:
        dep.stop()
    sends = np.array([row["gap_s"] for row in rows(tmp_path / "handoff.jsonl")])
    pushed = np.array(pushes)
    assert len(sends) >= 990, f"{len(sends)} of 1000 sends measured"
    assert len(pushed) == len(sends), f"{len(pushed)} lk_push calls for {len(sends)} sends: the split cannot pair them"
    exports = sends - pushed
    split = {name: {"p50_s": float(np.percentile(values, 50)), "p99_s": float(np.percentile(values, 99)),
                    "max_s": float(values.max())} for name, values in (("send", sends), ("export", exports),
                                                                       ("lk_push", pushed))}
    split["n"] = len(sends)
    BUILD.mkdir(parents=True, exist_ok=True)
    (BUILD / "t2_split.json").write_text(json.dumps(split, indent=1) + "\n")
    over = [f"{name} p99 {split[name]['p99_s'] * 1e3:.3f} ms" for name in ("export", "lk_push", "send")
            if split[name]["p99_s"] > 0.1e-3]
    assert not over, f"over T2's in-process 0.1 ms: {', '.join(over)}"


# ---------------------------------------------------------------- A30 (contract-007 item 3; M0.4): T1's log tap

LOG_STEPS = [(0.3, 20, "line 0"), (1.0, 30, "line 1"), (1.7, 40, "line 2")]  # log_steps.cpp: when, level, text
LOG_STEPS_UNTIL_S = 1.9  # WHY: "line 2" falls after the last facility round (1.0 s), as consumer-012's brake line did


def _log_tap(placement: str, directory) -> list[tuple[float, int, str, int]]:
    """log_steps.cpp (same_box, target) or its Python twin LogSteps (fallback) at 50 Hz, lockstep until 1.9 s, its
    `log` port wired to a reader in a rate-less group of its own process. What the reader got: (its now() at the
    step it got the record, level, message, the record's tick), in order."""
    lk = like_keras()
    from kpi.facilities.parts import LogLog

    directory.mkdir(parents=True, exist_ok=True)
    s = lk.System(f"log_tap_{placement}")
    if placement == "fallback":
        from kpi.consumer.friction_parts import LogSteps

        talker = s.add(LogSteps(), "talker")
    else:
        talker = s.add(lk.native(feature_library("log_steps"), config={"unused": 0}), "talker")
    s.add(LogLog(path=str(directory / "log.jsonl")), "reader", value=talker.log)
    dep = lk.Deployment(s, backend=backend("dora" if placement == "target" else "inproc"), realtime=False)
    dep.place(s.talker, process="talker", hz=50)
    dep.place(s.reader, process="reader")
    try:
        dep.start(until=LOG_STEPS_UNTIL_S)
    finally:
        dep.stop()
    return [(row["now"], row["level"], row["message"], row["tick"]) for row in rows(directory / "log.jsonl")]


@pytest.mark.backend("dora")
def test_log_reaches_readers_at_its_step(tmp_path, request):
    """CONTRACT §15 A30 (contract-007 item 3), T1's swap seen through a log tap: under lockstep a reader component
    gets a native box's log line at the step that logged it, the same as a Python twin's. log_steps.cpp and its
    Python twin log "line 0" (info) at 0.3 s, "line 1" (warning) at 1.0 s and "line 2" (error) at 1.7 s, at 50 Hz,
    until 1.9 s; a reader in a rate-less group of its own process gets, in fallback (the Python twin), each line at
    the step that logged it; in same_box and in target (the C++ box, inproc and dora) exactly the same (its now(),
    to 1e-9 s, and the record's level, message and tick). WHY (library verifier, round 1; consumer-012): the box's log ring was drained only
    with health and metrics, once a second, so a reader got a C++ line up to a second late, and a line logged after
    the last facility round never, while the Python twin's arrived at once: a log tap read a swap differently."""
    request.getfixturevalue("private_dora")
    taps = {placement: _log_tap(placement, tmp_path / placement) for placement in ("fallback", "same_box", "target")}
    fallback = taps["fallback"]
    assert [(level, message) for _, level, message, _ in fallback] == [(level, message) for _, level, message in LOG_STEPS], \
        f"fallback: the reader got {fallback}"
    for (got_at, _, message, _), (logged_at, _, _) in zip(fallback, LOG_STEPS):
        assert abs(got_at - logged_at) <= 1e-9, f"fallback: {message!r}, logged at {logged_at} s, read at {got_at} s"
    for placement in ("same_box", "target"):
        got = taps[placement]
        assert [row[1:] for row in got] == [row[1:] for row in fallback], \
            f"{placement}: the reader got {got}; with the Python twin (fallback) it got {fallback}"
        late = [(message, at, twin_at) for (at, _, message, _), (twin_at, _, _, _) in zip(got, fallback)
                if abs(at - twin_at) > 1e-9]
        assert not late, f"{placement}: (line, read at, read at with the Python twin): {late}"


# ================================================================ ruler M0.5 (CONTRACT §15 A38): consumer-014..019

# ---------------------------------------------------------------- A32 (consumer-015): T1's Python peer

DEADMAN_TIMEOUT_S = 0.3  # verdict.cpp's and DeadmanPy's guard_timeout_s
DEADMAN_TICK_S = 0.01
VERDICT_TICK_S = 0.05


def _deadman_pair(backend_name: str, directory, guard: dict | None) -> dict[str, list[dict]]:
    """verdict.cpp's box and its Python twin DeadmanPy (lk.In(DiagnosticRecord, optional=True)) side by side in one
    System at 100 Hz, lockstep for 2 s, each in a process of its own: both `verdict` inputs wired to one VerdictGuard
    (**guard) at 20 Hz, or both left unwired (guard None). Each one's commands, in step order, and the Python one's
    ports as describe() lists them."""
    lk = like_keras()
    from kpi.consumer.optional_parts import DeadmanPy
    from kpi.polyglot.friction_parts import JointStateLog, VerdictGuard

    directory.mkdir(parents=True, exist_ok=True)
    s = lk.System(f"deadman_{backend_name}_{directory.name}")
    config = {"guard_timeout_s": DEADMAN_TIMEOUT_S, "step_rad": 0.001}
    if guard is None:
        cpp = s.add(lk.native(feature_library("verdict"), config=config), "cpp")
        python = s.add(DeadmanPy(**config), "python")
    else:
        verdicts = s.add(VerdictGuard(**guard), "guard")
        cpp = s.add(lk.native(feature_library("verdict"), config=config), "cpp", verdict=verdicts.verdict)
        python = s.add(DeadmanPy(**config), "python", verdict=verdicts.verdict)
    s.add(JointStateLog(path=str(directory / "cpp.jsonl")), "cpp_sink", value=cpp.command)
    s.add(JointStateLog(path=str(directory / "python.jsonl")), "python_sink", value=python.command)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.cpp, process="cpp", hz=1 / DEADMAN_TICK_S)
    dep.place(s.python, process="python", hz=1 / DEADMAN_TICK_S)
    if guard is not None:
        dep.place(s.guard, process="guard", hz=1 / VERDICT_TICK_S)
    dep.place(s.cpp_sink, s.python_sink, process="sinks")
    ports = next(component["ports"] for path, component in dep.describe()["components"].items()
                 if path.endswith(".python"))
    try:
        dep.start(until=2.0)
    finally:
        dep.stop()
    return {"ports": ports, **{language: sorted(rows(directory / f"{language}.jsonl"), key=lambda row: row["stamp"])
                               for language in ("cpp", "python")}}


@pytest.mark.parametrize("backend_name", backends.names())
def test_optional_input_in_python(backend_name, tmp_path, request):
    """CONTRACT §15 A32: lk.In(T, optional=True), A23's meaning in Python: unwired is legal, forward gets None for
    it (the argument is passed; DeadmanPy's forward has no default for it), describe() lists it. T1's shape: the
    Python peer of verdict.cpp's guard deadman (kpi/consumer/optional_parts.py, the same ports) beside the C++ box,
    lockstep, in one process and across processes: unwired, both move on every step; wired to a guard that stops
    confirming at 1 s, and to one that sends level error at 0.5 s, both hold at the same step; in every run the two
    send the same commands to 1e-9 rad. WHY (consumer-015): the all-Python fallback had to set a private attribute
    (`port.optional = True`) to mirror the C++ verdict input, so a refactor of the port class could silently break
    the one-line swap (T1, criterion 5)."""
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    for wiring, guard in (("unwired", None), ("silent", {"confirm_until_s": 1.0}), ("stopped", {"stop_at_s": 0.5})):
        run = _deadman_pair(backend_name, tmp_path / wiring, guard)
        assert run["ports"].get("verdict", {}).get("dir") == "in", \
            f"describe() does not list the optional input: {run['ports']}"
        python, cpp = run["python"], run["cpp"]
        assert len(python) >= 150 and len(python) == len(cpp), \
            f"{wiring}: {len(python)} Python, {len(cpp)} C++ commands"
        assert [row["stamp"] for row in python] == [row["stamp"] for row in cpp], f"{wiring}: command stamps differ"
        worst = max(abs(left["position"][0] - right["position"][0]) for left, right in zip(python, cpp))
        assert worst <= 1e-9, f"{wiring}: the Python peer's commands differ from the C++ box's by {worst} rad"
        moved = [later["position"][0] > earlier["position"][0] for earlier, later in zip(python, python[1:])]
        if guard is None:
            assert all(moved), "unwired: the Python peer did not move on every step (forward got a verdict?)"
        else:
            assert not all(moved), f"{wiring}: the guard's verdicts never reached the Python peer"


# ---------------------------------------------------------------- A34 (consumer-017): T7 under dora

REFUSAL = "the bus lock is held by kpi_holder"
START_BOUND_S = 60.0  # WHY: a dora dataflow starts in seconds here, but tens of seconds on the loaded cores
SETTLE_S = 3.0  # a node dora restarts refuses again within this (consumer-017: 30-100 restarts a second)


@pytest.mark.backend("dora")
@pytest.mark.parametrize("mode", ["realtime", "lockstep"])
def test_refused_activate_is_final_under_dora(mode, tmp_path, request):
    """CONTRACT §15 A34: a refused activate() is final under dora too, in real time and in lockstep alike.
    refuser.cpp refuses in every activate (it notes each start in a file first), placed in its own process at
    20 Hz, with a reader of its output in another: Deployment.start() raises within START_BOUND_S, naming the
    refusal (the activate's own words); the node was started once and is not started again (counted SETTLE_S
    after the raise); and the dataflow is stopped (no process of it remains, and dora no longer lists it running).
    WHY (consumer-017): under dora a controller refused in activate was restarted by restart_policy 30-100 times a
    second, on the cores the powered arm shares, and start() reported success in real time."""
    lk = like_keras()
    dora = request.getfixturevalue("private_dora")
    chosen = backend("dora")
    library = feature_library("refuser")
    from kpi.polyglot.friction_parts import JointStateLog

    name = f"refuse_{mode}_{os.getpid()}"
    starts = tmp_path / "starts"
    s = lk.System(name)
    box = s.add(lk.native(library, config={"starts_path": str(starts), "reason": REFUSAL}), "box")
    s.add(JointStateLog(path=str(tmp_path / "echo.jsonl")), "reader", value=box.echo)
    dep = lk.Deployment(s, backend=chosen, realtime=mode == "realtime")
    dep.place(s.box, process="box", hz=20)
    dep.place(s.reader, process="reader")
    outcome: dict = {}

    def run() -> None:
        try:
            dep.start(until=None if mode == "realtime" else 5.0)
            outcome["returned"] = True
        except Exception as error:  # noqa: BLE001 - what start() raised is what this test judges
            outcome["error"] = error

    def started() -> int:
        return len(starts.read_text().splitlines()) if starts.exists() else 0

    thread = threading.Thread(target=run, daemon=True)
    begun = time.monotonic()
    thread.start()
    try:
        thread.join(START_BOUND_S)
        took = time.monotonic() - begun
        assert not thread.is_alive(), \
            f"start() neither raised nor returned within {START_BOUND_S} s ({started()} starts)"
        assert "error" in outcome, (f"start() returned normally after {took:.1f} s although the node refused in "
                                    f"activate ({started()} starts so far)")
        assert REFUSAL in str(outcome["error"]), f"start() raised without naming the refusal: {outcome['error']!r}"
        time.sleep(SETTLE_S)
        assert started() == 1, f"the refused node was started {started()} times: dora restarted it"
        left = [pid for needle in ("lk-dora-host", "like_keras.backends.dora.host")
                for pid in procs.pids_with(needle, name)]
        assert not left, f"the dataflow was not stopped: its nodes {left} still run {SETTLE_S} s after start() raised"
        listing = subprocess.run([dora.cli, "list"], cwd=dora.workdir, env=subprocess_env(), capture_output=True,
                                 text=True, timeout=60).stdout
        assert not any(line.split()[1:3] == [name, "Running"] for line in listing.splitlines()[1:]), listing
    finally:
        dep.stop()
        subprocess.run([dora.cli, "stop", "--name", name], cwd=dora.workdir, env=subprocess_env(), capture_output=True,
                       timeout=60)


# ---------------------------------------------------------------- A36 (consumer-019): T8 with a lease per input

LEASE_S = 0.25  # lk.attach's default lease
RETRACT_WITHIN_S = LEASE_S + 0.75  # WHY + 0.75 s: the gateway's lapse check and a step of the box, on loaded cores


def _rows_between(found: list[dict], start: float, end: float) -> list[dict]:
    return [row for row in found if start <= row["wall"] <= end]


def test_attach_lease_per_input(tmp_path):
    """CONTRACT §15 A36: the attach write lease is per exposed input, not per socket. The always-on form of a box
    with two exposed inputs, `a` (a planner's) and `b` (a guard's) (kpi/consumer/lease_parts.py, inproc, 100 Hz),
    and attach clients in processes of their own: writer_a writes `a` and writer_b writes `b`, at once, and both
    land (each input advances with its own writer's values while the other writer holds its lease), with no
    LeaseHeld refusal on either; T8 unchanged for two writers on one input: a rival writing `a` meanwhile is refused
    (its proxy's health error / LeaseHeld, naming writer_a) and none of its values ever lands; killing writer_b
    retracts `b` only (it reads None within the lease plus a margin, while `a` keeps advancing); killing writer_a
    then retracts `a`. WHY (consumer-019): one lease per socket made a guard in its own process a second writer,
    refused while the planner held the lease, so the always-on form forced planner and guard into one process."""
    like_keras()
    from kpi.support.parts import read_log

    serve = procs.Child("kpi.consumer.lease_parts", "serve", "--name", f"lease_{os.getpid()}", "--path",
                        str(tmp_path / "rows.jsonl"), log=tmp_path / "serve.log")
    writers: dict[str, procs.Child] = {}

    def writer(name: str, port: str, base: float) -> procs.Child:
        child = procs.Child("kpi.consumer.lease_parts", "write", "--address", address, "--port", port, "--base",
                            str(base), "--name", name, "--record", str(tmp_path / f"{name}.mcap"),
                            log=tmp_path / f"{name}.log")
        writers[name] = child
        child.wait_ready(timeout_s=60.0)
        return child

    try:
        address = serve.wait_ready(timeout_s=60.0)["attach"]["socket"]
        writer("writer_a", "a", 1000.0)
        writer("writer_b", "b", 2000.0)
        both_from = time.time() + 0.5
        time.sleep(3.0)
        both_until = time.time()
        writer("rival_a", "a", -100000.0)
        rival_from = time.time()
        time.sleep(2.0)
        writers["rival_a"].stop()
        rival_until = time.time()
        writers["writer_b"].signal(signal.SIGKILL)
        killed_b = time.time()
        time.sleep(2.5)
        writers["writer_a"].signal(signal.SIGKILL)
        killed_a = time.time()
        time.sleep(2.0)
    finally:
        for child in writers.values():
            child.stop()
        serve.stop()
    found = read_log(tmp_path / "rows.jsonl")
    both = _rows_between(found, both_from, both_until)
    a_values = {row["a"] for row in both if row["a"] is not None and 1000.0 < row["a"] < 2000.0}
    b_values = {row["b"] for row in both if row["b"] is not None and 2000.0 < row["b"] < 3000.0}
    assert len(both) >= 100, f"{len(both)} steps of the box while both wrote"
    assert len(a_values) >= 20 and len(b_values) >= 20, (
        f"both writers at once: {len(a_values)} values of writer_a landed on a, {len(b_values)} of writer_b on b "
        "(each needs its own lease)")
    for name in ("writer_a", "writer_b"):
        refused = [row for row in recorded(tmp_path / f"{name}.mcap", "health") if row["code"] == "LeaseHeld"]
        assert not refused, f"{name}'s writes were refused: {refused[:2]}"
    assert not [row for row in found if row["a"] is not None and row["a"] < 0], "a rival's value landed on a (T8)"
    rival = [row for row in recorded(tmp_path / "rival_a.mcap", "health") if row["code"] == "LeaseHeld"]
    assert any(row["level"] == "error" and "writer_a" in row["message"] for row in rival), \
        f"the rival on a got no LeaseHeld naming writer_a (T8, §4.5): {rival[:3]}"
    held = _rows_between(found, rival_from, rival_until)
    assert len({row["a"] for row in held if row["a"] is not None}) >= 10, \
        "writer_a stopped landing while the rival tried"
    after_b = _rows_between(found, killed_b + RETRACT_WITHIN_S, killed_a)
    assert after_b and all(row["b"] is None for row in after_b), \
        f"b was not retracted {RETRACT_WITHIN_S} s after writer_b died: {[row['b'] for row in after_b][:5]}"
    a_after_b = {row["a"] for row in after_b if row["a"] is not None}
    assert len(a_after_b) >= 10, f"a stopped advancing when writer_b died ({len(a_after_b)} values): retracted with b?"
    after_a = _rows_between(found, killed_a + RETRACT_WITHIN_S, float("inf"))
    assert after_a and all(row["a"] is None for row in after_a), \
        f"a was not retracted {RETRACT_WITHIN_S} s after writer_a died: {[row['a'] for row in after_a][:5]}"


# ---------------------------------------------------------------- A35 (consumer-018): the A/B against raw dora, timing

A35_PAIRS = 5  # CONTRACT §11: timing criteria run as >= 5 interleaved A/B pairs
A35_SECONDS = 20.0


@pytest.mark.timing
@pytest.mark.backend("dora")
def test_A35_boundary_within_noise_of_raw_dora(tmp_path, request):
    """CONTRACT §15 A35's exit, timing tier: on the same twin scenario (kpi/consumer/boundary_ab.py: the reference
    arm at 100 Hz on the fake bus, a 7 x 1000 plan every 0.5 s with lead 0, a subscriber in a process of its own),
    `target`'s states lost at the subscriber and its hand-off p99 are within noise of a raw-dora reference written
    in kpi (kpi/consumer/raw_arm_node.py: the same laws as raw dora 0.5.0 nodes, as bench_overhead's raw side is):
    A35_PAIRS interleaved pairs of A35_SECONDS, the upper edge of a bootstrap 95 % CI of the paired difference at
    most the raw side's own spread, or 0.1 % of states / 1 ms. Every number, the thread policies of both arms and
    what rtkit offers here go to build/kpi/a35_ab.json. Runs through kpi/run.py like every suite; run it outside a
    private PID namespace (bwrap --unshare-net only) to let rtkit grant (CONTRACT §15 A19, A35).
    WHY (consumer-018): the raw node lost 0-0.1 % of its states at a subscriber, like_keras's target 2-3 %, with a
    hand-off tail about 10x the raw node's, because only the RT thread was raised and nothing counted the loss."""
    timing_tier()
    like_keras()
    dora_backend()
    dora = request.getfixturevalue("private_dora")
    native_library("arm_controller")
    from kpi.consumer import boundary_ab

    oracle = subprocess.run([sys.executable, str(KPI / "blackbox" / "rt_ladder_child.py"), "--oracle"],
                            env=subprocess_env(), capture_output=True, text=True, timeout=60)
    offered = json.loads(oracle.stdout.splitlines()[-1]) if oracle.stdout.strip() else {"error": oracle.stderr[-500:]}
    runs: dict[str, list[dict]] = {"raw": [], "like_keras": []}
    for index in range(A35_PAIRS):
        order = ("raw", "like_keras") if index % 2 == 0 else ("like_keras", "raw")  # interleaved
        for side in order:
            directory = tmp_path / f"{index}_{side}"
            # WHY a 5 s board watchdog on both sides: the A/B compares the boundary, and a stalled raw Python arm
            # latched its boards in timeout mode on the loaded rig (ruler lane, 1 leg of 5), ending its plans
            config = cells.config(directory, lockstep=False, watchdog_s=5.0)
            name = f"ab_{side.replace('like_keras', 'lk')}_{index}_{os.getpid()}"
            if side == "raw":
                runs[side].append(boundary_ab.run_raw(dora, directory, A35_SECONDS, config, name))
            else:
                runs[side].append(boundary_ab.run_lk(directory, A35_SECONDS, config, name, kpi_cpus()))
    measured = all(run["states"]["received"] >= 0.5 * A35_SECONDS * 100
                   and run["handoff"]["measured"] >= 0.5 * A35_SECONDS * 2 for side_runs in runs.values()
                   for run in side_runs)
    lost = boundary_ab.within_noise([run["states"]["lost_fraction"] for run in runs["like_keras"]],
                                    [run["states"]["lost_fraction"] for run in runs["raw"]], boundary_ab.LOST_FLOOR)
    handoff = boundary_ab.within_noise([run["handoff"]["p99_s"] for run in runs["like_keras"]],
                                       [run["handoff"]["p99_s"] for run in runs["raw"]],
                                       boundary_ab.HANDOFF_FLOOR_S) if measured else None
    report = {"tier": "timing" if os.environ.get("KPI_TIER") == "timing" else "not the timing tier",
              "host_note": os.environ.get("KPI_HOST_NOTE", ""), "rtkit_offered": offered,
              "pairs": A35_PAIRS, "seconds": A35_SECONDS, "states_lost_fraction": lost, "handoff_p99_s": handoff,
              "runs": runs}
    BUILD.mkdir(parents=True, exist_ok=True)
    (BUILD / "a35_ab.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    for side, side_runs in runs.items():
        for run in side_runs:
            assert run["states"]["received"] >= 0.5 * A35_SECONDS * 100, f"{side}: too few states: {run['states']}"
            assert run["handoff"]["measured"] >= 0.5 * A35_SECONDS * 2, \
                f"{side}: too few plans measured: {run['handoff']}"
    assert lost["within"], (f"states lost at the subscriber: like_keras {lost['like_keras']}, raw {lost['raw']}; "
                            f"the CI's upper edge {lost['upper_95_of_mean_difference']:.4f} > band {lost['band']:.4f}")
    assert handoff["within"], (f"hand-off p99 (s): like_keras {handoff['like_keras']}, raw {handoff['raw']}; the "
                               f"CI's upper edge {handoff['upper_95_of_mean_difference']:.4f} s > band "
                               f"{handoff['band']:.4f} s")
