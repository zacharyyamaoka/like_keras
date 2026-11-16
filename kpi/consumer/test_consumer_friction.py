"""Criterion 1 (consumer fit), ruler M0.3 (CONTRACT §15 A27): the first consumer's friction with the controller's
life cycle and the swap.

Each test judges one decision of §15 and counts as a companion of the consumer test it extends (kpi/score.py: that
test counts only when its companion passes too):
  A20 test_period_in_activate            T7: refused in activate before any step, when one exchange cannot fit a tick
  A25 test_fresh_input_counts_arrivals   T1: Python and C++ count the same arrivals (the swap changes nothing)
  A26 test_T2_export_and_push_apart      T2 (timing tier): the send split into the export and lk_push
  A30 test_log_reaches_readers_at_its_step (M0.4)  T1: a native log line reaches a reader as a Python twin's does
The C++ boxes (period_probe.cpp, arrivals.cpp, log_steps.cpp) are built on demand (kpi/support/lk.feature_library):
until a decision lands, its box does not compile, or the library does not do it, and its test FAILS, never skips.
"""

from __future__ import annotations

import ctypes
import json
import time

import numpy as np
import pytest

from kpi.support import cells
from kpi.support.lk import BUILD, KPI, backend, feature_library, kpi_cpus, like_keras, native_library

COMBINATIONS = ["python-inproc", "python-dora", "cpp-inproc", "cpp-dora"]


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
@pytest.mark.parametrize("placement", ["same_box"])
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
