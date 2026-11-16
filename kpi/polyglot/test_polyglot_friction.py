"""Criterion 8 (Python + C++ reach), ruler M0.3 (CONTRACT §15 A27) and M0.4: the first consumer's friction on the
C++ side.

Each test judges one decision of §15 and counts with a part kpi/score.py already scores (a part passes only when
every one of its tests passes):
  A16 test_T10_trajectory_joint_names                    part 4, the codec: a JointTrajectory view's joint names
  A17 test_converted_input_keeps_a_plan_off_the_rt_thread  part 1, the mixed chain: In<T, Converted>
  A23 test_optional_verdict_input                       part 1, the mixed chain: an optional C++ In, a guard's verdict
  A23 test_optional_queue_input (M0.4)                  part 1, the mixed chain: an optional C++ queue In (A30)
  A31 test_conversion_sees_its_owner (M0.5)             part 1, the mixed chain: from(const T&, const Owner&)
Their C++ boxes (joint_names.cpp, kept_plan.cpp, verdict.cpp, optional_queue.cpp, owner_conversion.cpp) are built on
demand (kpi/support/lk.feature_library): until a decision lands, its box does not compile and its test FAILS, never skips.
M0.4 (CONTRACT §15 A29) moves A16's source to module level (friction_parts.GoldenPlan), so its dora case can start.
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import time

import numpy as np
import pytest

from kpi.golden import make_golden
from kpi.support.lk import backend, feature_library, kpi_cpus, like_keras, python, subprocess_env


def rows(path) -> list[dict]:
    try:
        return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    except FileNotFoundError:
        return []


# ---------------------------------------------------------------- A16 (consumer-001)


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_T10_trajectory_joint_names(backend_name, tmp_path, request):
    """CONTRACT §15 A16: the C++ JointTrajectory view carries the plan's joint names (a std::string_view per joint,
    into the received buffer, valid for the step). The golden plan (a 2-byte and a 3-byte UTF-8 name) goes Python ->
    C++ in one process and across processes; the box sends back each joint's name with its last position. WHY
    (consumer-001): a C++ controller could only map a plan by column order while its Python peer mapped it by name,
    so the same plan moved different boards in same_box and fallback.
    WHY GoldenPlan, a module-level source (M0.4, CONTRACT §15 A29, ruler-008): under dora the source runs in a process
    of its own, which rebuilds it as module:qualname (§4.1); kpi/golden/instances.py's emitter() defines its class
    inside a function, so the dora case was refused before it started. Both cases send the same golden plan."""
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    chosen = backend(backend_name)
    library = feature_library("joint_names")
    from kpi.polyglot.friction_parts import GoldenPlan, JointStateLog

    s = lk.System(f"plan_names_{backend_name}")
    emit = s.add(GoldenPlan(), "emit")
    box = s.add(lk.native(library, config={"unused": 0}), "box", trajectory=emit.value)
    s.add(JointStateLog(path=str(tmp_path / "names.jsonl")), "sink", value=box.names)
    dep = lk.Deployment(s, backend=chosen, realtime=False)
    dep.place(s.emit, process="emit", hz=10)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    try:
        dep.start(until=0.45)
    finally:
        dep.stop()
    plan = make_golden.INSTANCES["trajectory/JointTrajectory"]
    found = rows(tmp_path / "names.jsonl")
    assert [row["name"] for row in found] == [list(plan["joint_names"])], found
    assert found[0]["position"] == [float(x) for x in np.asarray(plan["positions"])[-1]], found


# ---------------------------------------------------------------- A17 (consumer-002)

COUNTS = ("made", "made_on_rt", "freed", "freed_on_rt", "taken", "taken_broken", "kept_checks", "kept_broken",
          "kept_id")  # kept_plan.cpp's Count, in order: int64 each in the file counts_path


def test_converted_input_keeps_a_plan_off_the_rt_thread(tmp_path):
    """CONTRACT §15 A17: In<rm::JointTrajectory, KeptPlan>. A Python planner pushes a new 7 x 100 plan every step at
    20 Hz into a C++ box at 100 Hz (same_box, real time: only there are the pushing thread and the RT thread two
    threads). The box adopts the first plan by moving the converted data out of the value it is handed, keeps it
    for the whole run while every later plan is taken and refused, and frees it in deactivate. Then:
    every plan was converted by KeptPlan::from and none on the RT thread; none was freed on the RT thread; every
    plan converted was freed by the time the deployment stopped (retired through the graveyard, nothing leaked);
    every plan take() handed over, and the kept one on every step, read intact (each plan is poisoned when freed);
    and the kept plan outlived at least 20 later plans.
    WHY (consumer-002): a latest view died under a component still using it, so the port copied and validated
    every plan on the RT thread, the work the raw node kept on its I/O thread."""
    lk = like_keras()
    library = feature_library("kept_plan")
    from kpi.polyglot.friction_parts import PlanSource

    s = lk.System("kept_plan")
    source = s.add(PlanSource(), "source")
    counts_path = tmp_path / "counts.bin"
    s.add(lk.native(library, config={"counts_path": str(counts_path)}), "box", trajectory=source.trajectory)
    dep = lk.Deployment(s)
    dep.place(s.box, process="box", hz=100, cpus=kpi_cpus())
    dep.place(s.source, hz=20)
    dep.start()
    try:
        time.sleep(4.0)
    finally:
        dep.stop()
    counts = dict(zip(COUNTS, struct.unpack(f"<{len(COUNTS)}q", counts_path.read_bytes()[:8 * len(COUNTS)])))
    assert counts["made"] >= 30, counts
    assert counts["made_on_rt"] == 0, f"Converted::from ran on the RT thread: {counts}"
    assert counts["freed_on_rt"] == 0, f"a converted plan was freed on the RT thread: {counts}"
    assert counts["freed"] == counts["made"], f"converted plans not retired once the deployment stopped: {counts}"
    assert counts["taken"] >= 10 and counts["taken_broken"] == 0, f"take() handed over a plan not intact: {counts}"
    assert counts["kept_id"] >= 1 and counts["made"] - counts["kept_id"] >= 20, f"the kept plan outlived too few: {counts}"
    assert counts["kept_checks"] >= 100 and counts["kept_broken"] == 0, f"the kept plan did not stay intact: {counts}"


# ---------------------------------------------------------------- A23 (consumer-008)

GUARD_TIMEOUT_S = 0.3
BOX_TICK_S = 0.01
GUARD_TICK_S = 0.05


def _guarded_run(placement: str, directory, guard: dict | None) -> list[dict]:
    """verdict.cpp's box at 100 Hz, lockstep for 2 s, its verdict input wired to a VerdictGuard(**guard) at 20 Hz,
    or left unwired (guard None). The box's commands, in step order."""
    lk = like_keras()
    library = feature_library("verdict")
    from kpi.polyglot.friction_parts import JointStateLog, VerdictGuard

    directory.mkdir(parents=True, exist_ok=True)
    s = lk.System(f"guarded_{placement}_{'unwired' if guard is None else 'guarded'}_{directory.name}")
    if guard is None:
        arm = s.add(lk.native(library, config={"guard_timeout_s": GUARD_TIMEOUT_S, "step_rad": 0.001}), "arm")
    else:
        verdicts = s.add(VerdictGuard(**guard), "guard")
        arm = s.add(lk.native(library, config={"guard_timeout_s": GUARD_TIMEOUT_S, "step_rad": 0.001}), "arm",
                    verdict=verdicts.verdict)
    s.add(JointStateLog(path=str(directory / "command.jsonl")), "sink", value=arm.command)
    if placement == "target":
        from like_keras.backends import dora

        dep = lk.Deployment(s, backend=dora, realtime=False)
    else:
        dep = lk.Deployment(s, realtime=False)
    dep.place(s.arm, process="arm", hz=1 / BOX_TICK_S)
    if guard is not None:
        dep.place(s.guard, process="guard", hz=1 / GUARD_TICK_S)
    dep.place(s.sink, process="sink")
    plan = dep.describe()
    ports = next(component["ports"] for path, component in plan["components"].items() if path.endswith(".arm"))
    assert "verdict" in ports and ports["verdict"]["dir"] == "in", f"describe() does not list the optional input: {ports}"
    try:
        dep.start(until=2.0)
    finally:
        dep.stop()
    return sorted(rows(directory / "command.jsonl"), key=lambda row: row["stamp"])


def _first_hold(commands: list[dict]) -> float | None:
    """The stamp of the first step that did not move, after which no step moved again; None if it never held."""
    positions = [row["position"][0] for row in commands]
    moved = [later > earlier for earlier, later in zip(positions, positions[1:])]
    if all(moved):
        return None
    first = moved.index(False)
    assert not any(moved[first:]), "the box moved again after it held"
    return commands[first + 1]["stamp"]


@pytest.mark.parametrize("placement", ["same_box", "target"])
def test_optional_verdict_input(placement, tmp_path, request):
    """CONTRACT §15 A23: a C++ input may be optional (InOpts{.optional = true}): unwired it is legal, reads nullptr,
    and describe() lists it; a guard's verdict is a robot_msgs DiagnosticRecord (header.stamp = the judged state's
    stamp, level error = stop). The guard deadman (verdict.cpp), lockstep, in one process and across processes:
    unwired, the box moves on every step; wired to a guard that stops confirming at 1 s, it holds within
    guard_timeout_s + one tick of the last confirmation; wired to a guard that sends level error at 0.5 s, it holds
    from the next step. WHY (consumer-008): the raw node's guard deadman and stop requests could not be ported: a
    C++ input had to be wired in every cell, even with no guard, and no v0 type carried a verdict."""
    if placement == "target":
        request.getfixturevalue("private_dora")
    unwired = _guarded_run(placement, tmp_path / "unwired", None)
    assert len(unwired) >= 150 and _first_hold(unwired) is None, "unwired, the box did not move on every step"

    silent = _guarded_run(placement, tmp_path / "silent", {"confirm_until_s": 1.0})
    last_confirmation = max(index * GUARD_TICK_S for index in range(40) if index * GUARD_TICK_S < 1.0 - 1e-9)
    held = _first_hold(silent)
    assert held is not None, "a guard that stopped confirming did not stop the box"
    assert last_confirmation + GUARD_TIMEOUT_S < held <= last_confirmation + GUARD_TIMEOUT_S + BOX_TICK_S + 1e-6, (
        f"held at {held:.3f} s; the last confirmation judged {last_confirmation:.3f} s, so the hold is due after "
        f"{last_confirmation + GUARD_TIMEOUT_S:.3f} s and within one tick of it")

    stopped = _guarded_run(placement, tmp_path / "stopped", {"stop_at_s": 0.5})
    held = _first_hold(stopped)
    assert held is not None and 0.5 < held <= 0.5 + BOX_TICK_S + 1e-6, \
        f"a level error verdict sent at 0.5 s held the box at {held}"


# ---------------------------------------------------------------- A23 for a queue input (M0.4; library verifier, round 1)

OPTIONAL_QUEUE_TIMEOUT_S = 240.0  # WHY this long: a dora dataflow starts in tens of seconds on the loaded cores


def _optional_queue_run(backend_name: str, wiring: str, directory, library, dora) -> dict:
    """kpi/polyglot/optional_queue_child.py in a process of its own: its exit status, its reports and the box's
    ports. A run that hangs is killed (and its dataflow stopped by name) and reported as such."""
    directory.mkdir(parents=True, exist_ok=True)
    name = f"optq_{backend_name}_{wiring}_{os.getpid()}"
    command = [python(), "-m", "kpi.polyglot.optional_queue_child", "--library", str(library), "--backend",
               backend_name, "--wiring", wiring, "--directory", str(directory), "--name", name]
    with open(directory / "child.log", "wb") as log:
        child = subprocess.Popen(command, env=subprocess_env(), stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            returncode = child.wait(timeout=OPTIONAL_QUEUE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=30)
            returncode = "hung"
    if dora is not None and returncode != 0:  # a launcher that died or hung leaves its dataflow running
        subprocess.run([dora.cli, "stop", "--name", name], cwd=dora.workdir, env=subprocess_env(),
                       capture_output=True, timeout=60)
    ports = directory / "ports.json"
    return {"returncode": returncode, "log": (directory / "child.log").read_text(errors="replace")[-1500:],
            "ports": json.loads(ports.read_text()) if ports.exists() else None,
            "idle": [row["position"] for row in rows(directory / "idle.jsonl")],
            "gated": [row["position"] for row in rows(directory / "gated.jsonl")]}


def _stepped_every_tick_reading_nullptr(reports: list, what: str) -> None:
    assert len(reports) >= 10, f"{what}: {len(reports)} steps in 1.05 s at 10 Hz, lockstep (11 ticks): {reports}"
    assert [report[0] for report in reports] == [float(step) for step in range(1, len(reports) + 1)], reports
    assert all(report[1] == 1.0 for report in reports), f"{what}: an input nothing writes did not read nullptr: {reports}"


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_optional_queue_input(backend_name, tmp_path, request):
    """CONTRACT §15 A23 for a QUEUE input (InOpts{.queue = 4, .optional = true}), as A30 ratifies it: unwired is
    legal and reads nullptr, so an optional queue input nothing writes never holds the gather of §5.2. One box
    (optional_queue.cpp), lockstep at 10 Hz for 1.05 s, in one process and across processes:
      - `idle`, an optional queue input neither wired nor exposed inside the box: steps every tick, reads nullptr;
      - `gated`, an optional queue input the box exposes: left unwired by the deployment, steps every tick and reads
        nullptr; wired to a source that sends on its steps 1, 2 and 6, it gates as a queue does: exactly one step
        per message, reading that message (values 1, 2, 6).
    describe() lists the exposed one as an input. Each deployment runs in a child process, so a crash fails this
    test only. WHY (library verifier, round 1): left unwired and unexposed, the box crashed on its first step (a
    SIGSEGV in the gather, on a queue with no ring); exposed and left unwired, it never stepped (0 ticks)."""
    like_keras()
    backend(backend_name)
    dora = request.getfixturevalue("private_dora") if backend_name == "dora" else None
    library = feature_library("optional_queue")
    runs = {wiring: _optional_queue_run(backend_name, wiring, tmp_path / wiring, library, dora)
            for wiring in ("unwired", "wired")}
    for wiring, run in runs.items():
        assert run["returncode"] == 0, f"{wiring}: the deployment exited {run['returncode']}:\n{run['log']}"
        assert run["ports"] is not None and run["ports"].get("extra", {}).get("dir") == "in", \
            f"{wiring}: describe() does not list the optional input: {run['ports']}"
        _stepped_every_tick_reading_nullptr(run["idle"], f"{wiring}: idle (neither wired nor exposed)")
    _stepped_every_tick_reading_nullptr(runs["unwired"]["gated"], "gated, exposed and left unwired")
    expected = [[1.0, 0.0, 1.0], [2.0, 0.0, 2.0], [3.0, 0.0, 6.0]]  # steps, read nullptr, the value it read
    assert runs["wired"]["gated"] == expected, (
        f"gated, wired to a source that sent 1, 2 and 6: one step per message expected {expected}, "
        f"got {runs['wired']['gated']}")


# ---------------------------------------------------------------- A31 (consumer-014; ruler M0.5)

OWNER_GAINS = {"box_a": 2.0, "box_b": 3.0}


def test_conversion_sees_its_owner(tmp_path):
    """CONTRACT §15 A31: a conversion may see its owner, `static Converted from(const T&, const Owner&)`, called only
    between activate() and deactivate(), reading what the owner set before activate() returned. Two boxes of one
    .so (owner_conversion.cpp) in one process (same_box, real time, each its own RT thread at 100 Hz), with gains 2
    and 3, both fed by one Python writer at 50 Hz for 3 s: each converts every value with its own config (its gain,
    a param, and its offset 1000 x gain, built in activate after a 0.3 s pause), so every echo reads converted =
    raw x gain + 1000 x gain, to 1e-9; no conversion ran outside activate()..deactivate() (counted by the box, after
    deactivate as well). WHY (consumer-014): from(const T&) could not reach its component, so the chain's config
    went into a process-wide global and a second chain in the process had to be refused."""
    lk = like_keras()
    library = feature_library("owner_conversion")
    from kpi.polyglot.friction_parts import JointStateLog, StepCount

    s = lk.System("owners")
    source = s.add(StepCount(), "source")
    for box, gain in OWNER_GAINS.items():
        native = lk.native(library, config={"gain": gain, "counts_path": str(tmp_path / f"{box}.counts")})
        echo = s.add(native, box, value=source.value)
        s.add(JointStateLog(path=str(tmp_path / f"{box}.jsonl")), f"{box}_sink", value=echo.echo)
    dep = lk.Deployment(s)
    for box in OWNER_GAINS:
        dep.place(getattr(s, box), hz=100, cpus=kpi_cpus())  # one process: inproc runs everything in this one
    dep.place(s.source, hz=50)
    dep.place(*(getattr(s, f"{box}_sink") for box in OWNER_GAINS))
    dep.start()
    try:
        time.sleep(3.0)
    finally:
        dep.stop()
    for box, gain in OWNER_GAINS.items():
        echoed = rows(tmp_path / f"{box}.jsonl")
        assert len(echoed) >= 50, f"{box}: {len(echoed)} values converted and taken in 3 s"
        outside = [row["position"] for row in echoed if row["position"][2] != 0.0]
        assert not outside, f"{box}: converted outside activate()..deactivate(): {outside[:3]}"
        wrong = [(raw, converted) for raw, converted, _ in (row["position"] for row in echoed)
                 if converted is None or abs(converted - (raw * gain + 1000.0 * gain)) > 1e-9]
        assert not wrong, f"{box} (gain {gain}): (raw, converted) not converted with its own config: {wrong[:3]}"
        converted, after = struct.unpack("<2q", (tmp_path / f"{box}.counts").read_bytes()[:16])
        assert converted >= len(echoed) and after == 0, f"{box}: {converted} conversions, {after} outside the window"
