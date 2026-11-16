"""Criterion 8 (Python + C++ reach), ruler M0.3 (CONTRACT §15 A27): the first consumer's friction on the C++ side.

Each test judges one decision of §15 and counts with a part kpi/score.py already scores (a part passes only when
every one of its tests passes):
  A16 test_T10_trajectory_joint_names                    part 4, the codec: a JointTrajectory view's joint names
  A17 test_converted_input_keeps_a_plan_off_the_rt_thread  part 1, the mixed chain: In<T, Converted>
  A23 test_optional_verdict_input                       part 1, the mixed chain: an optional C++ In, a guard's verdict
Their C++ boxes (joint_names.cpp, kept_plan.cpp, verdict.cpp) are built on demand (kpi/support/lk.feature_library):
until a decision lands, its box does not compile and its test FAILS, never skips.
"""

from __future__ import annotations

import json
import struct
import time

import numpy as np
import pytest

from kpi.golden import make_golden
from kpi.support.lk import backend, feature_library, kpi_cpus, like_keras


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
    so the same plan moved different boards in same_box and fallback."""
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    chosen = backend(backend_name)
    library = feature_library("joint_names")
    from kpi.golden.instances import emitter
    from kpi.polyglot.friction_parts import JointStateLog

    s = lk.System(f"plan_names_{backend_name}")
    emit = s.add(emitter(lk, "trajectory/JointTrajectory")(), "emit")
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
