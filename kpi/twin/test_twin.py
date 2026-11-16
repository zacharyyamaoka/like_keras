"""Criterion 10, testability with twins: four properties, scored only together with "no wall-clock sleep here".

1 twin swap         real bus <-> twin bus is one config line, no component line (the hardware-station pattern)
2 deterministic     ten lockstep runs of arm_cell on the twin give identical output hashes, at >= 10x real time
                    (on CPU time in the default tier; on the wall clock in the timing tier, its own test id)
3 replay            replaying a recording through the same ports reproduces the run bit for bit
4 isolation         one component tested alone: eager calls on plain values, no backend, no thread

No test in this file may sleep on the wall clock (kpi/blackbox scans it): a twin run that needs a sleep fails
criterion 10 by its own definition.
"""

from __future__ import annotations

import difflib
import hashlib
import json
import time

import numpy as np
import pytest

from kpi.support import cells, mcap
from kpi.support.lk import BUILD, KPI, like_keras

ARM = KPI / "tasks" / "arm_cell"
UNTIL_S = 4.0


def test_twin_swap_is_one_config_line(tmp_path):
    """arm_controller.toml (twin) and arm_controller.real.toml differ by one line; both build the same System."""
    twin = (ARM / "arm_controller.toml").read_text().splitlines()
    real = (ARM / "arm_controller.real.toml").read_text().splitlines()
    changed = [line for line in difflib.unified_diff(twin, real, lineterm="", n=0) if line[:1] in "+-" and line[:3] not in ("+++", "---")]
    assert len(changed) == 2, changed  # one line out, one line in
    like_keras()
    from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy

    for config in ("arm_controller.toml", "arm_controller.real.toml"):
        system = ArmControllerPy(config=ARM / config)
        assert {"trajectory", "state", "command"} <= set(dir(system)), config


def stream_hash(path) -> str:
    """sha256 over every recorded frame of the exposed ports and crossing edges, in file order."""
    digest = hashlib.sha256()
    recording = mcap.read(path)
    for message in recording.messages:
        digest.update(recording.channels[message.channel_id].topic.encode())
        digest.update(message.data)
    return digest.hexdigest()


def ten_lockstep_runs(placement: str, tmp_path, clock) -> tuple[list[str], list[float]]:
    """Ten seeded lockstep runs of arm_cell on the twin: each run's output-stream hash, and its speed against real
    time (UNTIL_S logical seconds over the seconds `clock` counted for start())."""
    if placement == "same_box" and not (BUILD / "libarm_controller.so").exists():
        pytest.skip("build/kpi/libarm_controller.so not built")
    hashes, speeds = [], []
    for run in range(10):
        directory = tmp_path / f"run{run}"
        directory.mkdir()
        dep = cells.deployment(placement, directory, lockstep=True)
        dep.record(directory / "run.mcap")
        started = clock()
        dep.start(until=UNTIL_S)
        speeds.append(UNTIL_S / (clock() - started))
        dep.stop()
        hashes.append(stream_hash(directory / "run.mcap"))
    return hashes, speeds


@pytest.mark.parametrize("placement", ["fallback", "same_box"])
def test_ten_runs_hash_identical_at_10x_cpu_time(placement, tmp_path):
    """Default tier: ten seeded lockstep runs give identical output-stream hashes, each at >= 10x real time on this
    process's CPU time. WHY CPU time here: on the powered rig the lanes run at nice 19 on two shared cores, where
    the wall clock measures how long the scheduler starved the process, not what like_keras costs (it read
    1.4x-8.9x in runs that measured 22-60x on CPU time, ruler-002 §1). The wall-clock check is the timing-tier
    twin below."""
    hashes, speeds = ten_lockstep_runs(placement, tmp_path, time.process_time)
    assert len(set(hashes)) == 1, f"{len(set(hashes))} different output streams in 10 runs"
    assert min(speeds) >= 10.0, f"slowest run at {min(speeds):.1f}x real time on CPU time (needs >= 10x)"


@pytest.mark.timing
@pytest.mark.parametrize("placement", ["fallback", "same_box"])
def test_ten_runs_hash_identical_at_10x_wall_clock(placement, tmp_path):
    """Timing tier (rack unpowered or the fixed cloud VM): the same ten runs, identical hashes, each at >= 10x real
    time on the wall clock."""
    hashes, speeds = ten_lockstep_runs(placement, tmp_path, time.perf_counter)
    assert len(set(hashes)) == 1, f"{len(set(hashes))} different output streams in 10 runs"
    assert min(speeds) >= 10.0, f"slowest run at {min(speeds):.1f}x real time (needs >= 10x)"


def test_replay_reproduces_the_run(tmp_path):
    """Run 1 records; run 2 swaps TargetFile for lk.replay(run 1's recording) through the same port: the command
    stream is bit-identical."""
    lk = like_keras()
    first = tmp_path / "first"
    first.mkdir()
    dep = cells.deployment("fallback", first, lockstep=True)
    dep.record(first / "run.mcap")
    dep.start(until=UNTIL_S)
    dep.stop()

    from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy
    from kpi.tasks.arm_cell.planner import PickPlanner

    second = tmp_path / "second"
    second.mkdir()
    s = lk.System("pick_cell")
    targets = s.add(lk.replay(first / "run.mcap"), "targets")
    arm = s.add(ArmControllerPy(config=cells.config(second, lockstep=True)), "arm")
    planner = s.add(PickPlanner(), "planner", target=targets.targets__target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    s.expose(arm.state, arm.command)
    replayed = lk.Deployment(s, realtime=False)
    replayed.place(s.arm, process="arm-controller", hz=100)
    replayed.place(s.targets, hz=2)
    replayed.place(s.planner)
    replayed.record(second / "run.mcap", arm.command)
    replayed.start(until=UNTIL_S)
    replayed.stop()
    original = [m.data for m in mcap.read(first / "run.mcap").on("command")]
    again = [m.data for m in mcap.read(second / "run.mcap").on("command")]
    assert original and again == original, "replay did not reproduce the command stream bit for bit"


def test_component_alone():
    """A component tested by feeding its inputs, eagerly: no backend, no process, no thread; now() is 0.0."""
    like_keras()
    from robot_msgs.control import ServoState

    from kpi.tasks.arm_cell.arm_controller_py import JtcPy
    from kpi.tasks.arm_cell.jtc_law import JtcCore
    from kpi.tasks.arm_cell.plan import JOINTS, PickTarget
    from kpi.tasks.arm_cell.planner import PickPlanner

    target = PickTarget(name=JOINTS, position=json.loads((ARM / "targets.jsonl").read_text().splitlines()[0])["position"])
    state = ServoState(name=JOINTS, position=np.zeros(7), velocity=np.zeros(7), fault=np.zeros(7, dtype=np.int64))
    planner = PickPlanner()
    plan = planner(target=target, state=state)
    assert plan.header.stamp == 0.05 and plan.positions.shape == (1000, 7)
    assert planner(target=target, state=None) is None, "no fresh state: nothing sent"
    jtc = JtcPy()
    jtc.activate()
    command = jtc(trajectory=plan, state=state)
    expected = JtcCore()
    expected.replace(plan)
    position, _, _ = expected.sample(0.0, state)
    assert np.array_equal(command.position, position)
