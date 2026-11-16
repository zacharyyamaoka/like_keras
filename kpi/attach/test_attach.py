"""Criterion 11, network attach (the IP-camera idea), same host in v0: four properties.

The always-on form (kpi/tasks/arm_cell/always_on.py: the Python controller on the twin bus, exposing trajectory,
state and command) runs as its own process; every client is a separate process started at least 1 s later.
1 list          `lk ls <address>` lists the exposed ports, types, schema hashes, policies and the dora version in 1 s
2 attach        a late client receives `state` within 1 s and its `trajectory` runs, under the write lease
3 harmless      a client killed or stalled does not hurt the node (against a no-client window taken just before; the
                absolute tick rate is the timing tier's); a slow one is dropped; a new one gets the cached latest
                state at once
4 explicit      attaching needs an explicit address; nothing is discovered
"""

from __future__ import annotations

import itertools
import json
import os
import signal
import subprocess
import time

import pytest

from kpi.golden import lkf
from kpi.support import cells, procs
from kpi.support.lk import KPI, like_keras, lk_command, kpi_cpus
from kpi.tasks.arm_cell.fake_bus import EVENT_POSITION, FakeBoards

TYPES = {"trajectory": "trajectory/JointTrajectory", "state": "control/ServoState", "command": "control/ServoCommand"}


@pytest.fixture
def node(tmp_path):
    like_keras()
    child = procs.Child("kpi.tasks.arm_cell.always_on", "--name", f"attach_{os.getpid()}", "--language", "py",
                        "--config", json.dumps(cells.config(tmp_path, lockstep=False)), "--cpus", str(kpi_cpus()[0]),
                        log=tmp_path / "node.log")
    ready = child.wait_ready()
    time.sleep(1.0)  # every client starts at least 1 s after the node (criterion 11)
    child.address = ready["attach"]["socket"]
    child.boards = tmp_path / "arm.boards"
    yield child
    assert child.stop() == 0, "the node did not exit cleanly"


def ticks(boards_path) -> list[float]:
    boards = FakeBoards(boards_path)
    events = boards.events()
    return list(events["wall"][(events["kind"] == EVENT_POSITION) & (events["t"] > 0)])


WINDOW_S = 3.0


def tick_window(boards_path, seconds: float = WINDOW_S) -> tuple[int, float]:
    """(ticks, the longest gap between two ticks in s) of the node over the next `seconds` of wall time."""
    before = len(ticks(boards_path))
    time.sleep(seconds)
    after = ticks(boards_path)
    window = after[before:]
    gaps = [later - earlier for earlier, later in itertools.pairwise(window)]
    return len(window), max(gaps, default=float("inf"))


def bad_clients(node, tmp_path) -> dict:
    """test_clients_cannot_hurt_the_node's run: a no-client window, then a stalled and a killed attached planner and a
    window with them, then a late client. The ticks and longest gaps of both windows, and how long the late client
    waited for the cached latest state."""
    like_keras()
    from kpi.support.parts import read_log

    quiet_ticks, quiet_gap = tick_window(node.boards)  # the baseline: the same node, the same host, no client yet
    stalled = procs.Child("kpi.tasks.arm_cell.attached_planner", "--address", node.address, "--name", "stalled")
    killed = procs.Child("kpi.tasks.arm_cell.attached_planner", "--address", node.address, "--name", "killed")
    try:
        stalled.wait_ready()
        killed.wait_ready()
        stalled.signal(signal.SIGSTOP)  # never reads its socket again: its send queue fills; it must be dropped
        killed.signal(signal.SIGKILL)
        bad_ticks, bad_gap = tick_window(node.boards)
        log = tmp_path / "late.jsonl"
        lk = like_keras()
        from kpi.support.parts import StateLog

        s = lk.System("late")
        arm = s.add(lk.attach(node.address), "arm")
        s.add(StateLog(path=str(log)), "log", value=arm.state)
        dep = lk.Deployment(s)
        dep.place(arm, hz=100)
        dep.place(s.log)
        started = time.time()
        dep.start()
        try:
            first = procs.wait_for(lambda: read_log(log), 2.0, "the cached latest state")[0]
        finally:
            dep.stop()
    finally:
        stalled.signal(signal.SIGCONT)
        stalled.stop()
        killed.stop()
    return {"quiet_ticks": quiet_ticks, "quiet_gap_s": quiet_gap, "bad_ticks": bad_ticks, "bad_gap_s": bad_gap,
            "late_client_s": first["wall"] - started}


def test_list_ports_within_1s(node):
    started = time.monotonic()
    listing = subprocess.run(lk_command("ls", node.address), capture_output=True, text=True, timeout=5)
    elapsed = time.monotonic() - started
    assert listing.returncode == 0, listing.stderr
    assert elapsed <= 1.0, f"lk ls took {elapsed:.2f} s"
    for port, type_name in TYPES.items():
        assert port in listing.stdout and type_name in listing.stdout, f"{port} ({type_name}) not listed"
        assert f"{lkf.schema_hash(type_name):016x}" in listing.stdout, f"{port}'s schema hash not listed"
    assert "latest" in listing.stdout and "0.5.0" in listing.stdout, "policy or dora version not listed"


def test_attach_receive_and_write(node, tmp_path):
    lk = like_keras()
    from kpi.support.parts import StateLog, observe, read_log

    s = lk.System("listener")
    arm = s.add(lk.attach(node.address), "arm")
    s.add(StateLog(path=str(tmp_path / "state.jsonl")), "log", value=arm.state)
    dep = lk.Deployment(s)
    dep.place(arm, hz=100)
    dep.place(s.log)
    started = time.time()
    dep.start()
    try:
        first = procs.wait_for(lambda: read_log(tmp_path / "state.jsonl"), 3.0, "the first state")[0]
    finally:
        dep.stop()
    assert first["wall"] - started <= 1.0, f"state arrived {first['wall'] - started:.2f} s after attaching"
    log = tmp_path / "commands.jsonl"
    observer = observe(node.address, log)
    planner = procs.Child("kpi.tasks.arm_cell.attached_planner", "--address", node.address)
    try:
        planner.wait_ready()
        targets = [json.loads(line)["position"] for line in open(KPI / "tasks" / "arm_cell" / "targets.jsonl") if line.strip()]

        def reached():
            return any(row["position"] and max(abs(a - b) for a, b in zip(row["position"], target)) < 1e-6
                       for row in read_log(log) for target in targets)

        procs.wait_for(reached, 5.0, "the attached planner's trajectory running")
    finally:
        planner.stop()
        observer.stop()


def test_clients_cannot_hurt_the_node(node, tmp_path):
    """A stalled and a killed attached planner cost the node nothing it would not have lost with no client at all:
    against a no-client window of the same length taken just before, at least 0.9 x its ticks, and a longest gap
    no more than its own + 20 ms. A new client gets the cached latest state at once.
    WHY a baseline (CONTRACT §15 A14, ruler-006 §1): on the powered rig the node alone ticked 214-269 times in 3 s
    with 36-113 ms gaps, so absolute bounds measured the host, not the clients. They are the timing twin's
    (test_clients_cannot_hurt_the_node_unloaded)."""
    run = bad_clients(node, tmp_path)
    assert run["bad_ticks"] >= 0.9 * run["quiet_ticks"], \
        f"the node ticked {run['bad_ticks']} times in {WINDOW_S} s with bad clients, {run['quiet_ticks']} with none"
    assert run["bad_gap_s"] <= run["quiet_gap_s"] + 0.020, \
        f"a {run['bad_gap_s'] * 1e3:.0f} ms gap with bad clients, {run['quiet_gap_s'] * 1e3:.0f} ms with none"
    assert run["late_client_s"] <= 0.5, "a new client did not get the cached latest state at once"


@pytest.mark.timing
def test_clients_cannot_hurt_the_node_unloaded(node, tmp_path):
    """Timing tier (rack unpowered or the fixed cloud VM): test_clients_cannot_hurt_the_node, and with the bad clients
    the node still ticks at least 280 times in 3 s with every gap under 50 ms."""
    run = bad_clients(node, tmp_path)
    assert run["bad_ticks"] >= 0.9 * run["quiet_ticks"], run
    assert run["bad_gap_s"] <= run["quiet_gap_s"] + 0.020, run
    assert run["bad_ticks"] >= 280, f"the node ticked {run['bad_ticks']} times in {WINDOW_S} s with bad clients"
    assert run["bad_gap_s"] < 0.05, f"a {run['bad_gap_s'] * 1e3:.0f} ms gap in the node's ticks"
    assert run["late_client_s"] <= 0.5, "a new client did not get the cached latest state at once"


def test_attach_needs_an_explicit_address(node):
    lk = like_keras()
    name = node.address.rsplit("/", 1)[-1].removesuffix(".sock")
    for implicit in (name, "", "pick_cell"):
        with pytest.raises((ValueError, TypeError)):
            s = lk.System("implicit")
            s.add(lk.attach(implicit), "arm")
            dep = lk.Deployment(s)
            dep.place(s.arm, hz=10)
            dep.describe()
    assert node.address.startswith("unix:"), "describe() must print the explicit unix: address"
