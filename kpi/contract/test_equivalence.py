"""The equivalence gate (CONTRACT §11): identical output streams on recorded inputs, in lockstep, in one process,
across processes and with the C++ replacement; sent = received. Any red here zeroes the total.

The gate's checks, each its own test id (kpi/score.py lists them; M1 needs the python/inproc ones):
  test_one_process_matches_schedule      fallback, inproc: bit-identical to the hand-stepped schedule
  test_across_processes                  the Python cell with the arm in its own process (dora) = in one process
  test_cpp_replacement[same_box|target]  the C++ chain = the Python chain to 1e-9 rad; target = same_box bit for bit
  test_sent_equals_received[...]         a queue edge delivers every value once, in order
  test_reference_solutions[...]          the criterion-4 task solutions = their plain-Python outputs
  test_swap_variant[...]                 every frozen swap variant (criterion 5) runs (or describes) equivalently
"""

from __future__ import annotations

import importlib
import json

import numpy as np
import pytest

from kpi.support import arm_cell_oracle, backends, cells, mcap
from kpi.support.lk import BUILD, KPI, backend, like_keras, recorded, kpi_cpus

UNTIL_S = 4.0


def frames(path, port: str) -> list[bytes]:
    return [message.data for message in mcap.read(path).on(port)]


def run_variant(module: str, directory, until: float = UNTIL_S):
    like_keras()
    deploy = importlib.import_module(module)
    dep = deploy.deployment(cells.config(directory, lockstep=True), realtime=False, cpus=kpi_cpus())
    path = directory / "run.mcap"
    dep.record(path)
    dep.start(until=until)
    dep.stop()
    return path


def test_one_process_matches_schedule(tmp_path):
    streams = cells.run_lockstep("fallback", tmp_path, UNTIL_S)
    expected = arm_cell_oracle.run(UNTIL_S)
    stamps, positions = cells.command_rows(streams["command"])
    want = np.array([command.position for command in expected.commands])
    common = min(len(stamps), len(want))
    assert common >= len(want) - 1 and common > 300
    assert positions[:common].tobytes() == want[:common].tobytes(), "the one-process run differs from the schedule"
    assert len(streams["trajectory"]) == len(expected.plans), "sent != received: a plan was lost"


@pytest.mark.backend("dora")
def test_across_processes(tmp_path, private_dora):
    """The same Python cell, the arm in one process vs in its own process under dora: the same bytes."""
    one = run_variant("kpi.tasks.arm_cell.swaps.process.a.deploy", tmp_path / "one")
    two = run_variant("kpi.tasks.arm_cell.swaps.process.b.deploy", tmp_path / "two")
    assert frames(one, "command") and frames(one, "command") == frames(two, "command")
    assert frames(one, "trajectory") == frames(two, "trajectory")


@pytest.mark.parametrize("placement", backends.placements("same_box", "target"))
def test_cpp_replacement(placement, tmp_path, request):
    if placement == "target":
        request.getfixturevalue("private_dora")
    python = cells.run_lockstep("fallback", tmp_path / "python", UNTIL_S)
    cpp = cells.run_lockstep(placement, tmp_path / placement, UNTIL_S)
    py_stamps, py_positions = cells.command_rows(python["command"])
    cpp_stamps, cpp_positions = cells.command_rows(cpp["command"])
    assert np.array_equal(py_stamps, cpp_stamps)
    assert float(np.max(np.abs(py_positions - cpp_positions))) <= 1e-9
    assert len(python["trajectory"]) == len(cpp["trajectory"]), "sent != received"
    if placement == "target":
        same_box = cells.run_lockstep("same_box", tmp_path / "same_box_again", UNTIL_S)
        assert frames(same_box["_path"], "command") == frames(cpp["_path"], "command"), "the same .so, two placements"


@pytest.mark.parametrize("backend_name", backends.names())
def test_sent_equals_received(backend_name, tmp_path, request):
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    from kpi.conformance.parts import Counter, QueueSink

    s = lk.System(f"sent_{backend_name}")
    counter = s.add(Counter(limit=300), "counter")
    s.add(QueueSink(path=str(tmp_path / "sink.jsonl")), "sink", value=counter.value)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.counter, process="writer", hz=100)
    dep.place(s.sink, process="reader")
    dep.start(until=3.5)
    dep.stop()
    received = [json.loads(line)["value"] for line in open(tmp_path / "sink.jsonl")]
    assert received == list(range(300))


@pytest.mark.parametrize("task", backends.imported_by(["pipeline", "relay"], lambda task: (
    KPI / "tasks" / task / "solution" / f"{task}.py").read_text()))
def test_reference_solutions(task, tmp_path, request):
    """The frozen criterion-4 solutions run and produce exactly their plain-Python outputs."""
    like_keras()
    if task == "relay":
        request.getfixturevalue("private_dora")
    solution = importlib.import_module(f"kpi.tasks.{task}.solution.{task}")
    plain = importlib.import_module(f"kpi.tasks.{task}.plain.{task}")
    dep = getattr(solution, task)(path=str(tmp_path / "solution.txt"), realtime=False)
    dep.start(until=0.95)
    dep.stop()
    if task == "relay":
        plain.relay(path=str(tmp_path / "plain.txt"), steps=10)
    else:
        plain.pipeline(path=str(tmp_path / "plain.txt"), steps=10, hz=1e6)
    assert (tmp_path / "solution.txt").read_text() == (tmp_path / "plain.txt").read_text()


@pytest.mark.parametrize("variant", backends.imported_by(
    ["process-a", "process-b", "language-a", "language-b", "backend-a", "backend-b", "host-a", "host-b"],
    lambda variant: (KPI / "tasks" / "arm_cell" / "swaps" / variant.replace("-", "/") / "deploy.py").read_text()))
def test_swap_variant(variant, tmp_path, request):
    """Each frozen swap variant runs in lockstep with the reference command stream; host-b (two machines) is v0.1,
    so it must only describe (CONTRACT §13)."""
    swap, side = variant.split("-")
    module = f"kpi.tasks.arm_cell.swaps.{swap}.{side}.deploy"
    source = (KPI / "tasks" / "arm_cell" / "swaps" / swap / side / "deploy.py").read_text()
    if "lk.native" in source and not (BUILD / "libarm_controller.so").exists():
        pytest.skip("build/kpi/libarm_controller.so not built")
    if "dora" in source.split("def deployment")[1]:
        request.getfixturevalue("private_dora")
    if variant == "host-b":
        like_keras()
        deploy = importlib.import_module(module)
        plan = deploy.deployment(cells.config(tmp_path, lockstep=True), realtime=False, cpus=kpi_cpus()).describe()
        assert any(group.get("host") == "rack" for group in plan["groups"])
        return
    path = run_variant(module, tmp_path)
    reference = cells.run_lockstep("fallback", tmp_path / "reference", UNTIL_S)
    got = [message for message in recorded(path, "command")]
    stamps, positions = cells.command_rows(got)
    want_stamps, want_positions = cells.command_rows(reference["command"])
    assert np.array_equal(stamps, want_stamps) and float(np.max(np.abs(positions - want_positions))) <= 1e-9
