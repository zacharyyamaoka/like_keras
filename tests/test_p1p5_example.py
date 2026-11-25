"""The worked example (examples/pick_cell): one System, wired by calls over roles (P1 + P5), runs on the twin with
bind=SIM, on inproc and on dora with only `backend=` changed, and sends the same commands; the test double and the
C++ arm box are each one binding line.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_p1p5_example.py
"""

from __future__ import annotations

import numpy as np
import pytest

from examples.pick_cell import cell

UNTIL_S = 3.0


def commands(result: dict) -> list:
    return result["streams"]["pick_cell.command"]


@pytest.fixture(scope="module")
def sim_inproc(tmp_path_factory):
    return cell.run("sim", "inproc", UNTIL_S, tmp_path_factory.mktemp("sim_inproc"))


def test_sim_on_inproc_runs_the_plans_on_the_twin(sim_inproc):
    plans = sim_inproc["streams"]["pick_cell.planner.trajectory"]
    sent = commands(sim_inproc)
    assert len(plans) == 5 and len(sent) == 300
    last_plan, last_command = plans[-1][2], sent[-1][2]
    assert np.abs(np.asarray(last_command.position) - last_plan.positions[-1]).max() < 1e-3
    sheet = sim_inproc["sheet"]["components"]
    assert sheet["pick_cell.arm.io"]["bound_by"] == "bind[Arm]" and sheet["pick_cell.arm.io"]["impl"].endswith(":MoteusBusPy")


def test_the_same_system_on_dora_sends_the_same_commands(sim_inproc, tmp_path, private_dora):
    dora = cell.run("sim", "dora", UNTIL_S, tmp_path)
    assert [frame for _, frame, _ in commands(dora)] == [frame for _, frame, _ in commands(sim_inproc)]
    plans = [frame for _, frame, _ in dora["streams"]["pick_cell.planner.trajectory"]]
    assert plans == [frame for _, frame, _ in sim_inproc["streams"]["pick_cell.planner.trajectory"]]


def test_the_test_double_is_one_binding_line(tmp_path):
    result = cell.run("test", "inproc", 2.0, tmp_path)
    [(_, _, plan)] = result["streams"]["pick_cell.planner.trajectory"]
    assert np.allclose(plan.positions[-1], cell.OneTarget.Params().position)


@pytest.mark.skipif(not cell.LIBRARY.exists(), reason="build/kpi/libarm_controller.so not built")
def test_the_cpp_arm_box_is_one_binding_line(sim_inproc, tmp_path):
    cpp = cell.run("cpp", "inproc", UNTIL_S, tmp_path)
    assert cpp["sheet"]["components"]["pick_cell.arm"]["language"] == "native"
    python = np.array([command.position for _, _, command in commands(sim_inproc)])
    native = np.array([command.position for _, _, command in commands(cpp)])
    assert python.shape == native.shape and np.abs(python - native).max() <= 1e-9
