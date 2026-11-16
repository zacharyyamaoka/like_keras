"""File 3 of the reference consumer (CONTRACT §4.6, §10): the wiring, and its three deployments.

Each deployment differs from the next by one line (criterion 5): `target` (the controller in its own process, dora
between), `same_box` (the C++ chain on its own thread inside this Python process) and `fallback` (all Python).
The builders take `config`, `realtime`, `cpus` and `targets` so the kpi suites can run them on a per-test fake bus,
in lockstep, and off the rack's cores; `python -m kpi.tasks.arm_cell.cell` runs `target` as the contract writes it.
"""

from __future__ import annotations

import pathlib
import signal

import like_keras as lk

from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy
from kpi.tasks.arm_cell.planner import PickPlanner
from kpi.tasks.arm_cell.targets import TARGETS, TargetFile

HERE = pathlib.Path(__file__).resolve().parent
CONFIG = HERE / "arm_controller.toml"
LIBRARY = HERE.parents[2] / "build" / "kpi" / "libarm_controller.so"  # kpi/CMakeLists.txt builds it


def pick_cell(arm: lk.Component, targets: str | pathlib.Path = TARGETS, repeat: bool = False) -> lk.System:
    s = lk.System("pick_cell")
    source = s.add(TargetFile(path=str(targets), repeat=repeat), "targets")
    s.add(arm, "arm")
    planner = s.add(PickPlanner(), "planner", target=source.target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    s.expose(arm.state, arm.command)
    return s


def target(config=CONFIG, *, realtime=True, cpus=(20,), targets=TARGETS, repeat=False) -> lk.Deployment:
    # WHY here, not at module level: importing this file (for CONFIG, or for the inproc builders) must not import
    # the dora backend, which kpi/blackbox blocks (CONTRACT §15 A1; ruler-002 §2).
    from like_keras.backends import dora

    cell = pick_cell(lk.native(LIBRARY, config=config), targets, repeat)
    dep = lk.Deployment(cell, backend=dora, realtime=realtime)
    dep.place(cell.arm, process="arm-controller", hz=100, cpus=list(cpus), fifo=80)
    dep.place(cell.targets, hz=2)
    dep.place(cell.planner)  # rate-less: steps once per target
    return dep


def same_box(config=CONFIG, *, realtime=True, cpus=(20,), targets=TARGETS, repeat=False) -> lk.Deployment:
    cell = pick_cell(lk.native(LIBRARY, config=config), targets, repeat)
    dep = lk.Deployment(cell, realtime=realtime)
    dep.place(cell.arm, process="arm-controller", hz=100, cpus=list(cpus), fifo=80)
    dep.place(cell.targets, hz=2)
    dep.place(cell.planner)
    return dep


def fallback(config=CONFIG, *, realtime=True, cpus=(20,), targets=TARGETS, repeat=False) -> lk.Deployment:
    cell = pick_cell(ArmControllerPy(config=config), targets, repeat)
    dep = lk.Deployment(cell, realtime=realtime)
    dep.place(cell.arm, process="arm-controller", hz=100, cpus=list(cpus), fifo=80)
    dep.place(cell.targets, hz=2)
    dep.place(cell.planner)
    return dep


PLACEMENTS = {"target": target, "same_box": same_box, "fallback": fallback}


if __name__ == "__main__":
    dep = target()
    dep.start()
    try:
        signal.pause()
    finally:
        dep.stop()
