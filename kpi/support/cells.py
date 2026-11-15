"""Running the reference consumer from a suite: a fake bus of its own per test, lockstep or real time, recorded."""

from __future__ import annotations

import importlib
import pathlib
import tomllib
from typing import Any

import numpy as np

from kpi.support.lk import KPI, like_keras, native_library, recorded, kpi_cpus

CONFIG = KPI / "tasks" / "arm_cell" / "arm_controller.toml"
NEEDS_NATIVE = {"target", "same_box"}
NEEDS_DORA = {"target"}


def config(directory: pathlib.Path, *, lockstep: bool, name: str = "arm", **bus: Any) -> dict[str, Any]:
    """arm_controller.toml with the board file and BUS_LOCK in `directory` (never the rack's), and no wire time
    under lockstep (the exchange is modelled, not slept, so a twin run goes faster than real time)."""
    with open(CONFIG, "rb") as file:
        settings = tomllib.load(file)
    settings["bus"]["board_file"] = str(directory / f"{name}.boards")
    settings["bus"]["lock_path"] = str(directory / f"{name}.lock")
    if lockstep:
        settings["bus"]["exchange_s"] = 0.0
    settings["bus"].update(bus)
    return settings


def cell_module():
    like_keras()
    return importlib.import_module("kpi.tasks.arm_cell.cell")


def deployment(placement: str, directory: pathlib.Path, *, lockstep: bool, **options: Any):
    """One of cell.py's three deployments on a fake bus in `directory`, pinned to this test's CPU."""
    if placement in NEEDS_NATIVE:
        native_library("arm_controller")
    directory.mkdir(parents=True, exist_ok=True)
    module = cell_module()
    return module.PLACEMENTS[placement](config(directory, lockstep=lockstep), realtime=not lockstep, cpus=kpi_cpus(),
                                        **options)


def run_lockstep(placement: str, directory: pathlib.Path, until: float, **options: Any) -> dict[str, list]:
    """Run a placement for `until` logical seconds, recording every crossing edge and exposed port; the decoded
    streams by port name."""
    dep = deployment(placement, directory, lockstep=True, **options)
    path = directory / f"{placement}.mcap"
    dep.record(path)
    try:
        dep.start(until=until)
    finally:
        dep.stop()
    streams = {port: recorded(path, port) for port in ("command", "state", "trajectory", "target")}
    streams["_path"] = path
    return streams


def command_rows(commands: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    """(stamps, positions) of a decoded ServoCommand stream."""
    stamps = np.array([command["header"]["stamp"] for command in commands])
    positions = np.array([np.asarray(command["position"]) for command in commands])
    return stamps, positions
