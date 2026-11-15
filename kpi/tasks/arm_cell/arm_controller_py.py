"""`ArmControllerPy`: the all-Python peer of libarm_controller.so (CONTRACT §10), the same exposed ports.

bus -> jtc -> guard over the Python fake bus, the same laws as the C++ chain (jtc_law.py, guard_law.py,
fake_bus.py), so the fallback deployment is one line away from the native one and T1 holds them equal.
"""

from __future__ import annotations

import dataclasses
import os
import time
import tomllib
from collections.abc import Mapping
from typing import Any

import like_keras as lk
import numpy as np
from robot_msgs.control import ServoCommand, ServoState
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory

from kpi.tasks.arm_cell.fake_bus import FakeBusCore
from kpi.tasks.arm_cell.guard_law import GuardCore
from kpi.tasks.arm_cell.jtc_law import JtcCore


class MoteusBusPy(lk.Component):
    command = lk.In(ServoCommand, lagged=True)  # goes out in the next exchange
    state = lk.Out(ServoState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        adapter: str = "0607623A"
        twin: bool = False
        watchdog_s: float = 0.25
        exchange_s: float = 0.0035
        joints: int = 7
        board_file: str = ""
        lock_path: str = ""

    def activate(self):
        self._bus = FakeBusCore()
        self._bus.open_exclusive(self.params)  # R1: BUS_LOCK, refused before the port opens
        self._bus.take_over()  # R2: a hold in the same tick, never a bare stop
        self._exchange_s = np.zeros(128)
        self._steps = 0

    def forward(self, command):
        started = time.perf_counter()
        state = self._bus.exchange(self.now(), command)
        self._exchange_s[self._steps % 128] = time.perf_counter() - started
        self._steps += 1
        if self._steps % 100 == 0:  # R4, R14: the exchange p95, the bus errors and the watchdog lapses
            self.metric("exchange_p95_s", float(np.percentile(self._exchange_s[: min(self._steps, 128)], 95)))
            self.metric("bus_errors", 0.0)
            self.metric("watchdog_lapses", float(self._bus.lapses))
        return state

    def deactivate(self):
        self._bus.stop_feeding()  # boards fall into their timeout mode


class JtcPy(lk.Component):
    trajectory = lk.In(JointTrajectory)  # latest
    state = lk.In(ServoState)
    command = lk.Out(ServoCommand)

    def activate(self):
        self._core = JtcCore()
        self._seen = None

    def forward(self, trajectory, state):
        if state is None:
            return None
        if trajectory is not None and not _same_plan(trajectory, self._seen):  # C++ take(): new since last step
            self._seen = trajectory
            self._core.replace(trajectory)  # starts at the first tick >= trajectory.header.stamp
        if trajectory is None:
            self._core.hold_here(state)  # none yet, or retracted (lease lapsed): hold where it is (R5)
        position, velocity, effort = self._core.sample(self.now(), state)
        return ServoCommand(name=state.name, position=position, velocity=velocity, feedforward_effort=effort,
                            state_stamp=state.header.stamp, state_seq=state.seq, header=Header(stamp=self.now()))


def _same_plan(trajectory, seen) -> bool:
    """WHY by value as well as identity: a backend may hand forward a fresh object for the same latest value."""
    if seen is None:
        return False
    if trajectory is seen:
        return True
    return (trajectory.header.stamp == seen.header.stamp and trajectory.t.shape == seen.t.shape
            and np.array_equal(trajectory.t, seen.t) and np.array_equal(trajectory.positions, seen.positions, equal_nan=True))


class GuardPy(lk.Component):
    state = lk.In(ServoState)
    proposed = lk.In(ServoCommand)
    safe = lk.Out(ServoCommand)

    @dataclasses.dataclass(frozen=True)
    class Params:
        runaway_rad: float = 0.5
        max_velocity_rad_s: float = 10.0
        max_effort_nm: float = 2.0

    def activate(self):
        self._core = GuardCore(self.params.runaway_rad, self.params.max_velocity_rad_s, self.params.max_effort_nm)

    def forward(self, state, proposed):
        if state is None or proposed is None:
            return None  # nothing to guard yet: nothing sent this step
        position, velocity, effort, cap, events = self._core.check(
            state.position, state.velocity, state.fault, proposed.position, proposed.velocity, proposed.feedforward_effort)
        for event in events:
            if event == "overspeed":
                self.log.warning("overspeed above %g rad/s (logged only)", self.params.max_velocity_rad_s)
            else:
                self.log.error("%s: holding at the measured position", "board fault" if event == "fault" else "runaway")
        return proposed.replace(position=position, velocity=velocity, feedforward_effort=effort, max_effort=cap)


def load_config(config: str | os.PathLike | Mapping[str, Any]) -> dict[str, Any]:
    """The controller's config: a TOML path or a mapping with [bus] and [guard], as lk.native takes it."""
    if isinstance(config, Mapping):
        return {key: dict(value) for key, value in config.items()}
    with open(config, "rb") as file:
        return tomllib.load(file)


def ArmControllerPy(config: str | os.PathLike | Mapping[str, Any]) -> lk.System:  # noqa: N802
    """The Python arm_controller System. WHY a CamelCase factory: it stands in for lk.native(...) in the contract's
    one-line swap (`arm = ArmControllerPy(config=CONFIG)`), as the C++ side's `arm_controller(config)` factory."""
    settings = load_config(config)
    s = lk.System("arm_controller")
    bus = s.add(MoteusBusPy(**settings["bus"]), "bus")
    jtc = s.add(JtcPy(), "jtc", state=bus.state)
    guard = s.add(GuardPy(**settings["guard"]), "guard", state=bus.state, proposed=jtc.command)
    s.wire(bus, command=guard.safe)  # a legal cycle: bus.command is lagged
    s.expose(jtc.trajectory, bus.state, command=guard.safe)
    return s  # derived order: bus -> jtc -> guard
