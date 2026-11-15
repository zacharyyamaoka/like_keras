"""The JTC law in Python: kpi/tasks/arm_cell/jtc_core.hpp plus the vendored joint_trajectory_controller.hpp, op for op.

WHY op for op (the same IEEE-754 operations in the same order: `(1 - w) * a + w * b`, a clock that adds the measured
dt, the end held at rest): T1 holds the C++ chain and this one to 1e-9 rad on every plan tick, and the equivalence
gate compares their command streams. numpy's elementwise multiply and add round exactly as the C++ built with
-ffp-contract=off (kpi/CMakeLists.txt) does.
"""

from __future__ import annotations

import numpy as np

END_TOLERANCE_S = 1e-9  # robot_controllers' kEndToleranceS


class TrajectoryLaw:
    """robot_controllers::JointTrajectoryController without a path tolerance or an effort cap."""

    def __init__(self) -> None:
        self.loaded = False
        self._time = 0.0
        self._fresh = True

    def replace_input(self, times: np.ndarray, positions: np.ndarray, velocities: np.ndarray | None,
                      effort: np.ndarray | None) -> None:
        self._times, self._positions, self._velocities, self._effort = times, positions, velocities, effort
        self._start, self._end = times[0], times[-1]
        joints = positions.shape[1]
        self.position, self.velocity, self.effort = np.zeros(joints), np.zeros(joints), np.zeros(joints)
        self._time, self._fresh, self.loaded = 0.0, True, True

    def tick(self, dt: float) -> bool:
        if not (dt >= 0.0 and dt != np.inf) or not self.loaded:
            return False
        if self._fresh:
            self._fresh = False
        else:
            self._time += dt
        time = self._time
        self._sample(time)
        if time + END_TOLERANCE_S >= self._end:
            self.velocity = np.zeros_like(self.velocity)  # WHY at rest: the hold must not carry the last velocity
        return True

    def _row(self, row: int) -> None:
        self.position = self._positions[row].copy()
        self.velocity = np.zeros_like(self.position) if self._velocities is None else self._velocities[row].copy()
        self.effort = np.zeros_like(self.position) if self._effort is None else self._effort[row].copy()

    def _sample(self, time: float) -> None:
        samples = len(self._times)
        if time >= self._end or samples == 1:
            self._row(samples - 1)
            return
        if time < self._start:
            self._row(0)
            return
        lower = int(np.searchsorted(self._times, time, side="right")) - 1  # bisect_right - 1 = upper_bound - 1
        upper = lower + 1
        span = self._times[upper] - self._times[lower]
        weight = (time - self._times[lower]) / span
        keep = 1.0 - weight
        low, high = self._positions[lower], self._positions[upper]
        self.position = keep * low + weight * high
        if self._velocities is None:
            self.velocity = (high - low) / span  # WHY the slope: a zero velocity makes the board's damping brake
        else:
            self.velocity = keep * self._velocities[lower] + weight * self._velocities[upper]
        if self._effort is None:
            self.effort = np.zeros_like(self.position)
        else:
            self.effort = keep * self._effort[lower] + weight * self._effort[upper]


class JtcCore:
    """jtc_core.hpp: a pending plan starts at the first tick >= its stamp; no plan or a retraction holds, latched once."""

    def __init__(self) -> None:
        self._law = TrajectoryLaw()
        self._pending = None
        self._active = False
        self._holding = False
        self._hold = None
        self._last_now = 0.0

    def replace(self, trajectory) -> bool:
        samples = len(trajectory.t)
        if samples == 0 or samples > 4096 or trajectory.positions.shape[1] > 16:
            return False
        self._pending = trajectory  # a robot_msgs message is immutable: keeping the reference is the copy
        return True

    def hold_here(self, state) -> None:
        self._pending = None  # a plan taken and then retracted before its start never runs
        if self._holding:
            return
        self._hold = np.array(state.position, dtype=np.float64)
        self._holding, self._active = True, False

    def sample(self, now: float, state) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(position, velocity, feed-forward effort) for this tick."""
        dt = now - self._last_now
        if self._pending is not None and now >= self._pending.header.stamp:
            plan = self._pending
            self._law.replace_input(plan.t, plan.positions, plan.velocities, plan.effort)
            self._pending, self._active, self._holding = None, True, False
            dt = 0.0
        self._last_now = now
        if not self._active and not self._holding:  # a plan waits for its start, nothing ran yet: hold here
            self._hold = np.array(state.position, dtype=np.float64)
            self._holding = True
        if self._active and self._law.tick(dt):
            return self._law.position, self._law.velocity, self._law.effort
        joints = len(state.position)
        return self._hold[:joints].copy(), np.zeros(joints), np.zeros(joints)
