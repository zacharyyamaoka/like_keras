"""The guard rule in Python: kpi/tasks/arm_cell/guard.hpp, same decisions in the same order (consumer R6).

A board fault (32-50) latches a hold at the measured position for the rest of the activation. A runaway
(|proposed - measured| > runaway_rad) holds the same way until the next plan that starts within start_tolerance_rad of
the measured position on every joint, which it accepts, reporting "released" (Zach, 2026-11-13: "robots recover,
they don't stop"; CONTRACT §15 A14; the reasons are in guard.hpp). Overspeed is reported for logging only;
commands stay PD + feed-forward with the datasheet effort cap.
"""

from __future__ import annotations

import numpy as np

START_TOLERANCE_RAD = 0.0524  # 3°, arm-hold's READY tolerance (guard.hpp's Params)


class GuardCore:
    def __init__(self, runaway_rad: float, max_velocity_rad_s: float, max_effort_nm: float,
                 start_tolerance_rad: float = START_TOLERANCE_RAD) -> None:
        self.runaway_rad = runaway_rad
        self.max_velocity_rad_s = max_velocity_rad_s
        self.max_effort_nm = max_effort_nm
        self.start_tolerance_rad = start_tolerance_rad
        self.holding: str | None = None  # None, "fault" (latched) or "runaway" (until a plan starts here)
        self._hold: np.ndarray | None = None
        self._overspeed_logged = False

    def _releases(self, measured_position, position) -> bool:
        """A proposal within start_tolerance_rad of the measured position on every joint. WHY `<=`: a NaN target
        compares false, so it is no start, exactly as guard.hpp's `!(... <= ...)`."""
        return bool(np.all(np.abs(np.asarray(position) - measured_position) <= self.start_tolerance_rad))

    def check(self, measured_position, measured_velocity, measured_fault, position, velocity, effort):
        """(position, velocity, effort, max_effort, events): the safe command and what to log
        ("released", "fault", "runaway", "overspeed", in that order)."""
        events = []
        if self.holding == "runaway" and self._releases(measured_position, position):
            self.holding = None
            events.append("released")
        if self.holding is None:
            fault = measured_fault is not None and bool(np.any((measured_fault >= 32) & (measured_fault <= 50)))
            targets = np.asarray(position)
            off = np.abs(targets - measured_position) > self.runaway_rad
            runaway = bool(np.any(off & ~np.isnan(targets)))
            if fault or runaway:
                self.holding = "fault" if fault else "runaway"
                self._hold = np.array(measured_position, dtype=np.float64)
                events.append(self.holding)
        overspeed = measured_velocity is not None and bool(np.any(np.abs(measured_velocity) > self.max_velocity_rad_s))
        if overspeed and not self._overspeed_logged:
            events.append("overspeed")
        self._overspeed_logged = overspeed
        joints = len(measured_position)
        if self.holding is not None:
            position, velocity, effort = self._hold.copy(), np.zeros(joints), np.zeros(joints)
        return position, velocity, effort, np.full(joints, self.max_effort_nm), events
