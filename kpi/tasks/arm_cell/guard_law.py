"""The guard rule in Python: kpi/tasks/arm_cell/guard.hpp, same decisions in the same order (consumer R6).

A board fault (32-50) or a runaway (|proposed - measured| > runaway_rad) latches a hold at the measured position for
the rest of the activation; overspeed is reported for logging only (Zach, 2026-11-13: "recover, don't stop");
commands stay PD + feed-forward with the datasheet effort cap.
"""

from __future__ import annotations

import numpy as np


class GuardCore:
    def __init__(self, runaway_rad: float, max_velocity_rad_s: float, max_effort_nm: float) -> None:
        self.runaway_rad = runaway_rad
        self.max_velocity_rad_s = max_velocity_rad_s
        self.max_effort_nm = max_effort_nm
        self.latched = False
        self._hold: np.ndarray | None = None
        self._overspeed_logged = False

    def check(self, measured_position, measured_velocity, measured_fault, position, velocity, effort):
        """(position, velocity, effort, max_effort, events): the safe command and what to log ("fault", "runaway", "overspeed")."""
        events = []
        if not self.latched:
            fault = measured_fault is not None and bool(np.any((measured_fault >= 32) & (measured_fault <= 50)))
            targets = np.asarray(position)
            off = np.abs(targets - measured_position) > self.runaway_rad
            runaway = bool(np.any(off & ~np.isnan(targets)))
            if fault or runaway:
                self.latched = True
                self._hold = np.array(measured_position, dtype=np.float64)
                events.append("fault" if fault else "runaway")
        overspeed = measured_velocity is not None and bool(np.any(np.abs(measured_velocity) > self.max_velocity_rad_s))
        if overspeed and not self._overspeed_logged:
            events.append("overspeed")
        self._overspeed_logged = overspeed
        joints = len(measured_position)
        if self.latched:
            position, velocity, effort = self._hold.copy(), np.zeros(joints), np.zeros(joints)
        return position, velocity, effort, np.full(joints, self.max_effort_nm), events
