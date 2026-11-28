"""What scripts send: JointTrajectory plans relative to where the pendulum is now. Plain numpy, no framework."""

from __future__ import annotations

import numpy as np
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory


def min_jerk(origin: float, goal: float, seconds: float, samples: int) -> tuple[np.ndarray, np.ndarray]:
    u = np.linspace(0.0, 1.0, samples)
    s = u**3 * (10.0 - 15.0 * u + 6.0 * u**2)
    ds = 30.0 * u**2 * (1.0 - u) ** 2 / seconds
    return origin + s * (goal - origin), ds * (goal - origin)


def swing(name: str, start: float, offsets_rad: list[float], leg_s: float, stamp: float) -> JointTrajectory:
    """Min-jerk legs through start + each offset, leg_s seconds each, at rest between legs."""
    times, positions, velocities, at, now = [], [], [], start, 0.0
    for offset in offsets_rad:
        q, qd = min_jerk(at, start + offset, leg_s, 100)
        t = now + np.linspace(0.0, leg_s, 100)
        if times:
            t, q, qd = t[1:], q[1:], qd[1:]
        times.append(t), positions.append(q), velocities.append(qd)
        at, now = start + offset, now + leg_s
    return _plan(name, np.concatenate(times), np.concatenate(positions), np.concatenate(velocities), stamp)


def sine(name: str, start: float, amplitude_rad: float, hz: float, seconds: float, stamp: float) -> JointTrajectory:
    """A sine about start whose amplitude ramps in and out over one period (smooth: it starts and ends at rest)."""
    t = np.linspace(0.0, seconds, int(seconds * 100) + 1)
    ramp_s = 1.0 / hz
    u = np.clip(np.minimum(t, seconds - t) / ramp_s, 0.0, 1.0)
    envelope = u**3 * (10.0 - 15.0 * u + 6.0 * u**2)
    q = start + amplitude_rad * envelope * np.sin(2 * np.pi * hz * t)
    return _plan(name, t, q, np.gradient(q, t), stamp)


def _plan(name, t, q, qd, stamp) -> JointTrajectory:
    return JointTrajectory(t=t, joint_names=(name,), positions=q[:, None], velocities=qd[:, None],
                           header=Header(stamp=stamp))
