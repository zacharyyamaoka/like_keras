"""The planner's law, a stand-in for bam_pick.plan (CONTRACT §4.6): seconds of numpy, no framework.

`PickTarget` is a robot_msgs `JointState` (the goal in joint space). WHY a robot_msgs type and not a new class:
like_keras defines no message types, and a robot_msgs type can cross a process and be recorded (CONTRACT §7.2:
"plus whatever type the reference planner's target uses if it crosses a process").
"""

from __future__ import annotations

import pathlib

import numpy as np
from robot_msgs.sensor import JointState
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory

PickTarget = JointState

JOINTS = tuple(f"j{index + 1}" for index in range(7))
TARGETS = pathlib.Path(__file__).resolve().with_name("targets.jsonl")  # the recorded targets T1 replays
SAMPLES = 1000  # A1: a 7-joint x 1000-sample trajectory
DURATION_S = 0.4  # shorter than the 0.5 s between targets, so every plan ends and holds before the next begins


def plan_pick(target: PickTarget, start: JointState) -> JointTrajectory:
    """A minimum-jerk move from `start` to `target`, SAMPLES samples over DURATION_S, t = 0 at the plan's start."""
    goal = np.asarray(target.position, dtype=np.float64)
    origin = np.asarray(start.position, dtype=np.float64)
    t = np.linspace(0.0, DURATION_S, SAMPLES)
    u = t / DURATION_S
    s = u**3 * (10.0 - 15.0 * u + 6.0 * u**2)
    ds = 30.0 * u**2 * (1.0 - u) ** 2 / DURATION_S
    delta = goal - origin
    return JointTrajectory(
        t=t,
        joint_names=tuple(target.name),
        positions=origin[None, :] + s[:, None] * delta[None, :],
        velocities=ds[:, None] * delta[None, :],
    )


def starting_at(plan: JointTrajectory, stamp: float) -> JointTrajectory:
    """The same plan with its absolute start (epoch s, or logical s under lockstep) in header.stamp (§5.6)."""
    return plan.replace(header=Header(stamp=stamp, frame_id=plan.header.frame_id))
