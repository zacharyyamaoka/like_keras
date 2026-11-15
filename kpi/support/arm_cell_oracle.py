"""The reference consumer stepped by hand, with no like_keras: what a correct lockstep run must produce (A3, twin).

The schedule is CONTRACT §5 as the ruler reads it (docs/friction/contract-002.md):
- one logical clock from 0.0; an hz group's tick i runs at i / hz; ticks at equal times run in place() order
  (arm, targets, planner);
- a rate-less group (the planner) steps once per queued message, at the time that message was sent, after every
  tick at that time; its now() is that time;
- a value sent at t is seen at the receiver's first step strictly after t; a latest input older than `stale`
  (now - sent > stale) reads None; a lagged input reads its writer's previous step in the same group.
The laws are the same Python ones the fallback runs (plan.py, jtc_law.py, guard_law.py, fake_bus.py), so a
mismatch is the framework's schedule or delivery, which is what A3 measures.
"""

from __future__ import annotations

import json
import pathlib
import tempfile
import types
from dataclasses import dataclass

import numpy as np
from robot_msgs.control import ServoCommand
from robot_msgs.std import Header

from kpi.tasks.arm_cell.fake_bus import FakeBusCore
from kpi.tasks.arm_cell.guard_law import GuardCore
from kpi.tasks.arm_cell.jtc_law import JtcCore
from kpi.tasks.arm_cell.plan import JOINTS, TARGETS, PickTarget, plan_pick, starting_at

ARM_HZ, TARGET_HZ, LEAD_S, STALE_S = 100, 2, 0.05, 0.1


@dataclass
class Run:
    commands: list[ServoCommand]  # the exposed `command` stream, one per arm tick
    plans: list  # every trajectory the planner sent, in order
    skipped: int  # targets the planner skipped for want of a fresh state


def run(until: float, targets: str | pathlib.Path = TARGETS, repeat: bool = False, bus: dict | None = None,
        guard: dict | None = None) -> Run:
    positions = [json.loads(line)["position"] for line in open(targets, encoding="utf-8") if line.strip()]
    settings = {"adapter": "0607623A", "twin": True, "watchdog_s": 0.25, "exchange_s": 0.0, "joints": len(JOINTS),
                "board_file": "", "lock_path": ""}
    settings.update(bus or {})
    with tempfile.TemporaryDirectory() as scratch:
        settings["lock_path"] = str(pathlib.Path(scratch) / "oracle.lock")
        settings["exchange_s"] = 0.0
        core_bus = FakeBusCore()
        core_bus.open_exclusive(types.SimpleNamespace(**settings))
        core_bus.take_over()
        guard_core = GuardCore(**{"runaway_rad": 0.5, "max_velocity_rad_s": 10.0, "max_effort_nm": 2.0, **(guard or {})})
        jtc = JtcCore()
        seen_trajectory = None
        last_safe = None  # guard.safe of the previous arm step: the lagged bus.command
        state_log: list[tuple[float, object]] = []  # (sent at, ServoState)
        plan_log: list[tuple[float, object]] = []  # (sent at, JointTrajectory)
        out = Run([], [], 0)
        next_target = 0
        events = []  # (time, place index, kind, tick index)
        for tick in range(int(np.floor(until * ARM_HZ)) + 1):
            if tick / ARM_HZ < until:
                events.append((tick / ARM_HZ, 0, "arm", tick))
        for tick in range(int(np.floor(until * TARGET_HZ)) + 1):
            if tick / TARGET_HZ < until:
                events.append((tick / TARGET_HZ, 1, "targets", tick))
        events.sort(key=lambda event: (event[0], event[1]))
        queued_targets = []
        for now, _, kind, _ in events:
            if kind == "arm":
                state = core_bus.exchange(now, last_safe)
                state_log.append((now, state))
                visible = [plan for sent, plan in plan_log if sent < now]
                trajectory = visible[-1] if visible else None
                if trajectory is not None and trajectory is not seen_trajectory:
                    seen_trajectory = trajectory
                    jtc.replace(trajectory)
                if trajectory is None:
                    jtc.hold_here(state)
                position, velocity, effort = jtc.sample(now, state)
                proposed = ServoCommand(name=state.name, position=position, velocity=velocity,
                                        feedforward_effort=effort, state_stamp=state.header.stamp, state_seq=state.seq,
                                        header=Header(stamp=now))
                position, velocity, effort, cap, _ = guard_core.check(
                    state.position, state.velocity, state.fault, proposed.position, proposed.velocity,
                    proposed.feedforward_effort)
                last_safe = proposed.replace(position=position, velocity=velocity, feedforward_effort=effort, max_effort=cap)
                out.commands.append(last_safe)
            else:
                if next_target == len(positions):
                    if not repeat:
                        continue
                    next_target = 0
                queued_targets.append((now, PickTarget(name=JOINTS, position=positions[next_target], header=Header(stamp=now))))
                next_target += 1
                # the rate-less planner: one step per queued target, at its send time, after every tick at that time
                sent, target = queued_targets.pop(0)
                fresh = [(at, state) for at, state in state_log if at < sent]
                state = fresh[-1][1] if fresh and sent - fresh[-1][0] <= STALE_S else None
                if state is None:
                    out.skipped += 1
                    continue
                plan = starting_at(plan_pick(target, start=state.joint_state()), sent + LEAD_S)
                plan_log.append((sent, plan))
                out.plans.append(plan)
        core_bus.stop_feeding()
    return out
