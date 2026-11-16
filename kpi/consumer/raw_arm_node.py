"""The raw-dora reference of A35's A/B (CONTRACT §15 A35; consumer-018), written straight on dora 0.5.0's Python API
the way a user without like_keras would (as kpi/bench/raw_dora_node.py is criterion 2's): the reference arm's chain on
the same fake bus, a planner and a subscriber, each a raw dora node.

    python -m kpi.consumer.raw_arm_node arm | planner | subscriber      (dora runs them; kpi/consumer/boundary_ab.py)

- arm: on every tick of dora's 10 ms timer, the chain of kpi/tasks/arm_cell in its derived order (bus -> jtc -> guard;
  the bus sends the previous tick's guarded command, as bus.command is lagged), with the same laws (fake_bus.py,
  jtc_law.py, guard_law.py) on the same kind of board file, so one oracle reads both sides of the A/B; it sends the
  joint state (seq, stamp, positions) every tick, and takes the newest plan that arrived before the tick.
- planner: on every tick of dora's 500 ms timer, the next target of targets.jsonl (repeated), planned with plan_pick
  from the newest joint state (skipped when that is older than 0.1 s, as PickPlanner's stale=0.1), stamped now (lead
  0), noted as boundary_ab.plan_row() and sent.
- subscriber: every joint state it receives, its seq and when it arrived, as boundary_parts.SeqLog writes them.
Settings come from the environment (dora's node `env`): AB_CONFIG (the arm's config as JSON), AB_TARGETS, AB_PLANS,
AB_STATES.
"""

from __future__ import annotations

import json
import os
import sys
import time
import types

import numpy as np
import pyarrow as pa
from dora import Node

from kpi.consumer.boundary_ab import plan_row
from kpi.tasks.arm_cell.fake_bus import FakeBusCore
from kpi.tasks.arm_cell.guard_law import GuardCore
from kpi.tasks.arm_cell.jtc_law import JtcCore
from kpi.tasks.arm_cell.plan import JOINTS, plan_pick
from robot_msgs.sensor import JointState
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory

STALE_S = 0.1


def _inputs(node: Node):
    for event in node:
        if event["type"] == "STOP":
            return
        if event["type"] == "INPUT":
            yield event["id"], event["value"]


def arm() -> None:
    settings = json.loads(os.environ["AB_CONFIG"])
    bus = FakeBusCore()
    bus.open_exclusive(types.SimpleNamespace(**settings["bus"]))
    bus.take_over()
    guard_settings = settings["guard"]
    guard = GuardCore(guard_settings["runaway_rad"], guard_settings["max_velocity_rad_s"],
                      guard_settings["max_effort_nm"], guard_settings["start_tolerance_rad"])
    jtc, safe, pending = JtcCore(), None, None
    node = Node()
    try:
        for name, value in _inputs(node):
            if name == "trajectory":
                payload = value.to_numpy()
                stamp, samples, joints = float(payload[0]), int(payload[1]), int(payload[2])
                body = payload[3:]
                pending = JointTrajectory(  # the newest plan that arrived before the tick wins (latest)
                    t=body[:samples].copy(), joint_names=JOINTS[:joints],
                    positions=body[samples:samples + samples * joints].reshape(samples, joints).copy(),
                    velocities=body[samples + samples * joints:].reshape(samples, joints).copy(),
                    header=Header(stamp=stamp))
            elif name == "tick":
                now = time.time()
                state = bus.exchange(now, safe)
                if pending is not None:
                    jtc.replace(pending)
                    pending = None
                position, velocity, effort = jtc.sample(now, state)
                position, velocity, _, _, _ = guard.check(state.position, state.velocity, state.fault, position,
                                                          velocity, effort)
                safe = types.SimpleNamespace(position=position, velocity=velocity)
                node.send_output("state", pa.array(np.concatenate([[float(state.seq), state.header.stamp],
                                                                   state.position])))
    finally:
        bus.stop_feeding()


def planner() -> None:
    with open(os.environ["AB_TARGETS"], encoding="utf-8") as file:
        targets = [json.loads(line)["position"] for line in file if line.strip()]
    plans = open(os.environ["AB_PLANS"], "a", encoding="utf-8")
    latest, latest_wall, index = None, 0.0, 0
    node = Node()
    for name, value in _inputs(node):
        if name == "state":
            latest, latest_wall = value.to_numpy().copy(), time.time()
        elif name == "tick":
            if latest is None or time.time() - latest_wall > STALE_S:
                continue
            target = JointState(name=JOINTS, position=targets[index % len(targets)])
            index += 1
            plan = plan_pick(target, start=JointState(name=JOINTS, position=latest[2:2 + len(JOINTS)]))
            stamp = time.time()  # lead 0: due now
            plan = plan.replace(header=Header(stamp=stamp))
            payload = np.concatenate([[stamp, float(len(plan.t)), float(len(JOINTS))], plan.t,
                                      plan.positions.ravel(), plan.velocities.ravel()])
            row = plan_row(plan, time.time())
            node.send_output("trajectory", pa.array(payload))
            plans.write(json.dumps(row) + "\n")
            plans.flush()
    plans.close()


def subscriber() -> None:
    states = open(os.environ["AB_STATES"], "a", encoding="utf-8")
    node = Node()
    for name, value in _inputs(node):
        if name == "state":
            row = value.to_numpy()
            states.write(json.dumps({"seq": int(row[0]), "stamp": float(row[1]), "wall": time.time()}) + "\n")
            states.flush()
    states.close()


if __name__ == "__main__":
    {"arm": arm, "planner": planner, "subscriber": subscriber}[sys.argv[1]]()
