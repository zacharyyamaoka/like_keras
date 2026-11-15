"""A planner attached to a running always-on controller (CONTRACT §4.5, §9.3; T5's attached form, T8).

    python -m kpi.tasks.arm_cell.attached_planner --address unix:/run/user/1000/like_keras/arm_kpi.sock \
        --targets kpi/tasks/arm_cell/targets.jsonl --record /tmp/planner.mcap

It records its proxy's health (the `LeaseHeld` refusal of T8) and the trajectories it sent.
"""

from __future__ import annotations

import argparse
import json
import signal

import like_keras as lk

from kpi.tasks.arm_cell.planner import PickPlanner
from kpi.tasks.arm_cell.targets import TARGETS, TargetFile


def attached_cell(address: str, targets: str = str(TARGETS), name: str = "planner_cell") -> lk.System:
    s = lk.System(name)
    arm = s.add(lk.attach(address), "arm")
    source = s.add(TargetFile(path=targets, repeat=True), "targets")
    planner = s.add(PickPlanner(), "planner", target=source.target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    return s


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", required=True)
    parser.add_argument("--targets", default=str(TARGETS))
    parser.add_argument("--name", default="planner_cell")
    parser.add_argument("--record", default="")
    arguments = parser.parse_args(argv)
    cell = attached_cell(arguments.address, arguments.targets, arguments.name)
    dep = lk.Deployment(cell)
    dep.place(cell.targets, hz=2)
    dep.place(cell.planner)
    dep.place(cell.arm, hz=100)  # the proxy: steps at 100 Hz, sending what is new and publishing what arrived
    if arguments.record:
        dep.record(arguments.record, cell.arm.health, cell.planner.trajectory)
    dep.start()
    print(json.dumps({"ready": True}), flush=True)
    try:
        signal.sigwait({signal.SIGTERM, signal.SIGINT})
    finally:
        dep.stop()
    return 0


if __name__ == "__main__":
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
    raise SystemExit(main())
