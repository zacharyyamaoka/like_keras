"""One script = one attached cell: it reads the core's `state`, sends ONE trajectory under the write lease, waits
for it to finish, and leaves. The core never notices it came or went, apart from the lease.

    .venv/bin/python -m examples.pendulum.script swing --deg 30 -30 0 --leg-s 2
    .venv/bin/python -m examples.pendulum.script sine --deg 20 --hz 0.3 --seconds 10
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import threading
import time

from robot_msgs.control import ServoState
from robot_msgs.trajectory import JointTrajectory

import like_keras as lk
from examples.pendulum import plans
from examples.pendulum.core import NAME

ADDRESS = f"unix:/run/user/1000/like_keras/{NAME}.sock"
DONE = threading.Event()


class OneShot(lk.Component):
    """Plans once from the first fresh state, sends it, then sets DONE when the plan has run out."""

    state = lk.In(ServoState, stale=0.1)
    trajectory = lk.Out(JointTrajectory)

    @dataclasses.dataclass(frozen=True)
    class Params:
        kind: str = "swing"
        deg: list[float] = dataclasses.field(default_factory=lambda: [30.0, 0.0])
        leg_s: float = 1.5
        hz: float = 0.5
        seconds: float = 8.0
        lead_s: float = 0.1

    def activate(self):
        self._ends = None

    def forward(self, state):
        if self._ends is not None:
            if self.now() > self._ends + 0.3:
                DONE.set()
            return None
        if state is None:
            return None
        start, stamp, p = float(state.position[0]), self.now() + self.params.lead_s, self.params
        if p.kind == "swing":
            plan = plans.swing(state.name[0], start, [math.radians(d) for d in p.deg], p.leg_s, stamp)
        else:
            plan = plans.sine(state.name[0], start, math.radians(p.deg[0]), p.hz, p.seconds, stamp)
        self._ends = stamp + float(plan.t[-1])
        self.log.info("%s from %.1f deg: %d samples over %.1f s", p.kind, math.degrees(start), len(plan.t), plan.t[-1])
        print(json.dumps({"sent": p.kind, "from_deg": round(math.degrees(start), 2), "seconds": float(plan.t[-1]),
                          "peak_deg_s": round(math.degrees(float(abs(plan.velocities).max())), 1)}), flush=True)
        return plan


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("kind", choices=("swing", "sine"))
    parser.add_argument("--deg", type=float, nargs="+", default=None)
    parser.add_argument("--leg-s", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=0.3)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--address", default=ADDRESS)
    arguments = parser.parse_args(argv)
    deg = arguments.deg or ([30.0, -30.0, 0.0] if arguments.kind == "swing" else [20.0])
    s = lk.System(f"script_{arguments.kind}")
    arm = s.add(lk.attach(arguments.address), "pendulum")
    shot = s.add(OneShot(kind=arguments.kind, deg=deg, leg_s=arguments.leg_s, hz=arguments.hz,
                         seconds=arguments.seconds), "shot", state=arm.state)
    s.wire(arm, trajectory=shot.trajectory)
    dep = lk.Deployment(s)
    dep.place(s.pendulum, hz=100)  # the proxy: sends what is new, publishes what arrived
    dep.place(s.shot, hz=50)
    began = time.monotonic()
    dep.start()
    try:
        if not DONE.wait(timeout=120):
            print(json.dumps({"error": "plan never finished"}), flush=True)
            return 1
    finally:
        dep.stop()  # leaving: the lease lapses, the core retracts `trajectory` and holds where it is
    print(json.dumps({"done": arguments.kind, "wall_s": round(time.monotonic() - began, 2)}), flush=True)
    return 0


if __name__ == "__main__":
    from examples.pendulum import script  # noqa: PLW0406  WHY: components must name a module, never __main__ (cell.py)

    raise SystemExit(script.main())
