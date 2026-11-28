"""The always-on core (CONTRACT §10's always-on form, R2): bus -> JTC -> guard at 100 Hz, exposing trajectory, state
and command on an attach socket. It never goes down between scripts: a script attaches, writes `trajectory` under the
write lease, and leaves; when its lease lapses the input is retracted and the JTC holds where it is (R5).

    .venv/bin/python -m examples.pendulum.core --bind twin
    .venv/bin/python -m examples.pendulum.core --bind real --can-id 11

The JTC and the guard are the reference consumer's (kpi/tasks/arm_cell), used read-only; only the bus is new.
"""

from __future__ import annotations

import argparse
import json
import signal

import like_keras as lk
from examples.pendulum.bus import STATIONS, PendulumBus
from kpi.tasks.arm_cell.arm_controller_py import GuardPy, JtcPy

NAME = "pendulum"


def controller(bind: str = "twin", can_id: int = 11) -> lk.System:
    """The pendulum's controller box: the bus (twin or real), the JTC and the guard, one 100 Hz loop."""
    with lk.System("controller") as s:
        bus = PendulumBus(bind=bind, can_id=can_id)
        cap = STATIONS[can_id]["cap_nm"]
        jtc, guard = JtcPy(), GuardPy(runaway_rad=0.5, max_velocity_rad_s=1.0, max_effort_nm=cap)  # 1.0: tape overspeed
        jtc(state=bus.state)
        guard(state=bus.state, proposed=jtc.command)
        bus(command=guard.safe)  # the loop: legal because bus.command is lagged
        s.expose(jtc.trajectory, bus.state, command=guard.safe)
    return s


def always_on(bind: str, can_id: int, backend=None, host: str | None = None) -> lk.Deployment:
    """The core alone, as kpi/tasks/arm_cell/always_on.py places the arm: what it exposes is the attach socket."""
    s = lk.System(NAME)
    arm = s.add(controller(bind, can_id), "pendulum")
    s.expose(arm.trajectory, arm.state, arm.command)
    dep = lk.Deployment(s) if backend is None else lk.Deployment(s, backend=backend)
    dep.place(s.pendulum, process="pendulum-core", hz=100, host=host)
    return dep


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bind", choices=("twin", "real"), default="twin")
    parser.add_argument("--can-id", type=int, default=11, choices=sorted(STATIONS))
    parser.add_argument("--backend", choices=("inproc", "dora"), default="inproc")
    parser.add_argument("--host", default=None, help="dora machine id of the daemon to run on (dora only; none for up.sh's private daemon)")
    parser.add_argument("--record", default="", help="an .mcap path: every exposed port, for the run's evidence")
    arguments = parser.parse_args(argv)
    backend = None
    if arguments.backend == "dora":
        from like_keras.backends import dora as backend
    # WHY host= is optional: the production daemon (dora-daemon-ubuntu) registers as machine "ubuntu", but like_keras's
    # dora gateway node carries no machine (emit.py, 2026-11-28), so up.sh runs a private, unnamed daemon instead
    dep = always_on(arguments.bind, arguments.can_id, backend, host=arguments.host if backend else None)
    if arguments.record:
        dep.record(arguments.record)
    sheet = dep.describe()
    dep.start()
    print(json.dumps({"ready": True, "bind": arguments.bind, "attach": sheet.get("attach")}), flush=True)
    try:
        signal.sigwait({signal.SIGTERM, signal.SIGINT})
    finally:
        dep.stop()
    return 0


if __name__ == "__main__":
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
    raise SystemExit(main())
