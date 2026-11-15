"""The always-on form (CONTRACT §10, consumer R2): the controller alone, exposing trajectory, state and command.

Planners reach it with `lk.attach(<address>)` (attached_planner.py). On the rack it runs under a systemd user unit;
the kpi suites start it as a subprocess:

    python -m kpi.tasks.arm_cell.always_on --name arm_kpi --language cpp --config <toml> --cpus 23
"""

from __future__ import annotations

import argparse
import json
import pathlib
import signal

import like_keras as lk

from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy
from kpi.tasks.arm_cell.cell import CONFIG, LIBRARY


def always_on(arm: lk.Component, name: str = "arm_controller", *, cpus=(20,), backend=None) -> lk.Deployment:
    s = lk.System(name)
    s.add(arm, "arm")
    s.expose(arm.trajectory, arm.state, arm.command)
    dep = lk.Deployment(s) if backend is None else lk.Deployment(s, backend=backend)
    dep.place(s.arm, process="arm-controller", hz=100, cpus=list(cpus), fifo=80)
    return dep


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--name", default="arm_controller")
    parser.add_argument("--language", choices=("cpp", "py"), default="cpp")
    parser.add_argument("--config", default=str(CONFIG), help="a TOML path, or a JSON object")
    parser.add_argument("--cpus", default="20")
    parser.add_argument("--backend", choices=("inproc", "dora"), default="inproc")
    arguments = parser.parse_args(argv)
    config = json.loads(arguments.config) if arguments.config.lstrip().startswith("{") else pathlib.Path(arguments.config)
    arm = lk.native(LIBRARY, config=config) if arguments.language == "cpp" else ArmControllerPy(config=config)
    backend = None
    if arguments.backend == "dora":
        from like_keras.backends import dora as backend
    dep = always_on(arm, arguments.name, cpus=[int(cpu) for cpu in arguments.cpus.split(",")], backend=backend)
    plan = dep.describe()
    dep.start()
    print(json.dumps({"ready": True, "attach": plan.get("attach")}), flush=True)
    try:
        signal.sigwait({signal.SIGTERM, signal.SIGINT})
    finally:
        dep.stop()
    return 0


if __name__ == "__main__":
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
    raise SystemExit(main())
