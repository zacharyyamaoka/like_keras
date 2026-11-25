"""The worked example of the API proposals (P1 + P5) as running code: a planner sends a JointTrajectory to the arm
controller, whose JTC closes a 100 Hz loop with the arm, first on the twin.

    python -m examples.pick_cell.cell                          # inproc, bind=SIM (the twin), 3 s in lockstep
    python -m examples.pick_cell.cell --backend dora           # the same System on dora: one Deployment line
    python -m examples.pick_cell.cell --profile cpp            # the arm box as the C++ .so: one binding line

Run it from the repository root in a private network namespace for dora (kpi/run.py's reasons: the production dora
holds 127.0.0.1:6012); `--private-dora` starts a coordinator and daemon of its own there.

- P1, the call form: inside `with lk.System(...)`, `planner(target=..., state=arm.state)` is the wiring.
- P5, roles and bindings: `Arm` and `Targets` are roles (ports, no forward). The System wires them and never changes;
  `lk.Deployment(cell, bind=SIM)` chooses what runs: the twin bus, the C++ box, a test double.

The implementations are the reference consumer's, used read-only (kpi/tasks/arm_cell): the planner, the target file,
the JTC, the guard and the fake moteus bus (the twin). Only the two roles and the wiring are new.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import importlib
import pathlib
import tempfile

import like_keras as lk
from robot_msgs.control import ServoCommand, ServoState
from robot_msgs.std import Header

from kpi.tasks.arm_cell.arm_controller_py import GuardPy, JtcPy, MoteusBusPy, load_config
from kpi.tasks.arm_cell.plan import JOINTS, TARGETS, PickTarget
from kpi.tasks.arm_cell.planner import PickPlanner
from kpi.tasks.arm_cell.targets import TargetFile

REPO = pathlib.Path(__file__).resolve().parents[2]
CONFIG = REPO / "kpi" / "tasks" / "arm_cell" / "arm_controller.toml"
LIBRARY = REPO / "build" / "kpi" / "libarm_controller.so"  # the reference arm box in C++: bus -> jtc -> guard


# ---------------------------------------------------------------- roles (P5): the datasheet without the device


class Arm(lk.Component):
    """An arm's I/O box: a servo command in, the measured servo state out. Bound to the twin bus in sim, to the
    moteus bus on the robot."""

    command = lk.In(ServoCommand, lagged=True)  # every arm sends this tick's command at the next exchange
    state = lk.Out(ServoState)


class Targets(lk.Component):
    """Where pick targets come from: a file on the rig, a camera pipeline in SPPM, a script in a test."""

    target = lk.Out(PickTarget)


class OneTarget(Targets):
    """A test double for Targets: one pick target, sent at the first tick at or after `at_s`."""

    @dataclasses.dataclass(frozen=True)
    class Params:
        position: list[float] = dataclasses.field(default_factory=lambda: [0.2, -0.1, 0.3, 0.1, 0.0, -0.2, 0.25])
        at_s: float = 0.1  # WHY a list above: params cross a process as JSON, and a tuple comes back a list

    def activate(self):
        self._sent = False

    def forward(self):
        if self._sent or self.now() < self.params.at_s:
            return None
        self._sent = True
        return PickTarget(name=JOINTS, position=self.params.position, header=Header(stamp=self.now()))


# ---------------------------------------------------------------- systems (P1): the call is the wiring


def arm_controller() -> lk.System:
    """The independent controller: arm I/O + JTC + guard, one box, 100 Hz when placed so."""
    with lk.System("arm_controller") as s:
        io, jtc, guard = Arm(), JtcPy(), GuardPy()  # a role and two concrete components
        jtc(state=io.state)  # subscribe direction: a component names only its inputs
        guard(state=io.state, proposed=jtc.command)
        io(command=guard.safe)  # the loop, legal because Arm.command is lagged
        s.expose(jtc.trajectory, io.state, command=guard.safe)
    return s


def pick_cell() -> lk.System:
    """Targets, the arm controller and the planner. No arguments: the same cell in sim, on the robot, in tests."""
    with lk.System("pick_cell") as s:
        targets, arm, planner = Targets(), arm_controller(), PickPlanner(lead_s=0.05)
        planner(target=targets.target, state=arm.state)
        arm(trajectory=planner.trajectory)  # a System is called like any component
        s.expose(arm.state, arm.command)
    return s


# ---------------------------------------------------------------- profiles (P5): what runs, by name


def profiles(directory: pathlib.Path) -> dict[str, dict]:
    """SIM, CPP and TEST for a run whose fake-bus board file lives in `directory` (never the rack's)."""
    config = load_config(CONFIG)
    config["bus"].update(board_file=str(directory / "arm.boards"), lock_path=str(directory / "arm.lock"))
    config["bus"]["exchange_s"] = 0.0  # lockstep: the exchange is modelled, not slept (kpi/support/cells.py)
    sim = {Arm: MoteusBusPy(**config["bus"]), Targets: TargetFile(path=str(TARGETS))}
    found = {"sim": sim, "test": {**sim, Targets: OneTarget()}}
    if LIBRARY.exists():
        found["cpp"] = {**sim, "arm": lk.native(LIBRARY, config=config)}  # the whole arm box -> the C++ .so
    return found


# ---------------------------------------------------------------- launch


def deployment(profile: dict, backend_name: str = "inproc") -> lk.Deployment:
    backend = importlib.import_module(f"like_keras.backends.{backend_name}")
    cell = pick_cell()
    dep = lk.Deployment(cell, backend=backend, realtime=False, bind=profile)  # inproc -> dora: only backend= changes
    dep.place(cell.arm, process="controller", hz=100)
    dep.place(cell.targets, hz=2)
    dep.place(cell.planner)  # rate-less: steps once per target
    return dep


def run(profile_name: str = "sim", backend_name: str = "inproc", until: float = 3.0, directory=None) -> dict:
    """Run the cell in lockstep for `until` logical seconds and return what it sent, decoded."""
    import robot_msgs
    from like_keras import codec, record

    directory = pathlib.Path(directory or tempfile.mkdtemp(prefix="pick_cell-"))
    directory.mkdir(parents=True, exist_ok=True)
    dep = deployment(profiles(directory)[profile_name], backend_name)
    sheet = dep.describe()
    dep.record(directory / "run.mcap")
    try:
        dep.start(until=until)
    finally:
        dep.stop()
    channels, messages = record.read(directory / "run.mcap")
    types = robot_msgs.message_types()
    streams: dict[str, list] = {}
    for channel, publish_ns, frame in messages:
        topic, kind = channels[channel]
        streams.setdefault(topic, []).append((publish_ns, frame, codec.decode(frame, types[kind])))
    return {"sheet": sheet, "streams": streams, "profile": profile_name, "backend": backend_name, "until": until}


def degrees(values) -> str:
    return "[" + ", ".join(f"{value * 57.29577951308232:+7.2f}" for value in values) + "]"


def report(result: dict) -> str:
    """What the cell sent, as text: the bindings, every plan, the commands and the stream's hash."""
    sheet, streams = result["sheet"], result["streams"]
    lines = [f"pick_cell on {result['backend']}, bind={result['profile'].upper()}, {result['until']} s in lockstep"]
    for node, entry in sheet["components"].items():
        role = f"  (role {entry['role']})" if "role" in entry else ""
        lines.append(f"  {node:<26} {entry['language']:<6} {entry['impl']:<52} {entry['bound_by']}{role}")
    plans = streams.get("pick_cell.planner.trajectory", [])
    lines.append(f"plans sent by the planner: {len(plans)}")
    for publish_ns, _, plan in plans:
        lines.append(f"  sent t={publish_ns / 1e9:6.3f} s  starts {plan.header.stamp:6.3f} s  {len(plan.t)} samples  "
                     f"ends at {degrees(plan.positions[-1])} deg")
    commands = streams.get("pick_cell.command", [])
    digest = hashlib.sha256(b"".join(frame for _, frame, _ in commands)).hexdigest()[:16]
    lines.append(f"commands to the arm: {len(commands)}, sha256 {digest}")
    for publish_ns, _, command in commands[:: max(1, len(commands) // 6)][:6] + commands[-1:]:
        lines.append(f"  t={command.header.stamp:6.3f} s  position {degrees(command.position)} deg")
    states = streams.get("pick_cell.state", [])
    if states:
        lines.append(f"arm state at t={states[-1][2].header.stamp:.3f} s: {degrees(states[-1][2].position)} deg")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--backend", default="inproc", choices=["inproc", "dora"])
    parser.add_argument("--profile", default="sim", choices=["sim", "test", "cpp"])
    parser.add_argument("--until", type=float, default=3.0)
    parser.add_argument("--directory", help="where the board file and run.mcap go (default: a new temp directory)")
    parser.add_argument("--private-dora", action="store_true", help="start a private dora coordinator and daemon")
    parser.add_argument("--summary", action="store_true", help="print pick_cell().summary() first")
    arguments = parser.parse_args()
    if arguments.summary:
        pick_cell().summary()
    private = None
    if arguments.private_dora:
        from kpi.support.dora_private import PrivateDora

        private = PrivateDora(REPO / "build" / "kpi" / f"dora-ex-{__import__('os').getpid()}").start()
    try:
        print(report(run(arguments.profile, arguments.backend, arguments.until, arguments.directory)))
    finally:
        if private is not None:
            private.stop()


if __name__ == "__main__":  # WHY through the package: under dora a component is rebuilt as module:qualname, and
    from examples.pick_cell import cell  # `__main__:OneTarget` names nothing in another process

    cell.main()
