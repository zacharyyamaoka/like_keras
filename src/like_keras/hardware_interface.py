"""An attached hardware interface: one robot_msgs `HardwareInterface` that is a remote connection to an always-on core
which already owns the hardware.

The core opens the hardware once and serves it on its attach socket; any number of scripts then attach, drive it
with the same loop they would run in-process, and leave:

    # the core (one process, up all day): the only place a bus is opened
    hw = HardwareInterface(BAM_FB, backend="real")
    dep = serve_hardware_interface(hw, "bam_fb")   # a Deployment: command + defines in, state out, on .../bam_fb.sock
    dep.start()

    # a script: the identical loop as in-process, against the core that is already up
    hw = attached_hardware_interface("unix:/run/user/1000/like_keras/bam_fb.sock", robot="bam_fb")
    controller.submit(sequence)
    state = hw.read()
    while controller.status == "executing":
        tick = controller.tick(state, hw.period_s)
        hw.write(tick.command, tick.defines)
        state = hw.read()

    # or as one more backend of the deployment, so the switch stays one word
    BAM_FB = HardwareDeployment("bam_fb", TABLE, backends={"sim": ..., "real": ..., "attached": AttachedBackend})
    hw = HardwareInterface(BAM_FB, backend="attached", address="unix:/run/user/1000/like_keras/bam_fb.sock")

WHY (Zach, Daily Note 2026-11-28): "One hardware interface can be just a remote connection to another hardware
interface that already exists!!!! That is how you can have like hardware interface that is always running and then
access it in a scripting like fashion." And the failure it must not repeat: a ROS-style remote arm "is a bit quite
fragile... you basically are making like a duplicate arm". So the attached side is a pipe and holds nothing:

- Only the core opens the bus. The wire is joint-named on both sides. The core's `HardwareInterface` applies its
  `JointTable` (CAN ids, signs, homed offsets) and whatever guard its backend has, once. The attached backend has
  no table, no limits and no model of its own: it undoes exactly the table the client's `HardwareInterface` put in
  front of it, so the client's table only names joints (an identity table from the core's joint names in
  `attached_hardware_interface`; the deployment's own table when used as a backend entry, whose signs and offsets
  then cancel instead of applying twice).
- Both sides confirm they describe the same robot before anything is written: the core publishes a description
  fingerprint (sha256 of its robot name, its joint names in order, and an optional caller-supplied description
  fingerprint such as a URDF hash) in its datasheet; the client recomputes it from what it expects and refuses
  with `DescriptionMismatch` on any difference. WHY not the table's signs and offsets: those are the core's
  runtime state (homing rewrites the offsets), not what robot it is.
- `simulated` comes from the core: a sim core's truth passes through, a real core's is stripped by both sides.
- Writes go under like_keras's write lease (attach.py, CONTRACT §9.3). A refused lease raises `LeaseHeld` naming
  the holder on the next call; it never hangs. A dropped connection makes `read()` raise; it never returns a stale
  state. A client that dies loses its lease after `lease_s`; the core retracts its command and holds (R5).
- Reuse, not a second protocol: the client is attach.py's `_Link`, the core an ordinary Deployment with one
  component, `ServedHardwareInterface`, around the core's `HardwareInterface`; the handshake rides on DESCRIBE.

Time: a served interface either steps once per command (`stepped=True`, the default for a simulated one: the
script's loop is the simulator's clock, as in-process, so the same script gives the same states attached or not),
or runs free at its own rate (`stepped=False`, the default for real hardware: the bus is fed every tick with the
newest command, and a client's `read()` returns the next sample, which paces the script's loop at the core's rate).
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os
import time
from collections.abc import Mapping, Sequence

import numpy as np
from robot_msgs.control import HardwareInterface, JointTable, Servo, ServoCommand, ServoState
from robot_msgs.sensor import JointState
from robot_msgs.std import Header

from like_keras.attach import _Link
from like_keras.component import Component, In, Out
from like_keras.deployment import Deployment
from like_keras.system import System

STEPPED_HZ = 1000.0  # how often a stepped core looks for the next command: at most 1 ms added per step


class LeaseHeld(RuntimeError):
    """Another client holds the core's write lease: this write was refused, nothing moved."""


class DescriptionMismatch(RuntimeError):
    """The core at this address describes another robot than the client expects: nothing was written."""


def description_fingerprint(robot: str, joints: Sequence[str], description: str | None = None) -> str:
    """sha256 over the robot's name, its joint names in order and an optional caller-supplied description
    fingerprint (a URDF or robot-description hash): what both ends compare before the first write."""
    text = json.dumps({"robot": str(robot), "joints": [str(joint) for joint in joints], "description": description})
    return hashlib.sha256(text.encode()).hexdigest()


# ---------------------------------------------------------------- the core's side


class ServedHardwareInterface(Component):
    """A robot_msgs `HardwareInterface` as a component: a joint-named `command` (and `defines`) in, its `state` out.

    WHY the interface is handed over by `serve_hardware_interface`, not a param: params are data in the datasheet;
    the interface owns a bus (or a simulator) in this process. The datasheet carries what a client needs instead:
    robot name, joint names, the description fingerprint, `simulated`, `period_s`, `stepped`."""

    command = In(ServoCommand)  # latest: a new object is a new command; None once its writer's lease lapsed
    defines = In(JointState)  # joint -> position declared with the next new command (homing's define)
    state = Out(ServoState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        robot: str = ""
        joints: tuple[str, ...] = ()
        description: str | None = None
        fingerprint: str = ""
        simulated: bool = False
        period_s: float = 0.0
        stepped: bool = False

    def activate(self) -> None:
        if getattr(self, "_hw", None) is None:
            raise RuntimeError("ServedHardwareInterface has no HardwareInterface: build it with "
                               "like_keras.hardware_interface.serve_hardware_interface(hw, name)")
        self._applied = self._defined = self._last = self._holding = None

    def forward(self, command, defines):
        hw = self._hw
        if command is not None and command is not self._applied:  # a new command: write it once with its defines
            self._applied, self._holding = command, None
            staged = None
            if defines is not None and defines is not self._defined:
                # WHY only with a new command: a define is an event (set the encoder here), re-applying it a tick
                # later would move the zero; the client sends it just before its command, so they pair up here
                self._defined = defines
                staged = dict(zip(defines.name, map(float, defines.position), strict=True))
            hw.write(command, staged)
        elif self.params.stepped:
            if self._last is not None:
                return None  # nothing new: a stepped interface waits for its client (its first tick reads once)
        elif command is not None:
            hw.write(command)  # free: the newest command again, every tick, as a bus stream (the board watchdog)
        elif self._applied is not None and self._last is not None:
            # WHY hold at the measured position (CONTRACT R5): the writer's lease lapsed (it crashed or left) and its
            # command was retracted; a real bus must not keep the last setpoint of a script that is gone
            if self._holding is None:
                last = self._last
                self._holding = ServoCommand(name=last.name, position=np.array(last.position, dtype=np.float64),
                                             velocity=np.zeros(len(last.name)), header=Header(stamp=self.now()))
                self.log.warning("command retracted (its writer left): holding at the measured position")
            hw.write(self._holding)
        self._last = hw.read()
        return self._last


def serve_hardware_interface(hw: HardwareInterface, name: str, *, description: str | None = None,
                             stepped: bool | None = None, hz: float | None = None) -> Deployment:
    """The core: `hw` served on `unix:$XDG_RUNTIME_DIR/like_keras/<name>.sock` (`describe()["attach"]["socket"]`).

    Returns the Deployment, not started (record, describe, then `start()`; `stop()` leaves `hw` open: its owner
    closes it). The robot name in the fingerprint is `hw.name` (its deployment's), else `name`; `description` is an
    optional extra fingerprint clients must match. `stepped` defaults to `hw.simulated` (see the module docstring);
    `hz` to `1 / hw.period_s` when free and to 1 kHz when stepped. In-process only: `hw` lives in this process."""
    stepped = bool(hw.simulated) if stepped is None else bool(stepped)
    if hz is None:
        hz = STEPPED_HZ if stepped else 1.0 / hw.period_s
    robot, joints = hw.name or name, tuple(hw.joint_names)
    s = System(name)
    served = ServedHardwareInterface(
        robot=robot, joints=joints, description=description,
        fingerprint=description_fingerprint(robot, joints, description),
        simulated=bool(hw.simulated), period_s=float(hw.period_s), stepped=stepped)
    served._hw = hw
    s.add(served, "hw")
    s.expose(served.command, served.defines, served.state)
    dep = Deployment(s)
    dep.place(s.hw, hz=hz)
    return dep


# ---------------------------------------------------------------- the client's side


def identity_table(joints: Sequence[str]) -> JointTable:
    """A table that only names joints: sign +1, offset 0, so mapping through it renames and changes no value.
    WHY not joint name == servo key: a JointTable's keys are "<bus>#<can_id>"; this is the nearest identity."""
    return JointTable(bus="attached", joints={joint: Servo(can_id=index) for index, joint in enumerate(joints)})


class AttachedBackend:
    """A `HardwareBackend` that is a pipe to a served interface (`serve_hardware_interface`) at `address`.

    `table` is the one the `HardwareInterface` in front of this backend maps through (a `HardwareDeployment`
    factory gets it first); `None` means `identity_table(core's joints)`. The handshake (module docstring) checks
    the table's joint names, `robot` when given (None accepts the core's name) and `description` when given.
    `timeout_s` bounds every `read()`: no new state within it raises, as a dropped connection does at once."""

    def __init__(self, table: JointTable | None = None, *, address: str, robot: str | None = None,
                 description: str | None = None, lease_s: float = 0.25, timeout_s: float = 1.0) -> None:
        self.address, self.timeout_s = address, float(timeout_s)
        self._link = _Link(address, lease_s, f"attached pid {os.getpid()}", ("state",))
        sheet = self._link.sheet
        try:
            leaf = sheet["exposed"]["state"].rsplit(".", 1)[0]
            params = sheet["components"][leaf]["params"]
            core = (params["robot"], tuple(params["joints"]), params["description"], params["fingerprint"])
            if "command" not in sheet["exposed"] or "defines" not in sheet["exposed"]:
                raise KeyError("command")
        except (KeyError, TypeError):
            self._link.close()
            raise TypeError(f"{address} ({sheet.get('system')!r}) is not a served hardware interface: serve it "
                            "with like_keras.hardware_interface.serve_hardware_interface(hw, name)") from None
        # WHY from the core: whether truth may pass is the core's fact (sim or real), never the client's claim
        self.simulated, self.period_s, self.system = bool(params["simulated"]), float(params["period_s"]), sheet["system"]
        self.robot = core[0]
        self.table = identity_table(core[1]) if table is None else table
        expected = (core[0] if robot is None else robot, tuple(self.table.names), description)
        if description_fingerprint(*expected) != core[3]:
            self._link.close()
            parts = [f"{what} {mine!r} != the core's {theirs!r}"
                     for what, mine, theirs in zip(("robot", "joints", "description"), expected, core, strict=False)
                     if mine != theirs]
            raise DescriptionMismatch(
                f"{address} serves another robot than this client describes ({'; '.join(parts) or 'fingerprint'}): "
                f"fingerprint {description_fingerprint(*expected)[:12]} != the core's {core[3][:12]}; nothing written")
        self._lost, self._broken = self._link.lost, None

    def read(self) -> ServoState:
        """The next state the core sends (joint-named there), in this table's servo keys. Raises, never stale."""
        link, deadline = self._link, time.monotonic() + self.timeout_s
        while True:
            link.arrived.clear()
            self._check()
            state = link.take("state")
            if state is not None:
                return self.table.to_servos(state)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no new state from the core at {self.address} within {self.timeout_s} s "
                                   "(it stopped stepping: see `lk top` on that address)")
            link.arrived.wait(remaining)

    def write(self, command: ServoCommand, defines: Mapping[str, float] | None = None) -> None:
        """Send `command` (servo keys of this table) to the core joint-named, under the write lease."""
        self._check()
        # WHY drop an unread state here: a read after a write answers that write (as in-process), so a sample that
        # arrived before it was sent is not handed on as its answer; a refused write then raises on that read
        self._link.take("state")
        table = self.table
        if defines:
            names, values = [], []
            for key, value in defines.items():
                joint = table.joint_at(key)
                servo = table.joints[joint]
                names.append(joint)
                values.append(servo.sign * float(value) + servo.offset_rad)  # HardwareInterface.write's mapping, undone
            self._link.write("defines", JointState(name=tuple(names), position=np.array(values)))
        self._link.write("command", table.to_joints(command))

    def close(self) -> None:
        if not self._link.closed:
            self._link.close()

    def _check(self) -> None:
        if self._broken is not None:
            raise self._broken
        link = self._link
        if link.lost != self._lost:
            # WHY sticky and closed: a reconnect would quietly resubscribe and hand on states from a core that may
            # have restarted under the script; the script must decide, so the backend stays broken
            self._broken = ConnectionError(f"lost the connection to the core at {self.address}")
            self.close()
            raise self._broken
        if link.refusals:
            refusal = link.refusals.popleft()
            raise LeaseHeld(f"write refused by {self.system!r} at {self.address}: the lease on "
                            f"{refusal.get('port')!r} is held by {refusal.get('holder')} ({refusal.get('reason')})")

    def __repr__(self) -> str:
        return f"AttachedBackend({self.address!r}, joints={list(self.table.names)}, simulated={self.simulated})"


def attached_hardware_interface(address: str, *, robot: str | None = None, description: str | None = None,
                                lease_s: float = 0.25, timeout_s: float = 1.0) -> HardwareInterface:
    """A `HardwareInterface` on the served interface at `address`, named by the core's own joints: a script needs
    the address and, to confirm it is the robot it means, `robot` (and `description`); the core owns the rest."""
    backend = AttachedBackend(address=address, robot=robot, description=description, lease_s=lease_s,
                              timeout_s=timeout_s)
    hw = HardwareInterface(backend.table, backend=backend)
    hw.name, hw.backend_name = backend.robot, "attached"
    return hw


__all__ = ["AttachedBackend", "DescriptionMismatch", "LeaseHeld", "ServedHardwareInterface",
           "attached_hardware_interface", "description_fingerprint", "identity_table", "serve_hardware_interface"]
