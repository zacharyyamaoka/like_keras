"""like_keras.hardware_interface: one robot_msgs `HardwareInterface` that is a pipe to an always-on core
(serve_hardware_interface) which owns the hardware.

The proof: the JCC bring-up loop (joint_command_controller_agent/bringup/run.py: submit, read, tick, write, read)
runs once on an in-process HardwareInterface and once through the attached backend against a served copy of the same
deterministic plant, and every state read must match; truth passes through from a sim core and is stripped from a
real one; a second writer is refused while the first holds the lease; killing a client leaves the core serving;
a dropped connection raises instead of handing on a stale state; a core that describes another
robot is refused before anything is written. No hardware, no bus, nothing under /dev.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_attached_hardware_interface.py
"""

from __future__ import annotations

import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import pytest
from robot_msgs.control import (
    GroundTruth,
    HardwareDeployment,
    HardwareInterface,
    JointCommandController,
    JointCommandSequence,
    JointTable,
    Servo,
    ServoCommand,
    ServoState,
)
from robot_msgs.control.helpers import define, hold, move_by, wait
from robot_msgs.std import Header

from like_keras.hardware_interface import (
    AttachedBackend,
    DescriptionMismatch,
    LeaseHeld,
    attached_hardware_interface,
    serve_hardware_interface,
)

REPO = pathlib.Path(__file__).resolve().parents[1]
# WHY a sign and an offset: the core applies its table once; if the attached side applied one too, j1 and j2
# would come back shifted or mirrored and the equality below would fail
TABLE = JointTable(bus="rig", joints={"j1": Servo(can_id=11, offset_rad=0.3), "j2": Servo(can_id=12, sign=-1.0)})


class Plant:
    """A deterministic two-servo plant as a HardwareBackend (servo keys both ways): a moteus-like PD per board with a
    torque cap, `define` shifting the board's reading, and the joint-frame truth on every read. One `read()` is one
    period, as bringup/sim.PlantBackend does it."""

    period_s = 0.002

    def __init__(self, table: JointTable, *, simulated: bool = True) -> None:
        self.table, self.simulated = table, simulated
        self.keys = list(table.addresses())
        self.q = np.array([0.10, -0.20])  # the shaft, in each board's frame
        self.qd = np.zeros(2)
        self.bias = np.array([0.05, 0.0])  # the board reads q + bias until a define moves it
        self.command, self.t, self.seq, self.tau = None, 0.0, 0, np.zeros(2)

    def write(self, command: ServoCommand, defines=None) -> None:
        self.command = command
        for key, value in (defines or {}).items():
            index = self.keys.index(key)
            self.bias[index] = float(value) - self.q[index]

    def read(self) -> ServoState:
        self.t += self.period_s
        for _ in range(4):
            self._integrate(self.period_s / 4)
        self.seq += 1
        joints = tuple(self.table.joint_at(key) for key in self.keys)
        sign = np.array([self.table.joints[j].sign for j in joints])
        offset = np.array([self.table.joints[j].offset_rad for j in joints])
        truth = GroundTruth(name=joints, position=sign * self.q + offset, velocity=sign * self.qd, source="test plant")
        return ServoState(name=tuple(self.keys), position=self.q + self.bias, velocity=self.qd.copy(),
                          effort=self.tau.copy(), seq=self.seq, header=Header(stamp=self.t), truth=truth)

    def _integrate(self, dt: float) -> None:
        tau = np.zeros(2)
        command = self.command
        for i, key in enumerate(self.keys):
            if command is None or key not in command.name:
                continue
            k = command.name.index(key)
            mode = command.mode_of(key) if command.mode is not None else "position"

            def pick(values, default, k=k):
                value = None if values is None else float(values[k])
                return default if value is None or np.isnan(value) else value

            if mode in ("stop", "limp"):
                continue
            target = float(command.position[k]) - self.bias[i]
            kp = 0.0 if np.isnan(target) else pick(command.stiffness, 40.0)
            kd = pick(command.damping, 2.0)
            effort = kp * ((0.0 if np.isnan(target) else target) - self.q[i]) + kd * (
                pick(command.velocity, 0.0) - self.qd[i]) + pick(command.feedforward_effort, 0.0)
            cap = pick(command.max_effort, 5.0)
            tau[i] = max(-cap, min(cap, effort))
        self.qd += (tau - 0.3 * self.qd) / 0.05 * dt  # inertia 0.05, viscous 0.3
        self.q += self.qd * dt
        self.tau = tau


SEQUENCE = JointCommandSequence(name="move, define, move back", steps=[
    hold("j2", 5.0),
    move_by("j1", +0.4, max_vel=1.0, max_effort=5.0),
    define("j1", 1.0),
    move_by("j1", -0.25, max_vel=1.0, max_effort=5.0),
    wait(0.1),
])


def drive(arm: HardwareInterface, sequence: JointCommandSequence = SEQUENCE, max_seconds: float = 10.0):
    """bringup/run.py's loop, unchanged: submit, read, then tick / write / read until the controller is done."""
    controller = JointCommandController(arm.joint_names)
    controller.submit(sequence)
    states, defined = [arm.read()], 0
    while controller.status == "executing" and states[-1].header.stamp < max_seconds:
        tick = controller.tick(states[-1], arm.period_s)
        arm.write(tick.command, tick.defines)
        defined += bool(tick.defines)
        states.append(arm.read())
    return controller.status, states, defined


@pytest.fixture
def runtime_dir(monkeypatch):
    """A private, short XDG_RUNTIME_DIR: the attach socket lives here (a unix path holds at most 107 bytes)."""
    with tempfile.TemporaryDirectory(prefix="lk-arm-") as path:
        monkeypatch.setenv("XDG_RUNTIME_DIR", path)
        yield path


RIG = HardwareDeployment("rig", TABLE, backends={"sim": lambda table, simulated=True: Plant(table, simulated=simulated)})


def serve(name: str, *, simulated: bool = True, stepped: bool | None = None, hz: float | None = None,
          description: str | None = None):
    core = HardwareInterface(RIG, backend="sim", simulated=simulated)  # its robot name is the deployment's: "rig"
    dep = serve_hardware_interface(core, name, description=description, stepped=stepped, hz=hz)
    address = dep.describe()["attach"]["socket"]
    dep.start()
    return dep, address


def assert_same(expected: list[ServoState], got: list[ServoState]) -> None:
    assert len(got) == len(expected), f"{len(got)} reads attached, {len(expected)} in-process"
    for index, (a, b) in enumerate(zip(expected, got, strict=True)):
        assert a.name == b.name, (index, a.name, b.name)
        assert a.header.stamp == pytest.approx(b.header.stamp, abs=1e-12), index
        for field in ("position", "velocity", "effort"):
            np.testing.assert_allclose(getattr(b, field), getattr(a, field), atol=1e-9, rtol=0,
                                       err_msg=f"read {index}: {field}")
        assert b.truth is not None, f"read {index}: a sim core's truth did not pass through"
        assert b.truth.name == a.truth.name
        np.testing.assert_allclose(b.truth.position, a.truth.position, atol=1e-9, rtol=0, err_msg=f"truth {index}")


def test_same_loop_in_process_or_attached(runtime_dir):
    status, expected, defined = drive(HardwareInterface(TABLE, backend=Plant(TABLE)))
    assert status == "done" and defined >= 1 and len(expected) > 200, (status, defined, len(expected))
    assert expected[-1].truth is not None

    dep, address = serve(f"hw_{os.getpid()}")
    try:
        arm = attached_hardware_interface(address, robot="rig")
        assert arm.name == "rig" and arm.simulated and arm.period_s == Plant.period_s and arm.joint_names == ("j1", "j2")
        status_attached, got, defined_attached = drive(arm)
        arm.close()
    finally:
        dep.stop()
    assert (status_attached, defined_attached) == (status, defined)
    assert_same(expected, got)


def test_deployment_entry_cancels_the_client_table(runtime_dir):
    """HardwareInterface(DEPLOYMENT, backend="attached", address=...): the client's table only names joints."""
    deployment = HardwareDeployment("rig", TABLE, backends={"sim": lambda table: Plant(table), "attached": AttachedBackend})
    _, expected, _ = drive(HardwareInterface(deployment, backend="sim"))
    dep, address = serve(f"entry_{os.getpid()}")
    try:
        with HardwareInterface(deployment, backend="attached", address=address, robot="rig") as arm:
            status, got, _ = drive(arm)
    finally:
        dep.stop()
    assert status == "done"
    assert_same(expected, got)


def test_a_real_core_hands_on_no_truth(runtime_dir):
    dep, address = serve(f"real_{os.getpid()}", simulated=False, stepped=True)
    try:
        with attached_hardware_interface(address) as arm:
            assert not arm.simulated
            status, states, _ = drive(arm)
    finally:
        dep.stop()
    assert status == "done" and all(state.truth is None for state in states)


def test_a_second_writer_is_refused_while_the_first_holds_the_lease(runtime_dir):
    dep, address = serve(f"lease_{os.getpid()}")
    try:
        first = attached_hardware_interface(address)
        state = first.read()
        hold_here = ServoCommand(name=state.name, position=state.position)
        first.write(hold_here)
        first.read()
        # WHY attach the second only now: it then holds just the core's cached state, with no step still in flight
        second = attached_hardware_interface(address)
        second.read()  # reading is open to everyone
        second.write(hold_here)
        began = time.monotonic()
        with pytest.raises(LeaseHeld, match=rf"held by attached pid {os.getpid()}"):
            second.read()
        assert time.monotonic() - began < 0.9, "the refusal must arrive, not a read timeout"
        for _ in range(20):  # the holder is unaffected
            first.write(hold_here)
            first.read()
        first.close()
        second.close()
    finally:
        dep.stop()


CHILD = """
import sys, numpy as np
from robot_msgs.control import ServoCommand
from like_keras.hardware_interface import attached_hardware_interface
arm = attached_hardware_interface(sys.argv[1])
state = arm.read()
target = ServoCommand(name=state.name, position=np.array(state.position) + np.array([0.3, -0.3]))
for n in range(100000):
    arm.write(target)
    arm.read()
    if n == 20:
        print("writing", flush=True)
"""


def test_killing_a_client_leaves_the_core_running(runtime_dir):
    dep, address = serve(f"kill_{os.getpid()}", stepped=False, hz=200)
    env = {**os.environ, "PYTHONPATH": str(REPO / "src")}
    child = subprocess.Popen([sys.executable, "-c", CHILD, address], stdout=subprocess.PIPE, text=True, env=env)
    try:
        assert child.stdout.readline().strip() == "writing", "the child never took the lease"
        with attached_hardware_interface(address) as watcher:
            state = watcher.read()
            with pytest.raises(LeaseHeld):  # the child holds the lease while it lives
                watcher.write(ServoCommand(name=state.name, position=state.position))
                for _ in range(10):
                    watcher.read()
            child.send_signal(signal.SIGKILL)
            child.wait(timeout=5)
            seqs = [watcher.read().seq for _ in range(20)]  # the core keeps serving, 200 Hz, fresh every read
            assert seqs == sorted(seqs) and len(set(seqs)) == 20, seqs
            time.sleep(0.25 + 0.2)  # the child's lease lapses after lease_s: the core retracts and holds
            state = watcher.read()
            goal = ServoCommand(name=state.name, position=np.array(state.position) + np.array([-0.2, 0.0]))
            for _ in range(300):  # 1.5 s at the core's rate: now this client's lease
                watcher.write(goal)
                state = watcher.read()
            np.testing.assert_allclose(state.position[0], goal.position[0], atol=0.02)
    finally:
        if child.poll() is None:
            child.kill()
        dep.stop()


def test_a_dropped_connection_raises_instead_of_reading_stale(runtime_dir):
    dep, address = serve(f"drop_{os.getpid()}", stepped=False, hz=200)
    arm = attached_hardware_interface(address, timeout_s=5.0)
    try:
        arm.read()
    finally:
        dep.stop()
    began = time.monotonic()
    with pytest.raises(ConnectionError, match="lost the connection"):
        for _ in range(10):  # a state already in flight may still be handed on; then it must raise
            arm.read()
    assert time.monotonic() - began < 1.0, "the loss must raise at once, not after timeout_s"
    with pytest.raises(ConnectionError):
        arm.write(ServoCommand(name=("j1",), position=np.array([0.0])))
    arm.close()


def test_an_address_that_serves_no_arm_is_refused(runtime_dir):
    import like_keras as lk
    from kpi.consumer.lease_parts import TwoInputs

    s = lk.System(f"notarm_{os.getpid()}")
    box = s.add(TwoInputs(path=os.path.join(runtime_dir, "rows.jsonl")), "box")
    s.expose(box.a, box.b)
    dep = lk.Deployment(s)
    dep.place(s.box, hz=50)
    address = dep.describe()["attach"]["socket"]
    dep.start()
    try:
        with pytest.raises(TypeError, match="is not a served hardware interface"):
            attached_hardware_interface(address)
    finally:
        dep.stop()


def test_a_core_describing_another_robot_is_refused_before_any_write(runtime_dir):
    """The handshake: robot name, joint names (in order) and an optional description fingerprint must all match."""
    dep, address = serve(f"print_{os.getpid()}", description="urdf sha256 1234")
    try:
        with attached_hardware_interface(address, robot="rig", description="urdf sha256 1234") as hw:
            hw.read()  # the same robot: accepted
        with pytest.raises(DescriptionMismatch, match=r"robot 'other_rig' != the core's 'rig'"):
            attached_hardware_interface(address, robot="other_rig", description="urdf sha256 1234")
        with pytest.raises(DescriptionMismatch, match=r"description 'urdf sha256 9999'"):
            attached_hardware_interface(address, robot="rig", description="urdf sha256 9999")
        with pytest.raises(DescriptionMismatch, match=r"description None"):
            attached_hardware_interface(address, robot="rig")  # the core asks for a description the client lacks
        other = JointTable(bus="rig", joints={"j1": Servo(can_id=11), "wrist": Servo(can_id=13)})
        with pytest.raises(DescriptionMismatch, match=r"joints \('j1', 'wrist'\)"):
            AttachedBackend(other, address=address, robot="rig", description="urdf sha256 1234")
        swapped = JointTable(bus="rig", joints={"j2": Servo(can_id=12), "j1": Servo(can_id=11)})
        with pytest.raises(DescriptionMismatch, match="joints"):  # same joints, another order: another robot
            AttachedBackend(swapped, address=address, robot="rig", description="urdf sha256 1234")
        with attached_hardware_interface(address, robot="rig", description="urdf sha256 1234") as hw:
            state = hw.read()  # a refused handshake took no lease: the next client writes at once
            hw.write(ServoCommand(name=state.name, position=state.position))
            hw.read()
    finally:
        dep.stop()
