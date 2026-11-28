"""Read and write through one HardwareInterface on backends 1-4, the real backend's lock refusal, and the remote fingerprint refusal.
No hardware: the real backend is only asked to refuse.

    bash /home/bam/like_keras/examples/hw_backends/run.sh test
"""

from __future__ import annotations

import fcntl
import math

import pytest
from robot_msgs.control import BackendUnavailable, JointTable, Servo

from examples.hw_backends.backends import (AnalyticalConfig, KinematicConfig, MujocoConfig, RealConfig, RemoteConfig, hardware,
                                           hold)
from examples.hw_backends.description import JOINT, TABLE, nominal, robot_fingerprint, twin_of_record

STEP = math.radians(2.0)


def _cases():
    return [
        pytest.param(lambda: hardware(MujocoConfig(), nominal()), 0.5, id="1-mujoco-nominal"),
        pytest.param(lambda: hardware(MujocoConfig(), twin_of_record()), 0.1, id="2-mujoco-twin"),
        pytest.param(lambda: hardware(AnalyticalConfig(), nominal()), 0.5, id="3-analytical"),
        pytest.param(lambda: hardware(KinematicConfig()), 1.0, id="4-kinematic"),
    ]


@pytest.mark.parametrize(("build", "moved_at_least"), _cases())
def test_read_write(build, moved_at_least):
    """Joint-named in and out; a 2 deg step moves the joint toward it within 1 s (kinematic: at once, exactly)."""
    with build() as hw:
        assert hw.simulated
        state = hw.read()
        assert state.name == (JOINT,) and math.isfinite(float(state.position[0]))
        q0 = float(state.position[0])
        command = hold(state, STEP)
        for _ in range(int(1.0 / hw.period_s)):
            hw.write(command)
        moved = float(hw.read().position[0]) - q0
        assert moved >= moved_at_least * STEP
        if hw.backend_name == "kinematic":
            assert moved == pytest.approx(STEP)


def test_nominal_mujoco_matches_analytical():
    """The two frictionless plants are one pendulum: MuJoCo (bam_rack_twin's stack) and the host ODE agree on a 2 deg step."""
    finals = []
    for config in (MujocoConfig(), AnalyticalConfig()):
        with hardware(config, nominal()) as hw:
            command = hold(hw.read(), STEP)
            for _ in range(100):
                hw.write(command)
            finals.append(float(hw.read().position[0]))
    assert abs(finals[0] - finals[1]) < math.radians(0.05)


def test_same_backend_two_descriptions():
    """No friction vs realistic is a parameter change, not a backend change (description.py WHY)."""
    a, b = hardware(MujocoConfig(), nominal()), hardware(MujocoConfig(), twin_of_record())
    try:
        assert type(a.backend) is type(b.backend)
        assert nominal().parameters_digest != twin_of_record().parameters_digest
    finally:
        a.close()
        b.close()


def test_real_refuses_when_the_rack_lock_is_held():
    """A rack session holding the adapter's lock makes the real backend refuse before it opens the port."""
    from examples.pendulum.bus import RACK_LOCK

    with open(RACK_LOCK, "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            pytest.skip("a rack session holds the lock right now")
        try:
            with pytest.raises(BackendUnavailable, match="is held"):
                hardware(RealConfig())
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def test_fingerprint_names_the_robot():
    other = JointTable(bus="fdcanusb:272813EF", joints={JOINT: Servo(can_id=15)})
    assert robot_fingerprint() == robot_fingerprint(TABLE)
    assert robot_fingerprint(other) != robot_fingerprint()


def test_remote_refuses_a_wrong_fingerprint(tmp_path):
    """A core serving the pendulum refuses a client that expects another robot, before anything is written."""
    attached = pytest.importorskip("like_keras.hardware_interface", reason="like_keras AttachedBackend not on main yet")
    from examples.hw_backends.backends import serve

    with hardware(KinematicConfig()) as hw:
        dep = serve(hw)
        dep.start()
        try:
            address = dep.describe()["attach"]["socket"]
            with pytest.raises(attached.DescriptionMismatch):
                hardware(RemoteConfig(address, fingerprint="0" * 64))
            with hardware(RemoteConfig(address, fingerprint=robot_fingerprint())) as remote:
                state = remote.read()
                remote.write(hold(state, STEP))
                assert float(remote.read().position[0]) == pytest.approx(float(state.position[0]) + STEP)
        finally:
            dep.stop()
