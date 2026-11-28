"""Six backends behind one robot_msgs `HardwareInterface` for the rack pendulum (CAN 11); the config object picks the backend.

    hw = hardware(MujocoConfig(), NOMINAL)          # 1. MuJoCo, frictionless nominal parameters
    hw = hardware(MujocoConfig(), TWIN)             # 2. MuJoCo, the realistic twin of record: same backend, other parameters
    hw = hardware(AnalyticalConfig(), NOMINAL)      # 3. the closed-form pendulum ODE, integrated on the host
    hw = hardware(KinematicConfig())                # 4. measured = commanded, for the fastest plan checks
    hw = hardware(RealConfig())                     # 5. the real board on fdcanusb 272813EF, under the rack's bus lock
    hw = hardware(RemoteConfig(address, fingerprint))   # 6. a pipe to a core that serves any of 1-5 (like_keras AttachedBackend)
    state = hw.read(); hw.write(command)

WHY a frozen Config per backend, and the config selecting it (Zach, Daily Note 2026-11-28, the six backends): a config is HOW a
run happens (gui, realtime factor, timestep, seed, which adapter); the description (description.py) is WHAT is simulated. One
type per backend keeps each backend's knobs its own (a timestep means nothing to the real board), and `hardware(config, ...)`
dispatches on the type, so switching backend is switching one value.

Every backend here is a robot_msgs `HardwareBackend`: `read()` the newest `ServoState`, `write(command)`, `period_s`,
`simulated`, servo-keyed (``"fdcanusb:272813EF#11"``). Time: a simulated backend advances exactly one `period_s` of plant time per
`write()` (a bus exchange is what moves time on the rig too), so a script's loop is the simulator's clock; `realtime_factor`
paces it against the wall when set. `read()` never advances time.
"""

from __future__ import annotations

import asyncio
import fcntl
import math
import tempfile
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from robot_msgs.control import BackendUnavailable, HardwareDeployment, HardwareInterface, JointTable, ServoCommand, ServoState
from robot_msgs.std import Header

from examples.hw_backends.description import ADAPTER, CAN_ID, ROBOT, TABLE, TWIN_JOINT, PendulumDescription, robot_fingerprint

TAU = 2.0 * math.pi


# ---------------------------------------------------------------- configs: how a run happens


@dataclass(frozen=True)
class MujocoConfig:
    """MuJoCo (bam_sim's MjcfEngine + the moteus PID twin at 30 kHz, bam_rack_twin's scene). `physics_dt_s` must divide `period_s`."""

    period_s: float = 0.01
    physics_dt_s: float = 0.0002  # bam_rack_twin DEFAULT_PHYSICS_DT_S: dt*kd/I stays far under 1
    realtime_factor: float | None = None  # None = as fast as it computes; 1.0 = wall-clock pace
    gui: bool = False  # mujoco.viewer passive window, synced after every write
    seed: int = 0
    encoder_noise_rad: float = 0.0


@dataclass(frozen=True)
class AnalyticalConfig:
    """The closed-form pendulum ODE, RK4 on the host."""

    period_s: float = 0.01
    substeps: int = 20
    realtime_factor: float | None = None
    seed: int = 0
    encoder_noise_rad: float = 0.0


@dataclass(frozen=True)
class KinematicConfig:
    """Measured = commanded: no plant at all."""

    period_s: float = 0.01


@dataclass(frozen=True)
class RealConfig:
    """The rack's CAN 11 board. `window_rad` clamps every target to the opening position +- it: a machine limit for this
    measurement (Zach's rule for it: hold in place, no motion beyond a few degrees), not a planner's."""

    can_id: int = CAN_ID
    adapter: str = ADAPTER
    window_rad: float = math.radians(3.0)
    cap_nm: float = 3.0
    watchdog_s: float = 0.25
    period_s: float = 0.01


@dataclass(frozen=True)
class RemoteConfig:
    """A core serving one of 1-5 at `address` (``unix:<path>``); `fingerprint` is what this client expects the core's robot to be
    (description.robot_fingerprint); a core serving another robot is refused before anything is written."""

    address: str
    fingerprint: str
    robot: str = ROBOT
    lease_s: float = 0.25
    timeout_s: float = 2.0


# ---------------------------------------------------------------- shared simulated-backend plumbing


class _Simulated:
    simulated = True

    def __init__(self, table: JointTable, period_s: float, realtime_factor: float | None, seed: int, noise: float) -> None:
        self._key = (table.key(table.names[0]),)
        self.period_s = float(period_s)
        self._rt = realtime_factor
        self._rng = np.random.default_rng(seed)
        self._noise = float(noise)
        self._seq = 0
        self._sim_t = 0.0
        self._wall0: float | None = None
        self._q = self._qd = self._tau = 0.0

    def _target(self, command: ServoCommand, cap: float) -> tuple[float, float, float, float]:
        position = float(command.position[0])
        velocity = 0.0 if command.velocity is None else float(command.velocity[0])
        effort = 0.0 if command.feedforward_effort is None else float(command.feedforward_effort[0])
        if command.max_effort is not None and math.isfinite(float(command.max_effort[0])):
            cap = min(cap, float(command.max_effort[0]))
        return position, velocity, effort, cap

    def _pace(self) -> None:
        self._sim_t += self.period_s
        if self._rt:
            now = time.perf_counter()
            if self._wall0 is None:
                self._wall0 = now - self._sim_t / self._rt
            ahead = self._sim_t / self._rt - (now - self._wall0)
            if ahead > 0:
                time.sleep(ahead)

    def read(self) -> ServoState:
        q = self._q + (self._rng.normal(0.0, self._noise) if self._noise else 0.0)
        return ServoState(name=self._key, position=np.array([q]), velocity=np.array([self._qd]), effort=np.array([self._tau]),
                          seq=self._seq, header=Header(stamp=self._sim_t))


# ---------------------------------------------------------------- 1 + 2: MuJoCo (nominal or twin of record: the same code)


class MujocoBackend(_Simulated):
    """bam_rack_twin's own stack, stepped one period per write: the scene `write_twin_scene` writes from the description's
    TwinParameters, bam_sim's MjcfEngine, and the moteus PID twin (MoteusMjcfAdapter at 30 kHz) with the twin's actuator seam.

    WHY wrapped, not rewritten: the realistic twin (LuGre friction, 200 Hz current loop, current limit, 2 ms command latch) is
    bam_rack_twin's, graded by its gates; replaying its physics in a new MuJoCo file would be a second twin nobody graded. The
    only thing done here that bam_rack_twin's JointTwin.replay does per logged row is done per write instead: sample, latch the
    command `command_delay_s` late, step to the end of the period."""

    def __init__(self, table: JointTable, config: MujocoConfig, description: PendulumDescription) -> None:
        if description.twin is None or description.pid is None:
            raise BackendUnavailable("MuJoCo runs bam_rack_twin's twin: build the description with description.nominal() or "
                                     "twin_of_record() under bam_rack_twin's venv (run.sh)")
        super().__init__(table, config.period_s, config.realtime_factor, config.seed, config.encoder_noise_rad)
        from bam_rack_twin import twin as rack  # noqa: PLC0415
        from bam_sim.engine.mjcf import JointGroupBinding, MjcfEngine  # noqa: PLC0415
        from bam_sim.moteus import MoteusActuatorBinding, MoteusMjcfAdapter  # noqa: PLC0415

        steps = config.period_s / config.physics_dt_s
        if abs(steps - round(steps)) > 1e-9:
            raise ValueError(f"physics_dt_s {config.physics_dt_s} must divide period_s {config.period_s}")
        self.description, self.config = description, config
        twin = description.twin
        folder = Path(tempfile.gettempdir()) / "lk_hw_backends"
        scene = rack.write_twin_scene(None, TWIN_JOINT, twin, folder / f"can11_{twin.digest()}.xml", physics_dt_s=config.physics_dt_s)
        engine = MjcfEngine(scene, (JointGroupBinding(device_id=TWIN_JOINT, joints=(TWIN_JOINT,), command_mode="torque"),),
                            physics_dt_s=config.physics_dt_s)
        self._adapter = MoteusMjcfAdapter(engine, (MoteusActuatorBinding(
            device_id=TWIN_JOINT, joints=(TWIN_JOINT,), pid_configs=(description.pid,), control_rate_hz=rack.MOTEUS_CONTROL_RATE_HZ,
            actuator_model=twin.joint_effects(0.0)),))
        self._adapter.reset_to_joint_positions({TWIN_JOINT: description.bottom_rad})
        self._adapter.apply_device_commands({TWIN_JOINT: {"mode": "torque", "torque": 0.0}})  # a board at power-up: stopped
        self._delay = round(float(twin.command_delay_s or 0.0) / config.physics_dt_s) * config.physics_dt_s
        self._viewer = None
        if config.gui:
            import mujoco.viewer  # noqa: PLC0415

            self._viewer = mujoco.viewer.launch_passive(engine._model, engine._data)
        self._sample()

    def _sample(self) -> None:
        observation = self._adapter.device_observations()[TWIN_JOINT]
        self._q, self._qd = float(observation["position"][0]), float(observation["velocity"][0])
        self._tau = float(observation["requested_torque"][0])

    def write(self, command: ServoCommand, defines: Mapping[str, float] | None = None) -> None:
        position, velocity, effort, cap = self._target(command, self.description.cap_nm)
        board = {"mode": "combined", "position": (position,), "velocity": (velocity,), "feedforward_torque": (effort,),
                 "kp_scale": (1.0,), "kd_scale": (1.0,), "ilimit_scale": (0.0,), "maximum_torque": (cap,)}
        if self._delay > 0.0:
            self._adapter.step(self._delay)  # WHY: the board latches a command command_delay_s after it arrives (twin.py)
        self._adapter.apply_device_commands({TWIN_JOINT: board})
        self._adapter.step(self.period_s - self._delay)
        self._seq += 1
        self._sample()
        if self._viewer is not None:
            self._viewer.sync()
        self._pace()

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
        self._adapter.close()


# ---------------------------------------------------------------- 3: analytical


class AnalyticalBackend(_Simulated):
    """``I q'' = tau - G sin(q - bottom) - b q' - Fc tanh(q' / 0.01)``, tau = the board's PD + ff capped, RK4 at `substeps` per
    period. No MuJoCo, no bam_rack_twin: the description's numbers only (so a cloud VM runs it from `numbers()`)."""

    def __init__(self, table: JointTable, config: AnalyticalConfig, description: PendulumDescription) -> None:
        super().__init__(table, config.period_s, config.realtime_factor, config.seed, config.encoder_noise_rad)
        self.description, self.config = description, config
        self._q = description.bottom_rad
        self._command: tuple[float, float, float, float] | None = None

    def _qdd(self, q: float, qd: float) -> tuple[float, float]:
        d = self.description
        if self._command is None:
            tau = 0.0  # stopped until the first command
        else:
            position, velocity, effort, cap = self._command
            tau = max(-cap, min(cap, d.kp_nm_rad * (position - q) + d.kd_nm_s_rad * (velocity - qd) + effort))
        friction = d.coulomb_nm * math.tanh(qd / 0.01) if d.coulomb_nm else 0.0
        return (tau - d.gravity_nm * math.sin(q - d.bottom_rad) - d.damping_nm_s_rad * qd - friction) / d.inertia_kg_m2, tau

    def write(self, command: ServoCommand, defines: Mapping[str, float] | None = None) -> None:
        self._command = self._target(command, self.description.cap_nm)
        h = self.period_s / self.config.substeps
        q, qd = self._q, self._qd
        for _ in range(self.config.substeps):  # classic RK4 on (q, q')
            a1, tau = self._qdd(q, qd)
            a2, _ = self._qdd(q + 0.5 * h * qd, qd + 0.5 * h * a1)
            v2 = qd + 0.5 * h * a1
            a3, _ = self._qdd(q + 0.5 * h * v2, qd + 0.5 * h * a2)
            v3 = qd + 0.5 * h * a2
            a4, _ = self._qdd(q + h * v3, qd + h * a3)
            v4 = qd + h * a3
            q, qd = q + h * (qd + 2 * v2 + 2 * v3 + v4) / 6.0, qd + h * (a1 + 2 * a2 + 2 * a3 + a4) / 6.0
        self._q, self._qd, self._tau = q, qd, tau
        self._seq += 1
        self._pace()


# ---------------------------------------------------------------- 4: kinematic


class KinematicBackend(_Simulated):
    """Perfect tracking: the next read is the command (position, velocity; effort = its feed-forward)."""

    def __init__(self, table: JointTable, config: KinematicConfig, description: PendulumDescription | None = None) -> None:
        super().__init__(table, config.period_s, None, 0, 0.0)
        self._q = 0.0 if description is None else description.bottom_rad

    def write(self, command: ServoCommand, defines: Mapping[str, float] | None = None) -> None:
        position, velocity, effort, _ = self._target(command, math.inf)
        self._q, self._qd, self._tau = position, velocity, effort
        self._seq += 1
        self._sim_t += self.period_s


# ---------------------------------------------------------------- 5: real


class RealBackend:
    """CAN 11 latency example using the adapter-specific lock and an opening fault check.

    The stock dispatch path separately requires the global rack lock or queue lease and a board-frame check.
    Unlike bus.py's one exchange per tick, `read()` and `write()` each make one CAN exchange so each latency is measured on its own.

    WHY no return-home here (bus.py has one): every target is clamped to the opening position +- `window_rad` (3 deg), and
    `close()` first holds at the opening position for 0.5 s, so the board stops where it was found (at rest at its bottom)."""

    simulated = False

    def __init__(self, table: JointTable, config: RealConfig, description: PendulumDescription | None = None) -> None:
        import moteus  # noqa: PLC0415

        from examples.pendulum.bus import RACK_LOCK  # noqa: PLC0415  WHY: the same lock file, so a rack session excludes this

        self.config, self.period_s = config, config.period_s
        self._key = (table.key(table.names[0]),)
        self._lock = open(RACK_LOCK, "a")  # noqa: SIM115  held until close()
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock.close()
            raise BackendUnavailable(f"{RACK_LOCK} is held: a rack session owns fdcanusb {config.adapter}") from None
        except BaseException:
            self._lock.close()
            raise
        self._moteus = moteus
        self._loop = None
        self._transport = None
        port = f"/dev/serial/by-id/usb-mjbots_fdcanusb_{config.adapter}-if00"

        async def connect():  # WHY inside the loop: moteus.Fdcanusb starts its reader task when it is built (bus.py)
            self._transport = moteus.Fdcanusb(port)
            return moteus.Controller(id=config.can_id, transport=self._transport)

        try:
            self._loop = asyncio.new_event_loop()
            self._controller = self._loop.run_until_complete(connect())
            self._seq = 0
            state = self._read()
            if int(state.fault[0]):
                raise BackendUnavailable(f"CAN {config.can_id} reports fault {int(state.fault[0])}: refusing to take over")
            self.home_rad = float(state.position[0])
        except BaseException as error:
            self._release_after_error(error)
            raise

    def _require_open(self) -> None:
        if self._lock.closed:
            raise BackendUnavailable(f"CAN {self.config.can_id} backend is closed")

    def _cycle(self, command):
        return self._loop.run_until_complete(self._transport.cycle([command]))

    def read(self) -> ServoState:
        self._require_open()
        try:
            return self._read()
        except BaseException as error:
            self._release_after_error(error)
            raise

    def _read(self) -> ServoState:
        replies = self._cycle(self._controller.make_query())
        values = replies[0].values if replies else {}
        position, velocity, torque, temperature, fault = (
            self._required_register(values, name) for name in ("POSITION", "VELOCITY", "TORQUE", "TEMPERATURE", "FAULT")
        )
        self._seq += 1
        return ServoState(name=self._key, position=np.array([position * TAU]),
                          velocity=np.array([velocity * TAU]), effort=np.array([torque]),
                          temperature=np.array([temperature]),
                          fault=np.array([int(fault)], dtype=np.int64), seq=self._seq,
                          header=Header(stamp=time.time()))

    def _required_register(self, values, name: str) -> float:
        value = values.get(getattr(self._moteus.Register, name))
        if value is None:
            raise BackendUnavailable(f"CAN {self.config.can_id} reply is missing {name}")
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise BackendUnavailable(f"CAN {self.config.can_id} reply has invalid {name}: {value!r}") from error
        if not math.isfinite(number) or (name == "FAULT" and not number.is_integer()):
            raise BackendUnavailable(f"CAN {self.config.can_id} reply has invalid {name}: {value!r}")
        return number

    def write(self, command: ServoCommand, defines: Mapping[str, float] | None = None) -> None:
        self._require_open()
        try:
            self._write(command)
        except BaseException as error:
            self._release_after_error(error)
            raise

    def _write(self, command: ServoCommand) -> None:
        position = float(command.position[0])
        low, high = self.home_rad - self.config.window_rad, self.home_rad + self.config.window_rad
        velocity = 0.0 if command.velocity is None else float(command.velocity[0])
        if not low <= position <= high:
            position, velocity = min(high, max(low, position)), 0.0
        effort = 0.0 if command.feedforward_effort is None else float(command.feedforward_effort[0])
        self._cycle(self._controller.make_position(position=position / TAU, velocity=velocity / TAU, feedforward_torque=effort,
                                                   maximum_torque=self.config.cap_nm, watchdog_timeout=self.config.watchdog_s,
                                                   query=False))

    def close(self, hold: bool = True) -> None:
        if self._lock.closed:
            return
        try:
            if hold:
                hold_here = ServoCommand(name=self._key, position=np.array([self.home_rad]), velocity=np.zeros(1))
                for _ in range(50):
                    self._write(hold_here)
                    time.sleep(0.01)
            self._loop.run_until_complete(self._controller.set_stop())
        except BaseException as error:
            self._release_after_error(error)
            raise
        self._release()

    def _release_after_error(self, error: BaseException) -> None:
        # A failed exchange is not evidence for another command. Teardown relies on the configured command watchdog.
        try:
            self._release()
        except Exception as cleanup_error:
            error.add_note(f"CAN {self.config.can_id} cleanup failed: {cleanup_error}")

    def _release(self) -> None:
        async def cancel_pending() -> None:
            tasks = asyncio.all_tasks() - {asyncio.current_task()}
            for task in tasks:
                task.cancel()
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)

        try:
            try:
                if self._transport is not None:
                    self._transport.close()
            finally:
                if self._loop is not None:
                    try:
                        self._loop.run_until_complete(cancel_pending())
                    finally:
                        self._loop.close()
        finally:
            self._lock.close()


# ---------------------------------------------------------------- 6: remote


def remote_backend(table: JointTable, config: RemoteConfig, description: PendulumDescription | None = None):
    """like_keras's AttachedBackend to a core serving one of 1-5 (`serve` below). Never a second protocol: if like_keras has no
    AttachedBackend yet, this backend is unavailable (the parallel agent builds it in like_keras src/)."""
    try:
        from like_keras.hardware_interface import AttachedBackend  # noqa: PLC0415
    except ImportError as error:
        raise BackendUnavailable(f"like_keras has no AttachedBackend yet ({error}): the remote backend waits on it") from None
    return AttachedBackend(table, address=config.address, robot=config.robot, description=config.fingerprint,
                           lease_s=config.lease_s, timeout_s=config.timeout_s)


def serve(hw: HardwareInterface, name: str = ROBOT, **options: Any):
    """A core: `hw` (any of 1-5) served on like_keras's attach socket, fingerprinted with the robot's fingerprint."""
    from like_keras.hardware_interface import serve_hardware_interface  # noqa: PLC0415

    return serve_hardware_interface(hw, name, description=robot_fingerprint(hw.joints), **options)


# ---------------------------------------------------------------- the deployment and the one-value switch


def _factory(cls):
    def build(table: JointTable, *, config, description: PendulumDescription | None = None):
        return cls(table, config, description)

    return build


PENDULUM = HardwareDeployment(ROBOT, TABLE, backends={
    "mujoco": _factory(MujocoBackend),
    "analytical": _factory(AnalyticalBackend),
    "kinematic": _factory(KinematicBackend),
    "real": _factory(RealBackend),
    "remote": remote_backend,
})
BACKEND_OF = {MujocoConfig: "mujoco", AnalyticalConfig: "analytical", KinematicConfig: "kinematic", RealConfig: "real",
              RemoteConfig: "remote"}


def hardware(config, description: PendulumDescription | None = None) -> HardwareInterface:
    """The pendulum on the backend `config` selects (module docstring)."""
    name = BACKEND_OF.get(type(config))
    if name is None:
        raise TypeError(f"no backend takes a {type(config).__name__}; one of {[c.__name__ for c in BACKEND_OF]}")
    if name in ("mujoco", "analytical") and description is None:
        raise TypeError(f"{name} needs a description (description.nominal() or twin_of_record())")
    return HardwareInterface(PENDULUM, backend=name, config=config, description=description)


def hold(state: ServoState, offset_rad: float = 0.0) -> ServoCommand:
    """A joint-named hold at `state`'s position (+ `offset_rad`), zero velocity."""
    return ServoCommand(name=state.name, position=np.array(state.position, dtype=np.float64) + offset_rad,
                        velocity=np.zeros(len(state.name)))


__all__ = ["AnalyticalConfig", "KinematicConfig", "MujocoConfig", "RealConfig", "RemoteConfig", "PENDULUM", "hardware", "hold",
           "serve"]
