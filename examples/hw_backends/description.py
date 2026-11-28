"""The pendulum's description: one robot (rack CAN 11), two parameter sets, one fingerprint.

    NOMINAL = nominal()            # the twin of record's mass and gravity, every loss and board non-ideality off
    TWIN = twin_of_record()        # bam_rack_twin's twin of record for CAN 11 (followup C: LuGre friction, 2 ms latch, 200 Hz current loop)

WHY two parameter sets and not two backends (Zach, Daily Note 2026-11-28, "1. Mujoco (no friction) 2. Mujoco (with realistic
sim)"): "no friction" and "realistic" are the SAME MuJoCo backend run on different description PARAMETERS. The backend's config
(`MujocoConfig`: timestep, realtime factor, seed) is HOW a run happens; the description is WHAT is simulated. Swapping one for
the other never touches the backend's code, and the analytical backend reads the very same numbers.

WHY bam_rack_twin's TwinParameters and not new numbers: the realistic twin already exists (bam_ws src/dev/bam_rack_twin, the
station's twin of record, graded by its acceptance gates); this module wraps it. The nominal set is that twin with its friction,
damping, latch delay, current loop and current limit switched off (`dataclasses.replace`), so the two differ only in what is
"realistic", never in the robot.

WHY the fingerprint covers the robot, not the parameters: a client planning on the nominal sim and a core running the realistic
twin (or the real board) drive the same robot; what must match before the first write is the robot's name and its joints. The
parameter set's digest travels beside it (`parameters_digest`) so a report can say which twin answered, but it never refuses.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any

from robot_msgs.control import JointTable, Servo

ROBOT = "pendulum_can11"
JOINT = "pendulum"
CAN_ID = 11
ADAPTER = "272813EF"
TWIN_JOINT = "can11_hinge"  # the twin of record's hinge name in its MJCF
TABLE = JointTable(bus=f"fdcanusb:{ADAPTER}", joints={JOINT: Servo(can_id=CAN_ID)})


@dataclass(frozen=True)
class PendulumDescription:
    """One parameter set for the pendulum, in the board's output frame (rad, N.m).

    The dynamics every simulated backend shares: ``I q'' = tau - G sin(q - bottom) - b q' - friction``, where tau is the board's
    position loop ``kp (target - q) + kd (target_v - q') + ff`` capped at `cap_nm`. `twin` and `pid` are the bam_rack_twin and
    moteus_servo objects the MuJoCo backend runs (None in an interpreter without bam_rack_twin: the analytical and kinematic
    backends need only the numbers)."""

    label: str
    gravity_nm: float
    bottom_rad: float
    inertia_kg_m2: float
    damping_nm_s_rad: float
    coulomb_nm: float
    kp_nm_rad: float
    kd_nm_s_rad: float
    cap_nm: float = 3.0  # examples/pendulum STATIONS[11]: the tape cap the demo runs CAN 11 at
    source: str = ""
    twin: Any = field(default=None, compare=False, repr=False)
    pid: Any = field(default=None, compare=False, repr=False)

    @property
    def parameters_digest(self) -> str:
        """sha256[:12] of the numbers: which parameter set a backend runs (reported, never refused on)."""
        numbers = {k: v for k, v in dataclasses.asdict(self).items() if k not in ("twin", "pid", "source", "label")}
        numbers["twin"] = None if self.twin is None else self.twin.digest()
        return hashlib.sha256(json.dumps(numbers, sort_keys=True).encode()).hexdigest()[:12]

    def numbers(self) -> dict[str, float | str]:
        """The plain numbers (what a cloud core without bam_rack_twin rebuilds this description from)."""
        return {k: v for k, v in dataclasses.asdict(self).items() if k not in ("twin", "pid")}


def robot_fingerprint(table: JointTable = TABLE, robot: str = ROBOT) -> str:
    """The robot's own fingerprint: its name and each joint with its servo address. Both ends of a remote connection compute it;
    a core and a client that disagree drive different robots (module WHY: the parameters are not in it)."""
    joints = [[name, table.key(name)] for name in table.names]
    return hashlib.sha256(json.dumps({"robot": robot, "joints": joints}).encode()).hexdigest()


def _rack_twin():
    from bam_rack_twin import schedule  # noqa: PLC0415  (bam_rack_twin is this example's runtime: run.sh)
    from bam_rack_twin.stations import LIVE_RECORD_PATH, load_stations  # noqa: PLC0415

    label, twin = schedule.record_twin(CAN_ID)
    station = next(s for s in load_stations(LIVE_RECORD_PATH) if s.can_id == CAN_ID)
    return label, twin, station.output_pid()


def _from_twin(label: str, twin, pid, source: str) -> PendulumDescription:
    lugre = twin.friction_lugre
    coulomb = float(lugre.coulomb_nm) if lugre is not None else float(twin.frictionloss_nm or 0.0)
    return PendulumDescription(
        label=label,
        gravity_nm=float(twin.gravity_nm),
        bottom_rad=float(twin.gravity_bottom_rad),
        inertia_kg_m2=float(twin.armature_kg_m2),
        damping_nm_s_rad=float(twin.damping_nm_s_rad or 0.0),
        coulomb_nm=coulomb,
        # moteus gains are N.m per output rev (and per rev/s); the plant is in rad
        kp_nm_rad=float(pid.kp) / (2.0 * math.pi),
        kd_nm_s_rad=float(pid.kd) / (2.0 * math.pi),
        source=source,
        twin=twin,
        pid=pid,
    )


@lru_cache(maxsize=1)
def twin_of_record() -> PendulumDescription:
    """bam_rack_twin's twin of record for CAN 11, as its acceptance gates grade it: the realistic parameter set."""
    label, twin, pid = _rack_twin()
    return _from_twin(f"twin of record ({twin.version})", twin, pid,
                      f"bam_rack_twin.schedule.record_twin(11): {label}; board PID from the live deployment record")


@lru_cache(maxsize=1)
def nominal() -> PendulumDescription:
    """The same twin with every loss and board non-ideality off: frictionless, undamped, an ideal board (module WHY)."""
    _, twin, pid = _rack_twin()
    ideal = dataclasses.replace(
        twin, version=f"nominal-of-{twin.version}", frictionloss_nm=0.0, damping_nm_s_rad=0.0, friction_lugre=None,
        friction_solref=None, friction_solimp=None, command_delay_s=None, current_loop_hz=None, current_limit=None,
        torque_scale=None)
    return _from_twin("nominal, frictionless", ideal, pid,
                      "twin of record with friction, damping, latch delay, current loop and current limit off")


def from_numbers(numbers: dict[str, Any]) -> PendulumDescription:
    """A description from `numbers()` (no twin object: analytical and kinematic only)."""
    return PendulumDescription(**{k: v for k, v in numbers.items() if k in {f.name for f in dataclasses.fields(PendulumDescription)}})
