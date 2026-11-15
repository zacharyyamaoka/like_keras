"""Write the golden LKF bytes (CONTRACT §7.2, T10): one fixed instance per v0 codec type.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/golden/make_golden.py --check

Encoded by kpi/golden/lkf.py (numpy + struct, independent of like_keras/codec.py). `--check` regenerates in memory
and fails if a committed file would change; without it the files are (re)written. Each instance carries a NaN with
payload 0x7ff8000000000001 and a non-ASCII name where its type has a float array and names.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import lkf  # noqa: E402  (the ruler's own codec, beside this file)

NAN_PAYLOAD_BITS = 0x7FF8000000000001
NAN = np.array([NAN_PAYLOAD_BITS], dtype="<u8").view("<f8")[0]
NAMES = ("j1", "épaule_2", "肘3")  # WHY: a 2-byte and a 3-byte UTF-8 name, so offsets count bytes, not characters


def header(stamp: float, frame_id: str = "base", meta: dict[str, str] | None = None) -> dict:
    return {"stamp": stamp, "frame_id": frame_id, "meta": dict(meta or {})}


# The instances, as plain dicts keyed by field name (the tests build the robot_msgs objects from the same values).
INSTANCES: dict[str, dict] = {
    "std/Header": header(1763193600.25, "wörld", {"origin": "kpi", "unité": "rad"}),
    "sensor/JointState": {
        "name": NAMES,
        "position": np.array([0.1, NAN, -2.5]),
        "velocity": np.array([1.0, 2.0, 3.0]),
        "effort": None,
        "header": header(1763193600.5),
    },
    "control/ServoState": {
        "name": NAMES,
        "position": np.array([0.25, NAN, -1.0]),
        "velocity": np.array([0.0, -0.5, 0.125]),
        "effort": None,
        "temperature": np.array([30.5, 31.0, 29.75]),
        "voltage": None,
        "fault": np.array([0, 33, 0], dtype=np.int64),
        "seq": 42,
        "header": header(1763193600.5),
    },
    "control/ServoCommand": {
        "name": NAMES,
        "position": np.array([0.25, NAN, -1.0]),  # NaN = no position target (moteus), so its bits matter
        "velocity": np.array([0.0, 0.0, 0.0]),
        "feedforward_effort": np.array([0.1, -0.2, 0.0]),
        "stiffness": None,
        "damping": None,
        "max_effort": np.array([2.0, 2.0, 1.5]),
        "state_stamp": 1763193600.49,
        "state_seq": 41,
        "header": header(1763193600.5),
    },
    "trajectory/JointTrajectory": {
        "t": np.array([0.0, 0.5, 1.0]),
        "channels": {},
        "header": header(1763193601.0, "base", {"planner": "kpi"}),
        "joint_names": NAMES,
        "positions": np.array([[0.0, NAN, 1.0], [0.5, 0.5, 0.5], [1.0, 2.0, 3.0]]),
        "velocities": np.array([[0.0, 0.0, 0.0], [1.0, -1.0, 0.5], [0.0, 0.0, 0.0]]),
        "accelerations": None,
        "effort": None,
        "limits": {
            "joint_names": NAMES,
            "min_position": np.array([-3.0, -3.0, -3.0]),
            "max_position": np.array([3.0, 3.0, 3.0]),
            "max_velocity": None,
            "max_acceleration": None,
            "max_jerk": None,
            "max_effort": np.array([2.0, np.inf, 1.5]),
            "position_kind": ("physical", None, "software"),
        },
    },
    "std/LogRecord": {
        "header": header(1763193602.125, ""),
        "path": "pick_cell.arm.jtc",
        "tick": 123456789012,
        "level": 30,
        "message": "früh: no fresh arm state ✓",
    },
    "std/DiagnosticRecord": {
        "header": header(1763193603.0, ""),
        "path": "pick_cell.planner",
        "level": "warn",
        "code": "InputStale",
        "message": "state older than 0.1 s — stale",
        "mode": "operational",
        "expected_period_s": 1.0,
        "uptime_s": 12.5,
        "stale_inputs": ("state",),
    },
    "std/Metrics": {
        "header": header(1763193604.0, ""),
        "path": "pick_cell.arm",
        "names": ("steps", "step_p99_s", "input_age_s.état"),
        "values": np.array([12.0, NAN, 0.0001]),
    },
}


def bench_trajectory() -> dict:
    """Not a golden: a 7-joint x 100-sample plan for kpi/alloc_probe.cpp (pushed into the C++ chain every few ticks)."""
    t = np.linspace(0.0, 0.4, 100)
    positions = np.outer(np.sin(np.linspace(0.0, np.pi, 100)), np.linspace(-0.5, 0.5, 7))
    return {"t": t, "channels": {}, "header": header(0.0, "", {}), "joint_names": tuple(f"j{i + 1}" for i in range(7)),
            "positions": positions, "velocities": np.gradient(positions, t, axis=0), "accelerations": None,
            "effort": None, "limits": None}


def file_name(type_name: str) -> str:
    """`control/ServoState` -> `control.ServoState.lkf`."""
    return type_name.replace("/", ".") + ".lkf"


def golden_path(type_name: str) -> pathlib.Path:
    return HERE / file_name(type_name)


def hashes() -> dict[str, dict[str, str]]:
    return {name: {"hash": f"{lkf.schema_hash(name):016x}", "preimage": lkf.schema_preimage(name)}
            for name in sorted(lkf.SCHEMAS)}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if a committed golden file would change")
    arguments = parser.parse_args(argv)
    outputs = {golden_path(name): lkf.encode(name, message) for name, message in INSTANCES.items()}
    outputs[HERE / "schema_hashes.json"] = (json.dumps(hashes(), indent=1, ensure_ascii=False) + "\n").encode("utf-8")
    bench = bench_trajectory()
    outputs[HERE / "bench" / "JointTrajectory_7x100.lkf"] = lkf.encode("trajectory/JointTrajectory", bench)
    outputs[HERE / "bench" / "JointTrajectory_7x100.nodes"] = "".join(
        f"{length} {buffers} {children}\n" for length, buffers, children in lkf.nodes("trajectory/JointTrajectory", bench)).encode()
    stale = [path.name for path, data in outputs.items() if not path.exists() or path.read_bytes() != data]
    if arguments.check:
        print(json.dumps({"golden": len(INSTANCES), "stale": stale}))
        return 1 if stale else 0
    for path, data in outputs.items():
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(data)
    print(json.dumps({"written": sorted(path.name for path in outputs)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
