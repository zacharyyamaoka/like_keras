"""What every kpi suite shares: like_keras or a clean skip, the built .so files, decoded recordings, the CPUs to use.

A suite on an empty repo SKIPS (never errors at collection), and the scoreboard counts a skip as not passed, so the
score of an empty repo is 0 and a builder cannot raise it by making a test skip (kpi/score.py's denominators are
frozen lists of test ids).
"""

from __future__ import annotations

import functools
import importlib
import os
import pathlib
import sys
from typing import Any

import pytest

from kpi.golden import lkf
from kpi.support import mcap

REPO = pathlib.Path(__file__).resolve().parents[2]
BUILD = REPO / "build" / "kpi"
KPI = REPO / "kpi"

TIMING = os.environ.get("KPI_TIER", "") == "timing"


def like_keras():
    """`import like_keras`, or skip the test: the M0 ruler runs before the core lane lands."""
    try:
        return importlib.import_module("like_keras")
    except ImportError as error:
        pytest.skip(f"like_keras is not importable yet ({error})")


def dora_backend():
    like_keras()
    try:
        return importlib.import_module("like_keras.backends.dora")
    except ImportError as error:
        pytest.skip(f"like_keras.backends.dora is not importable yet ({error})")


def inproc_backend():
    like_keras()
    try:
        return importlib.import_module("like_keras.backends.inproc")
    except ImportError as error:
        pytest.skip(f"like_keras.backends.inproc is not importable yet ({error})")


def backend(name: str):
    return dora_backend() if name == "dora" else inproc_backend()


def native_library(name: str) -> pathlib.Path:
    """build/kpi/lib<name>.so (cmake -S /home/bam/like_keras -B build), or skip."""
    path = BUILD / f"lib{name}.so"
    if not path.exists():
        pytest.skip(f"{path} is not built yet (cmake --build /home/bam/like_keras/build)")
    return path


def binary(name: str) -> pathlib.Path:
    path = BUILD / name
    if not path.exists():
        pytest.skip(f"{path} is not built yet")
    return path


def robot_msgs_type(type_name: str):
    """The robot_msgs class registered as `type_name` (e.g. std/LogRecord), or skip until robot_msgs has it."""
    import robot_msgs  # noqa: F401 - registers every type

    domain = importlib.import_module(f"robot_msgs.{type_name.split('/')[0]}")
    registry = robot_msgs.message_types()
    if type_name not in registry:
        pytest.skip(f"robot_msgs has no {type_name} yet (the robot_msgs lane, CONTRACT §2.3)")
    del domain
    return registry[type_name]


def kpi_cpus() -> list[int]:
    """The CPUs a test deployment may pin to: this process's own affinity, never core 22 (RESERVED_CPUS)."""
    allowed = sorted(cpu for cpu in os.sched_getaffinity(0) if cpu != 22)
    return [allowed[-1]] if allowed else [0]


def timing_tier() -> None:
    """Timing assertions run only in the timing tier (rack unpowered or the fixed cloud VM; CONTRACT §10)."""
    if not TIMING:
        pytest.skip("timing assertion: run with KPI_TIER=timing on an unpowered rack or the fixed cloud VM")


# ---------------------------------------------------------------- recordings

TYPE_BY_HASH = {lkf.schema_hash(name): name for name in lkf.SCHEMAS}


def decode(frame: bytes) -> tuple[str, dict[str, Any] | None]:
    """(type name, message dict) of one LKF frame, the type found by its schema hash."""
    hash_value, _, _ = lkf.split(frame)
    if hash_value not in TYPE_BY_HASH:
        raise ValueError(f"LKF frame with unknown schema hash {hash_value:016x}")
    type_name = TYPE_BY_HASH[hash_value]
    return type_name, lkf.decode(type_name, frame)


def recorded(path, suffix: str) -> list[dict[str, Any]]:
    """Every message recorded on the channel ending in `suffix`, decoded, in file order, with its MCAP times."""
    recording = mcap.read(path)
    out = []
    for message in recording.on(suffix):
        _, body = decode(message.data)
        if body is not None:
            body = dict(body)
            body["_log_time"], body["_publish_time"] = message.log_time, message.publish_time
        out.append(body)
    return out


@functools.cache
def python() -> str:
    return sys.executable


def lk_command(*args: str) -> list[str]:
    """The `lk` CLI (CONTRACT §9.3): the script on PATH, else `python -m like_keras.cli`."""
    import shutil

    script = shutil.which("lk")
    return [script, *args] if script else [python(), "-m", "like_keras.cli", *args]


def subprocess_env(**extra: str) -> dict[str, str]:
    """The environment for a child Python: the repo and src on the path, the lane's no-device flag kept."""
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src"), str(REPO), env.get("PYTHONPATH", "")]).rstrip(os.pathsep)
    env["TMPDIR"] = str(REPO / "build" / "kpi" / "tmp")  # kpi writes nothing outside the repo
    (REPO / "build" / "kpi" / "tmp").mkdir(parents=True, exist_ok=True)
    env.setdefault("OMP_NUM_THREADS", "1")
    env.setdefault("OPENBLAS_NUM_THREADS", "1")
    env.setdefault("MKL_NUM_THREADS", "1")
    env.update(extra)
    return env
