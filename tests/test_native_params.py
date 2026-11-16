"""Lane core's test of CONTRACT §15 A41 (consumer-020): a native box's describe() lists every LK_PARAMS field with its
default, overlaid by the config given, where the frozen ruler test (facilities::test_native_describe_lists_param_
defaults) does not reach: a nested aggregate given in part, and the box running with the mapping it is shown with.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_native_params.py

WHY a nested aggregate keeps its other defaults: LK_PARAMS fills a nested aggregate key by key, so the reference
controller given only {"bus": {"twin": true}} runs with the bus's other defaults and every guard default, and the
datasheet must show them (A41: "every LK_PARAMS field"). WHY run it: lk_create now gets the defaults lk_describe()
printed (NaN, a row table) back as JSON, in both hosts, and must run exactly as it did with them absent. The dora
case needs a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites).

CONTRACT §15 A44 where the frozen ruler (facilities::test_partly_given_rows_run_as_described, one flat table) does
not reach: tables inside a nested aggregate and inside a table's row, given in part (tests/cpp/row_tables.cpp); its
C ABI half, lk_create filling a partial row itself, is tests/cpp/test_friction.cpp's TableParams test.
"""

from __future__ import annotations

import ctypes
import json
import math
import pathlib
import subprocess

import pytest

import like_keras as lk
from kpi.polyglot.friction_parts import JointStateLog
from kpi.support.lk import backend, feature_library, native_library

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "tests-cpp"


def lk_describe_params(library) -> dict:
    so = ctypes.CDLL(str(library))
    so.lk_describe.restype = ctypes.c_char_p
    return json.loads(so.lk_describe())["params"]


def test_a_nested_aggregate_given_in_part_keeps_its_other_defaults():
    library = native_library("arm_controller")  # kpi/tasks/arm_cell: ArmConfig {bus, guard}, LK_PARAMS each
    defaults = lk_describe_params(library)
    assert set(defaults) == {"bus", "guard"} and defaults["bus"]["twin"] is False, defaults
    s = lk.System("nested_params")
    arm = s.add(lk.native(library, config={"bus": {"twin": True}}), "arm")
    s.expose(arm.trajectory)
    shown = lk.Deployment(s).describe()["components"]["nested_params.arm"]["params"]
    assert shown == {"bus": {**defaults["bus"], "twin": True}, "guard": defaults["guard"]}, shown
    # WHY the JSON text, not ==: a double default printed as 10 reads back as an int, so the datasheet of the box
    # differed in kind from a Python twin's 10.0 (ruler M0.7 compares datasheets as sorted JSON)
    guard = {"runaway_rad": 0.5, "max_velocity_rad_s": 10.0, "max_effort_nm": 2.0, "start_tolerance_rad": 0.0524}
    assert json.dumps(shown["guard"]) == json.dumps(guard), shown["guard"]
    assert json.dumps(shown["bus"]["joints"]) == "7", "an int field stays an int"


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
@pytest.mark.parametrize("config", [None, {"gain": 2.0}], ids=["defaults", "one_key"])
def test_the_box_runs_with_the_params_it_is_shown_with(backend_name, config, tmp_path, request):
    """described_params.cpp sends (gain, retries, timeout_s) once: what lk_create filled from the shown mapping."""
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    library = feature_library("described_params")
    name = f"shown_params_{backend_name}_{'one_key' if config else 'defaults'}"
    s = lk.System(name)
    box = s.add(lk.native(library, config=config), "box")
    s.add(JointStateLog(path=str(tmp_path / "echo.jsonl")), "sink", value=box.echo)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    shown = dep.describe()["components"][f"{name}.box"]["params"]
    assert set(shown) == set(lk_describe_params(library)) and shown["gain"] == (config or {}).get("gain", 0.5), shown
    try:
        dep.start(until=0.45)
    finally:
        dep.stop()
    echoed = [json.loads(line) for line in (tmp_path / "echo.jsonl").read_text().splitlines()]
    assert len(echoed) == 1, echoed
    gain, retries, timeout_s = echoed[0]["position"]
    assert (gain, retries) == (shown["gain"], 3.0) and (timeout_s is None or math.isnan(timeout_s)), echoed[0]


# tests/cpp/row_tables.cpp: rows given in part in a table inside a nested aggregate (arm.joints), in a table inside one
# of its rows (limits) and in a table at the top (stops); each key left out is its row type's member default (Joint:
# spare, 0.125, one limit [-2, 2]; Limit: [-1, 1]), never the default table's row at its index (shoulder 0.25 with
# two limits, elbow -0.5; the stop [0.25, 0.75])
ROW_TABLES_GIVEN = {"arm": {"joints": [{"name": "wrist"}, {"offset": 0.5, "limits": [{"low": -0.1}]}]},
                    "stops": [{"high": 0.9}, {}]}
ROW_TABLES_FILLED = {"arm": {"joints": [{"name": "wrist", "offset": 0.125, "limits": [{"low": -2.0, "high": 2.0}]},
                                        {"name": "spare", "offset": 0.5, "limits": [{"low": -0.1, "high": 1.0}]}]},
                     "stops": [{"low": -1.0, "high": 0.9}, {"low": -1.0, "high": 1.0}]}


@pytest.fixture(scope="module")
def row_tables() -> pathlib.Path:
    """tests/cpp built (tests/test_cpp_box.py builds the same project): librow_tables.so."""
    for command in (["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"],
                    ["nice", "-n", "19", "cmake", "--build", str(BUILD), "--target", "row_tables", "-j", "1"]):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    return BUILD / "librow_tables.so"


def leaves(params: dict) -> list[list]:
    """What row_tables.cpp sends for `params`: [path, value] per leaf, a string leaf as ["path=value", 0.0]."""
    def limits(table: str, rows: list) -> list[list]:
        return [leaf for at, row in enumerate(rows)
                for leaf in ([f"{table}[{at}].low", row["low"]], [f"{table}[{at}].high", row["high"]])]
    found = []
    for at, joint in enumerate(params["arm"]["joints"]):
        found += [[f"arm.joints[{at}].name={joint['name']}", 0.0], [f"arm.joints[{at}].offset", joint["offset"]]]
        found += limits(f"arm.joints[{at}].limits", joint["limits"])
    return found + limits("stops", params["stops"])


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_nested_tables_given_in_part_run_as_described(backend_name, row_tables, tmp_path, request):
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    name = f"row_tables_{backend_name}"
    s = lk.System(name)
    box = s.add(lk.native(row_tables, config=ROW_TABLES_GIVEN), "box")
    s.add(JointStateLog(path=str(tmp_path / "echo.jsonl")), "sink", value=box.echo)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    shown = dep.describe()["components"][f"{name}.box"]["params"]
    assert shown == ROW_TABLES_FILLED, f"describe() lists {shown}"
    try:
        dep.start(until=0.45)
    finally:
        dep.stop()
    echoed = [json.loads(line) for line in (tmp_path / "echo.jsonl").read_text().splitlines()]
    assert len(echoed) == 1, echoed
    ran = [list(leaf) for leaf in zip(echoed[0]["name"], echoed[0]["position"])]
    assert ran == leaves(shown), f"the box runs {ran}, describe() lists {shown}"
