"""Confirm the fallback gate's differences (kpi/fallback.py, CONTRACT §15 A49) on one cloud VM: rerun only the tests whose
outcome differed between the two arms, ROUNDS times each way in one venv, alternating without and with like_keras.

    python fallback_confirm.py --out OUT --payload PAYLOAD [--rounds 3]      # from the package folder, inside its venv

PAYLOAD/confirm.txt lists the differing tests as the gate names them (junit `classname::name`); `kpi/fallback.py --payload
DIR --runs ...` writes it. WHY a rerun: two runs of the same bam_ws commit without like_keras, on two sets of VMs, differed on
2 of 7,795 tests (timing tests: a 20 ms frame gap, a 300 ms GIL hold; 2026-11-25), so one pair of runs cannot tell noise
from like_keras. The gate counts a difference as like_keras's only when neither arm, rerun, ever shows the other arm's outcome.
Writes OUT/fallback_confirm.json, OUT/confirm_<round>_<arm>.xml and OUT/confirm.txt (pytest's output).
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import subprocess
import sys

from fallback_shard import PROBE, run


def node_id(key: str) -> str:
    """junit `classname::name` -> pytest node id (`tests/test_x.py::Class::test_y[case]`), from this package folder."""
    classname, _, name = key.partition("::")
    if not classname:  # a collection error names the module itself
        return name.replace(".", "/") + ".py"
    parts = classname.split(".")
    for cut in range(len(parts), 0, -1):
        path = pathlib.Path(*parts[:cut]).with_suffix(".py")
        if path.is_file():
            return "::".join([str(path), *parts[cut:], name])
    return key


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=pathlib.Path, required=True)
    parser.add_argument("--payload", type=pathlib.Path, required=True)
    parser.add_argument("--rounds", type=int, default=3)
    arguments = parser.parse_args(argv)
    out = arguments.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    tests = [line for line in (arguments.payload / "confirm.txt").read_text().splitlines() if line]
    uv = os.environ.get("UV") or shutil.which("uv") or "uv"
    freeze = [uv, "pip", "freeze", "--python", sys.executable]
    lk = str((arguments.payload / "like_keras").resolve())
    record = {"schema": "like_keras/fallback-confirm-1", "bam_ws_commit": run(["git", "rev-parse", "HEAD"]),
              "package": str(pathlib.Path.cwd().relative_to(run(["git", "rev-parse", "--show-toplevel"]))),
              "payload": json.loads((arguments.payload / "payload.json").read_text()), "tests": tests,
              "nodes": [node_id(test) for test in tests], "freeze_before": run(freeze).splitlines(), "phases": []}
    with open(out / "confirm.txt", "w") as log:
        for index in range(arguments.rounds):
            for arm in ("without", "with"):
                installed = json.loads(run([sys.executable, "-c", PROBE]))["version"] is not None
                if arm == "with" and not installed:
                    subprocess.run([uv, "pip", "install", "--python", sys.executable, "--no-deps", "--no-sources", lk],
                                   capture_output=True)
                if arm == "without" and installed:
                    subprocess.run([uv, "pip", "uninstall", "--python", sys.executable, "like_keras"], capture_output=True)
                junit = f"confirm_{index}_{arm}.xml"
                log.write(f"=== round {index} {arm}\n")
                log.flush()
                code = subprocess.run([sys.executable, "-m", "pytest", "-q", "-rfEs", "-p", "no:cacheprovider",
                                       "--junitxml", str(out / junit), *record["nodes"]], stdout=log,
                                      stderr=subprocess.STDOUT, timeout=3600).returncode
                record["phases"].append({"round": index, "arm": arm, "junit": junit, "pytest_exit": code,
                                         "freeze": run(freeze).splitlines(),
                                         "like_keras": json.loads(run([sys.executable, "-c", PROBE]))})
    (out / "fallback_confirm.json").write_text(json.dumps(record, indent=1) + "\n")
    print(json.dumps({"tests": len(tests), "phases": [(p["round"], p["arm"], p["pytest_exit"]) for p in record["phases"]]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
