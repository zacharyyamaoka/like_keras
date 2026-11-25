"""One shard of the fallback gate's differential run (kpi/fallback.py, CONTRACT §15 A49), on a cloud VM: run this shard's
slice of bam_rack_twin's tests in bam_rack_twin's own venv, as one arm of the comparison. JSON record on stdout.

    python fallback_shard.py --arm without|with --out OUT --payload PAYLOAD      # from the package folder, inside its venv

`--arm without`: like_keras must not be importable. `--arm with`: PAYLOAD's like_keras (the scored commit's `src/` and
`pyproject.toml`, from `kpi/fallback.py --payload`) is first installed into the same venv, with no dependencies and no
sources, so nothing else in the venv changes; `uv pip freeze` before and after shows it. Test files are split across
BAM_SHARDS workers (shard BAM_SHARD) by size, largest first, as bam_ws's tools/cloud/jobs/pytest_shard.py splits them.
Writes OUT/fallback_arm.json (what the gate reads), OUT/junit.xml and OUT/pytest.txt. Exits 0 once the record is written,
whatever the tests did: the gate, not this script, judges.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time

PROBE = """import importlib.metadata as m, importlib.util as u, json
spec = u.find_spec("like_keras")
try:
    dist = m.distribution("like_keras")
    version, direct_url = dist.version, dist.read_text("direct_url.json")
except m.PackageNotFoundError:
    version = direct_url = None
print(json.dumps({"importable": spec is not None, "origin": spec and spec.origin, "version": version,
                  "direct_url": direct_url and json.loads(direct_url)}))"""


def slice_files(files: list[pathlib.Path], shard: int, shards: int) -> list[pathlib.Path]:
    loads, mine = [0] * shards, []
    for path in sorted(files, key=lambda f: (-f.stat().st_size, f.name)):
        target = min(range(shards), key=lambda index: (loads[index], index))
        loads[target] += path.stat().st_size
        mine += [path] if target == shard else []
    return sorted(mine)


def run(command: list[str], **kwargs) -> str:
    return subprocess.run(command, capture_output=True, text=True, check=True, **kwargs).stdout.strip()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--arm", choices=["without", "with"], required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    parser.add_argument("--payload", type=pathlib.Path, required=True)
    parser.add_argument("--tests", type=pathlib.Path, default=pathlib.Path("tests"))
    # WHY 2.5 h: the slowest of 8 shards took 36 min (2026-11-25); a dispatch worker gives up at 3 h, so a stuck shard
    # still writes its record ("timed out", which the gate reads as red) before the worker does
    parser.add_argument("--timeout", type=float, default=9000.0)
    arguments = parser.parse_args(argv)
    out = arguments.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((arguments.payload / "fallback_manifest.json").read_text())
    shard, shards = int(os.environ.get("BAM_SHARD", "0")), int(os.environ.get("BAM_SHARDS", "1"))
    files = sorted(arguments.tests.glob("test_*.py"))
    mine = slice_files(files, shard, shards) if shards > 1 else files
    uv = os.environ.get("UV") or shutil.which("uv") or "uv"
    freeze = [uv, "pip", "freeze", "--python", sys.executable]
    record = {
        "schema": "like_keras/fallback-arm-1", "arm": arguments.arm, "shard": shard, "shards": shards,
        "bam_ws_commit": run(["git", "rev-parse", "HEAD"]),
        "package": str(pathlib.Path.cwd().relative_to(run(["git", "rev-parse", "--show-toplevel"]))),
        "fallback_files": {name: hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest() if pathlib.Path(name).exists()
                           else None for name in manifest["files"]},
        "all_files": [path.name for path in files], "files": [path.name for path in mine],
        "payload": json.loads((arguments.payload / "payload.json").read_text()),
        "freeze_before": run(freeze).splitlines(),
    }
    if arguments.arm == "with":
        # WHY --no-deps --no-sources: the gate measures like_keras itself; its one dependency (robot_msgs) is already in
        # this venv from bam_ws's own sibling, and like_keras's path source (../robot_msgs) does not exist here
        record["install"] = subprocess.run([uv, "pip", "install", "--python", sys.executable, "--no-deps", "--no-sources",
                                            str((arguments.payload / "like_keras").resolve())],
                                           capture_output=True, text=True).stderr.strip().splitlines()[-4:]
    record["freeze"] = run(freeze).splitlines()
    record["like_keras"] = json.loads(run([sys.executable, "-c", PROBE]))
    started = time.monotonic()
    command = [sys.executable, "-m", "pytest", "-q", "-rfEs", "-p", "no:cacheprovider", "--continue-on-collection-errors",
               "--junitxml", str(out / "junit.xml"), *map(str, mine)]
    with open(out / "pytest.txt", "w") as log:
        try:
            record["pytest_exit"] = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                                   timeout=arguments.timeout).returncode if mine else 5
            record["timed_out"] = False
        except subprocess.TimeoutExpired:
            record["pytest_exit"], record["timed_out"] = None, True
    record["wall_s"] = round(time.monotonic() - started, 1)
    (out / "fallback_arm.json").write_text(json.dumps(record, indent=1) + "\n")
    print(json.dumps({key: record[key] for key in ("arm", "shard", "shards", "bam_ws_commit", "pytest_exit", "timed_out",
                                                   "wall_s", "like_keras")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
