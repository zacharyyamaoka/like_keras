"""The fallback-untouched gate (CONTRACT §11): arm_hold, pick_loop_v1 and fivebar_run byte for byte as at M0, and
bam_rack_twin's own tests green with like_keras absent from its dependencies. JSON on stdout; exit 0 when green.

    python3 /home/bam/like_keras/kpi/fallback.py [--suite]

Without --suite only the bytes and the dependency are checked and the gate is reported `suite: not run` (red):
the suite is the contract's own command, `uv run --directory /home/bam/bam_ws/src/dev/bam_rack_twin pytest -q`,
run here with --no-sync and no pytest cache so it writes nothing into bam_ws. If Zach changes one of the three files
on purpose, the ruler lane re-records fallback_manifest.json; a builder never does.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import tomllib

MANIFEST = pathlib.Path(__file__).resolve().with_name("fallback_manifest.json")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--suite", action="store_true", help="also run bam_rack_twin's test suite")
    parser.add_argument("--timeout", type=float, default=3600.0)
    parser.add_argument("--manifest", type=pathlib.Path, default=MANIFEST, help="(the ruler's mutation checks)")
    arguments = parser.parse_args(argv)
    manifest = json.loads(arguments.manifest.read_text())
    root = pathlib.Path(manifest["root"])
    changed = [name for name, digest in manifest["files"].items()
               if not (root / name).exists() or hashlib.sha256((root / name).read_bytes()).hexdigest() != digest]
    with open(root / "pyproject.toml", "rb") as file:
        project = tomllib.load(file).get("project", {})
    hard = [spec for spec in project.get("dependencies", []) if spec.replace("-", "_").startswith("like_keras")]
    report = {"files_changed": changed, "like_keras_required": hard, "suite": "not run"}
    if arguments.suite:
        env = {key: value for key, value in os.environ.items() if key != "VIRTUAL_ENV"}
        try:
            result = subprocess.run(["uv", "run", "--no-sync", "--directory", str(root), "pytest", "-q", "-p",
                                     "no:cacheprovider"], env=env, capture_output=True, text=True,
                                    timeout=arguments.timeout)
            report["suite"] = "passed" if result.returncode == 0 else "failed"
            report["suite_tail"] = result.stdout.strip().splitlines()[-1:] if result.stdout.strip() else []
        except subprocess.TimeoutExpired:
            report["suite"] = "timed out"
    report["pass"] = not changed and not hard and report["suite"] == "passed"
    print(json.dumps(report))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
