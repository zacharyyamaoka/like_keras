"""The lean-core gate (CONTRACT §2.1, §11): at most one runtime dependency (robot_msgs), and `import like_keras`
loads no robot_msgs, numpy, dora, pyarrow or yaml. JSON on stdout; exit 0 when the gate is green.

    python3 /home/bam/like_keras/kpi/deps.py

The import time is reported here and scored by criterion 2 (kpi/bench_overhead.py measures it cold, 10 runs).
"""

from __future__ import annotations

import json
import os
import pathlib
import re
import subprocess
import sys
import tomllib

REPO = pathlib.Path(__file__).resolve().parents[1]
ALLOWED = {"robot-msgs", "robot_msgs"}
FORBIDDEN = {"robot_msgs", "numpy", "dora", "pyarrow", "yaml"}

PROBE = """
import json, sys, time
started = time.perf_counter()
import like_keras
elapsed = time.perf_counter() - started
print(json.dumps({"modules": sorted({name.split('.')[0] for name in sys.modules}), "import_s": elapsed,
                  "file": getattr(like_keras, "__file__", None)}))
"""


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--src", default=str(REPO / "src"), help="where like_keras is (the ruler's mutation checks)")
    parser.add_argument("--pyproject", default=str(REPO / "pyproject.toml"))
    arguments = parser.parse_args(argv)
    report: dict = {"pass": False}
    with open(arguments.pyproject, "rb") as file:
        project = tomllib.load(file).get("project", {})
    dependencies = [re.split(r"[<>=!~;\[ ]", spec, maxsplit=1)[0].strip() for spec in project.get("dependencies", [])]
    report["dependencies"] = dependencies
    extra = [name for name in dependencies if name not in ALLOWED]
    env = dict(os.environ, PYTHONPATH=arguments.src)
    probe = subprocess.run([sys.executable, "-c", PROBE], cwd="/", env=env, capture_output=True,
                           text=True, timeout=60)
    if probe.returncode != 0:
        report["reason"] = "import like_keras failed: " + probe.stderr.strip().splitlines()[-1] if probe.stderr.strip() else "import failed"
        print(json.dumps(report))
        return 1
    loaded = json.loads(probe.stdout)
    report["loaded_forbidden"] = sorted(FORBIDDEN & set(loaded["modules"]))
    report["import_ms"] = round(loaded["import_s"] * 1e3, 3)
    report["from"] = loaded["file"]
    report["pass"] = len(dependencies) <= 1 and not extra and not report["loaded_forbidden"]
    if not report["pass"]:
        report["reason"] = f"extra dependencies {extra}" if extra else f"import loaded {report['loaded_forbidden']}"
    print(json.dumps(report))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
