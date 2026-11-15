"""Criterion 5: lines changed, across every file, to swap the reference task between process, language, backend
and host. JSON on stdout.

    python3 /home/bam/like_keras/kpi/swap_cost.py

Each swap is a pair of frozen trees, kpi/tasks/arm_cell/swaps/<swap>/{a,b}. They are formatted (ruff, when present)
in a temporary copy, then compared with `git diff --no-index --numstat`. A changed line counts once (max of added and
removed per file). `deploy.py` is the Deployment file; any other changed file is a component or System line, which
zeroes the swap. Whether each variant also runs equivalently is kpi/contract's test_swap_variant (score.py joins them).
"""

from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from surface import ruff, scratch_dir  # noqa: E402

SWAPS = pathlib.Path(__file__).resolve().parent / "tasks" / "arm_cell" / "swaps"
DEPLOYMENT_FILES = {"deploy.py"}


def numstat(left: pathlib.Path, right: pathlib.Path) -> list[tuple[int, int, str]]:
    result = subprocess.run(["git", "diff", "--no-index", "--numstat", str(left), str(right)], capture_output=True,
                            text=True)
    rows = []
    for line in result.stdout.splitlines():
        added, removed, name = line.split("\t", 2)
        rows.append((int(added), int(removed), name.split("=>")[-1].strip(" {}").rsplit("/", 1)[-1]))
    return rows


def measure(swap: pathlib.Path, tool) -> dict:
    with tempfile.TemporaryDirectory(prefix="swap-", dir=scratch_dir()) as scratch:
        sides = {}
        for side in ("a", "b"):
            copy = pathlib.Path(scratch) / side
            shutil.copytree(swap / side, copy, ignore=shutil.ignore_patterns("__pycache__"))
            if tool:
                subprocess.run([*tool, "format", "--quiet", "--line-length", "120", str(copy)], capture_output=True)
            sides[side] = copy
        rows = numstat(sides["a"], sides["b"])
    deployment = sum(max(added, removed) for added, removed, name in rows if name in DEPLOYMENT_FILES)
    component = sum(max(added, removed) for added, removed, name in rows if name not in DEPLOYMENT_FILES)
    return {"deployment_lines": deployment, "component_lines": component,
            "files": [name for _, _, name in rows]}


def main() -> int:
    tool = ruff()
    report = {"formatted": tool is not None,
              "swaps": {swap.name: measure(swap, tool) for swap in sorted(SWAPS.iterdir()) if (swap / "a").is_dir()}}
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
