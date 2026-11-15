"""Run one kpi suite the way the scoreboard does: pytest in a private network and PID namespace.

    nice -n 19 ionice -c3 env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/run.py conformance [pytest args]

WHY the namespaces: the dora suites start their own dora 0.5.0 coordinator and daemon, which bind 127.0.0.1:6012,
53290 and 53291, the ports of the production dora that runs the OBSBOT and netbelt dataflows. Under
`bwrap --unshare-net` they get a fresh loopback; `--unshare-pid` makes sure nothing a suite starts outlives it.
Writes build/kpi/junit/<suite>.xml, which kpi/score.py reads.
"""

from __future__ import annotations

import os
import pathlib
import shutil
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
JUNIT = REPO / "build" / "kpi" / "junit"


def command(suite: str, extra: list[str]) -> list[str]:
    JUNIT.mkdir(parents=True, exist_ok=True)
    basetemp = REPO / "build" / "kpi" / "tmp" / suite.replace("/", "_")  # kpi writes nothing outside the repo
    pytest = [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", f"kpi/{suite}",
              f"--junitxml={JUNIT / (suite.replace('/', '_') + '.xml')}", f"--basetemp={basetemp}", *extra]
    bwrap = shutil.which("bwrap")
    if bwrap is None:
        return pytest
    return [bwrap, "--dev-bind", "/", "/", "--unshare-net", "--unshare-pid", "--proc", "/proc", "--die-with-parent",
            "--chdir", str(REPO), "--", *pytest]


def run(suite: str, extra: list[str] | None = None, timeout_s: float | None = None) -> int:
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(pathlib.Path(sys.executable).parent), env.get("PATH", "")])
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src"), str(REPO)])
    env["TMPDIR"] = str(REPO / "build" / "kpi" / "tmp")  # kpi writes nothing outside the repo
    (REPO / "build" / "kpi" / "tmp").mkdir(parents=True, exist_ok=True)
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        env.setdefault(name, "1")
    try:
        return subprocess.run(command(suite, extra or []), cwd=REPO, env=env, timeout=timeout_s).returncode
    except subprocess.TimeoutExpired:
        return 124


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    raise SystemExit(run(sys.argv[1], sys.argv[2:]))
