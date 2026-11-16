"""Run one kpi suite the way the scoreboard does: pytest in a private network and PID namespace.

    nice -n 19 ionice -c3 env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/run.py conformance [pytest args]

WHY the namespaces: the dora suites start their own dora 0.5.0 coordinator and daemon, which bind 127.0.0.1:6012,
53290 and 53291, the ports of the production dora that runs the OBSBOT and netbelt dataflows. Under
`bwrap --unshare-net` they get a fresh loopback; `--unshare-pid` makes sure nothing a suite starts outlives it.
WHY a private XDG_RUNTIME_DIR per run (ruler M0.4): like_keras serves a deployment's attach socket at
$XDG_RUNTIME_DIR/like_keras/<system>.sock (else /tmp/like_keras; CONTRACT §9.3), its one per-user socket directory.
A unix socket path is a file, so a network namespace does not separate it: a peer's live deployment named pick_cell
on the host's /run/user/1000 failed contract test_swap_variant[backend-b,host-a] ("pick_cell.sock is served by
another running deployment"; library verifier, round 1). Each run gets a fresh directory under build/kpi/xdg/, its
children and their dora nodes inherit it, and it is removed when the run ends.
Writes build/kpi/junit/<suite>.xml, which kpi/score.py reads.
"""

from __future__ import annotations

import os
import pathlib
import shutil
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parents[1]
JUNIT = REPO / "build" / "kpi" / "junit"
RUNTIME_DIRS = REPO / "build" / "kpi" / "xdg"  # WHY short: a unix socket path holds at most 107 bytes


def namespaced(argv: list[str]) -> list[str]:
    """argv in a private network and PID namespace: the line every suite runs under, and kpi/bench_overhead.py
    too (kpi/score.py), so its private dora may start (ruler-001 §1)."""
    bwrap = shutil.which("bwrap")
    if bwrap is None:
        return argv
    return [bwrap, "--dev-bind", "/", "/", "--unshare-net", "--unshare-pid", "--proc", "/proc", "--die-with-parent",
            "--chdir", str(REPO), "--", *argv]


def command(suite: str, extra: list[str]) -> list[str]:
    JUNIT.mkdir(parents=True, exist_ok=True)
    basetemp = REPO / "build" / "kpi" / "tmp" / suite.replace("/", "_")  # kpi writes nothing outside the repo
    pytest = [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", f"kpi/{suite}",
              f"--junitxml={JUNIT / (suite.replace('/', '_') + '.xml')}", f"--basetemp={basetemp}", *extra]
    return namespaced(pytest)


def environment(runtime_dir: pathlib.Path | None = None) -> dict[str, str]:
    """The environment of a namespaced kpi run: this venv first on PATH, src and the repo importable; with
    `runtime_dir`, that private directory as XDG_RUNTIME_DIR (the attach sockets' directory, CONTRACT §9.3)."""
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(pathlib.Path(sys.executable).parent), env.get("PATH", "")])
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src"), str(REPO)])
    env["TMPDIR"] = str(REPO / "build" / "kpi" / "tmp")  # kpi writes nothing outside the repo
    (REPO / "build" / "kpi" / "tmp").mkdir(parents=True, exist_ok=True)
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        env.setdefault(name, "1")
    if runtime_dir is not None:
        env["XDG_RUNTIME_DIR"] = str(runtime_dir)
    return env


def private_runtime_dir() -> pathlib.Path:
    """A fresh, empty XDG_RUNTIME_DIR for one run (mode 0700, as the XDG spec asks), under build/kpi/xdg/."""
    RUNTIME_DIRS.mkdir(parents=True, exist_ok=True)
    return pathlib.Path(tempfile.mkdtemp(dir=RUNTIME_DIRS))


def run(suite: str, extra: list[str] | None = None, timeout_s: float | None = None) -> int:
    runtime_dir = private_runtime_dir()
    try:
        return subprocess.run(command(suite, extra or []), cwd=REPO, env=environment(runtime_dir),
                              timeout=timeout_s).returncode
    except subprocess.TimeoutExpired:
        return 124
    finally:
        shutil.rmtree(runtime_dir, ignore_errors=True)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    raise SystemExit(run(sys.argv[1], sys.argv[2:]))
