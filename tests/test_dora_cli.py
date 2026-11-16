"""Lane dora's own tests of how like_keras runs the dora CLI (CONTRACT §9.2 skew check), beside kpi/churn's T11.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_dora_cli.py

WHY (M3 fix round 1): dora 0.5.0 builds its version string each time it parses its arguments
(binaries/cli/src/lib.rs, `#[clap(version = get_version_info())]`), so every call (`--version`, `list`, `start` and
`stop` alike) first runs `uv pip show dora-rs` and then `pip show dora-rs` off PATH. From a directory without a
.venv that is pip's start-up: 0.4 s idle, 0.8-2.6 s on the powered rig, where the 1.8 s skew check refused healthy
starts (27 of 50 at load 27.8). The fakes here run the same two helpers, and the helpers sleep 3 s.
"""

from __future__ import annotations

import os
import pathlib
import time

import pytest

import like_keras as lk
from kpi.conformance.parts import Counter
from like_keras.backends import dora

# WHY: dora 0.5.0's own order, uv then pip, before it answers anything (binaries/cli/src/lib.rs:62-88)
LIKE_DORA_0_5_0 = """#!/bin/sh
uv pip show dora-rs >/dev/null 2>&1 || pip show dora-rs >/dev/null 2>&1
echo "$@" >> "$(dirname "$0")/calls"
case "$1" in
  --version) echo "dora-cli 0.5.0" ;;
  list) echo "UUID Name Status" ;;
  *) echo "fake dora: no coordinator here"; exit 1 ;;
esac
"""
SLOW_HELPER = "#!/bin/sh\nsleep 3\nexit 1\n"


def on_path(monkeypatch, tmp_path: pathlib.Path, scripts: dict[str, str]) -> pathlib.Path:
    """Write executable scripts into tmp_path/bin and put that directory first on PATH."""
    folder = tmp_path / "bin"
    folder.mkdir()
    for name, body in scripts.items():
        (folder / name).write_text(body)
        (folder / name).chmod(0o755)
    monkeypatch.setenv("PATH", f"{folder}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.chdir(tmp_path)  # no .venv up from here, so a real `uv pip show` would fail over to pip, as on the rig
    return folder


def test_a_healthy_cli_is_accepted_within_2s_when_uv_and_pip_are_slow(tmp_path, monkeypatch):
    on_path(monkeypatch, tmp_path, {"dora": LIKE_DORA_0_5_0, "uv": SLOW_HELPER, "pip": SLOW_HELPER})
    started = time.monotonic()
    dora.check_version()  # raises when refused
    assert time.monotonic() - started < 2.0


def test_start_reaches_dora_start_within_2s_when_uv_and_pip_are_slow(tmp_path, monkeypatch):
    """`--version`, `list` and `start` all pay the helpers in dora 0.5.0, so all three run with them hidden."""
    folder = on_path(monkeypatch, tmp_path, {"dora": LIKE_DORA_0_5_0, "uv": SLOW_HELPER, "pip": SLOW_HELPER})
    system = lk.System("dora_cli_slow_helpers")
    system.add(Counter(), "counter")
    deployment = lk.Deployment(system, backend=dora)
    deployment.place(system.counter, hz=10)
    started = time.monotonic()
    with pytest.raises(RuntimeError, match="dora start refused") as refused:
        deployment.start()
    elapsed = time.monotonic() - started
    deployment.stop()
    assert "no coordinator here" in str(refused.value)
    assert [line.split()[0] for line in (folder / "calls").read_text().splitlines()] == ["--version", "list", "start"]
    assert elapsed < 2.0, f"start() took {elapsed:.2f} s to reach `dora start`"


def test_a_hanging_cli_is_still_refused_as_a_hang_within_2s(tmp_path, monkeypatch):
    """The CLI keeps the rest of PATH: a dora that really hangs (`sleep` found) is refused for not answering, not
    for an unreadable version (kpi/churn T11 accepts either, since both name 0.5.0)."""
    on_path(monkeypatch, tmp_path, {"dora": "#!/bin/sh\nsleep 30\n"})
    started = time.monotonic()
    with pytest.raises(RuntimeError, match=r"did not answer within 2\.0 s") as refused:
        dora.check_version()
    assert time.monotonic() - started < 2.0
    assert dora.VERSION in str(refused.value)
