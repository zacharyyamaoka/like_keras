"""A private dora 0.5.0 coordinator + daemon for the dora suites, started and stopped by PID.

WHY private, and only in a private network namespace: the production coordinator and daemon (systemd units
dora-coordinator, dora-daemon-ubuntu) run the OBSBOT and netbelt dataflows on 127.0.0.1:6012/53290/53291. A kpi
dataflow must never reach them, and dora 0.5.0 has no way to move the daemon's dynamic-node port. So the suites run
under `bwrap --unshare-net` (kpi/score.py and kpi/run.py do it), where 127.0.0.1 is a fresh loopback; this fixture
REFUSES to start when anything already answers on 127.0.0.1:6012 (the shared namespace). The wrapper also unshares
the PID namespace, so nothing the suite starts can outlive it.
"""

from __future__ import annotations

import os
import pathlib
import shutil
import signal
import socket
import subprocess
import time

import pytest

from kpi.support.lk import REPO, subprocess_env

CONTROL_PORT = 6012
DAEMON_PORT = 53290
VERSION = "0.5.0"


def dora_cli() -> str:
    found = shutil.which("dora") or str(REPO / ".venv" / "bin" / "dora")
    if not os.path.exists(found):
        pytest.skip("the dora 0.5.0 CLI is not on PATH (the test extra installs dora-rs-cli==0.5.0)")
    return found


def listening(port: int) -> bool:
    with socket.socket() as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def shared_namespace() -> bool:
    return listening(CONTROL_PORT)


class PrivateDora:
    def __init__(self, workdir: pathlib.Path) -> None:
        self.workdir = workdir
        self.cli = dora_cli()
        self.coordinator: subprocess.Popen | None = None
        self.daemon: subprocess.Popen | None = None

    def start(self) -> PrivateDora:
        if shared_namespace():
            pytest.skip("refused: 127.0.0.1:6012 answers, so this is the shared network namespace with the production "
                        "dora coordinator; run the suite through kpi/run.py or kpi/score.py (bwrap --unshare-net)")
        self.workdir.mkdir(parents=True, exist_ok=True)
        env = subprocess_env()
        log = open(self.workdir / "dora-private.log", "ab")
        self.coordinator = subprocess.Popen(
            [self.cli, "coordinator", "--interface", "127.0.0.1", "--control-interface", "127.0.0.1", "--quiet"],
            cwd=self.workdir, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self._wait(lambda: listening(CONTROL_PORT) and listening(DAEMON_PORT), "the private coordinator")
        self.daemon = subprocess.Popen(
            [self.cli, "daemon", "--coordinator-addr", "127.0.0.1", "--quiet"],
            cwd=self.workdir, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self._wait(self._daemon_connected, "the private daemon")
        return self

    def _daemon_connected(self) -> bool:
        try:
            result = subprocess.run([self.cli, "check"], cwd=self.workdir, env=subprocess_env(), capture_output=True,
                                    text=True, timeout=3)
        except subprocess.TimeoutExpired:
            return False
        return result.returncode == 0 and "Daemon: Running" in result.stdout

    @staticmethod
    def _wait(ready, what: str, timeout_s: float = 60.0) -> None:  # WHY 60 s: lanes share cores 22-23 at nice 19
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if ready():
                return
            time.sleep(0.1)
        raise RuntimeError(f"{what} did not come up in {timeout_s} s")

    @property
    def daemon_pid(self) -> int:
        assert self.daemon is not None
        return self.daemon.pid

    def alive(self) -> bool:
        return all(p is not None and p.poll() is None for p in (self.coordinator, self.daemon))

    def stop(self) -> None:
        for process in (self.daemon, self.coordinator):
            if process is None or process.poll() is not None:
                continue
            os.kill(process.pid, signal.SIGCONT)  # a test may have left it stopped (T4)
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        self.daemon = self.coordinator = None


@pytest.fixture(scope="session")
def private_dora():
    # WHY a short directory under build/: the daemon fails to come up from a deep working directory (a unix
    # socket path over 108 bytes, measured 2026-11-15 from the session scratchpad), and kpi writes nothing outside
    # the repo.
    dora = PrivateDora(REPO / "build" / "kpi" / f"dora-{os.getpid()}").start()
    yield dora
    dora.stop()
