"""Child processes for the kpi suites: start a module, wait for its `{"ready": true}` line, find dora nodes, stop by PID.

Everything a suite starts it stops by the PID it recorded (never by name or port: a peer's process may share both).
Under kpi/run.py the suite runs in its own PID namespace, so a leftover dies with it anyway.
"""

from __future__ import annotations

import json
import os
import pathlib
import select
import signal
import subprocess
import time

from kpi.support.lk import REPO, python, subprocess_env


class Child:
    """A Python module run as `python -m <module> <args>`, its stdout read line by line."""

    def __init__(self, module: str, *args: str, log: pathlib.Path | None = None, **env: str) -> None:
        self.module = module
        self.log = log
        self.lines: list[str] = []
        self.process = subprocess.Popen([python(), "-m", module, *args], cwd=REPO, env=subprocess_env(**env),
                                        stdout=subprocess.PIPE, stderr=open(log, "ab") if log else subprocess.DEVNULL,
                                        text=True, start_new_session=True)

    @property
    def pid(self) -> int:
        return self.process.pid

    def wait_ready(self, timeout_s: float = 20.0) -> dict:
        """The first stdout line that is a JSON object with "ready": true (else raise naming the exit status)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(f"{self.module} exited {self.process.returncode} before it was ready")
            readable, _, _ = select.select([self.process.stdout], [], [], 0.1)
            if not readable:
                continue
            line = self.process.stdout.readline()
            self.lines.append(line)
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if isinstance(message, dict) and message.get("ready"):
                return message
        raise TimeoutError(f"{self.module} not ready in {timeout_s} s")

    def signal(self, number: int) -> None:
        if self.process.poll() is None:
            os.kill(self.process.pid, number)

    def stop(self, timeout_s: float = 10.0) -> int:
        if self.process.poll() is None:
            self.signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                self.signal(signal.SIGKILL)
                self.process.wait(timeout=timeout_s)
        return self.process.returncode


def pids_with(*needles: str) -> list[int]:
    """PIDs whose command line holds every needle (scans /proc of this PID namespace, never the filesystem)."""
    found = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            command = open(f"/proc/{entry.name}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if all(needle in command for needle in needles) and int(entry.name) != os.getpid():
            found.append(int(entry.name))
    return found


def wait_for(condition, timeout_s: float, what: str, poll_s: float = 0.02):
    """Poll `condition()` until it returns a truthy value; raise naming `what` after `timeout_s`."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = condition()
        if value:
            return value
        time.sleep(poll_s)
    raise TimeoutError(f"{what}: not within {timeout_s} s")
