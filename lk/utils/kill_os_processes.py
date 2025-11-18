#!/usr/bin/env python3

"""
    Process discovery and force-kill helpers.
"""

# PYTHON
import os
import signal
import subprocess
from typing import Iterable


def find_os_processes(name: str) -> list[tuple[str, str]]:
    """Return (pid, command) tuples for processes whose command starts with name."""
    ps_output = subprocess.check_output(["ps", "aux"], text=True)
    matches: list[tuple[str, str]] = []
    for line in ps_output.splitlines():
        if name not in line:
            continue
        columns = line.split()
        if len(columns) < 11:
            continue
        pid = columns[1]
        command = " ".join(columns[10:])
        if command.startswith(name):
            matches.append((pid, command))
    return matches


def kill_process(pid: str) -> None:
    """Send SIGKILL to the provided PID."""
    os.kill(int(pid), signal.SIGKILL)


def kill_os_processes(name: str, on_kill: Iterable[str] | None = None) -> None:
    """Kill all running processes whose command matches."""
    processes = find_os_processes(name)
    if not processes:
        print(f"No processes found starting with {name}")
        return

    for pid, command in processes:
        try:
            kill_process(pid)
            print(f"Killed {pid} ({command})")
            if on_kill:
                for cmd in on_kill:
                    subprocess.run(cmd, shell=True, check=False)
        except Exception as exc:
            print(f"Failed to kill {pid}: {exc}")


__all__ = ["find_os_processes", "kill_process", "kill_os_processes"]

