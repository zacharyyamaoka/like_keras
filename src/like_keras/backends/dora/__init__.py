"""The dora backend (CONTRACT §9.2): every process a dora 0.5.0 node, the dataflow generated from the plan.

`start()` writes build/like_keras/<system>/{dataflow.yml, plan.json}, refuses a dora CLI of another version within
2 s, and runs `dora start --detach`; the launching process runs no components. Python processes run
`python -m like_keras.backends.dora.host`, a native .so runs in `lk-dora-host` (cpp/hosts), and a gateway node
serves the attach socket and the recordings when the System exposes ports or records.

This package is the one place like_keras names dora (criterion 6): the core and the components never do.
"""

from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

from like_keras.backends import address  # noqa: F401  (the attach socket describe() prints)

NAME = "dora"
VERSION = "0.5.0"  # CONTRACT §2.2: the one place the dora version is written
# processes: every process is a dora node of its own, so every component is rebuilt there as module:qualname(**params);
# no mixed_process: a native .so runs in lk-dora-host, a process of its own (CONTRACT §4.3 check 3).
CAPABILITIES = ("latest", "queue", "lagged", "host", "lockstep", "processes")
SKEW_TIMEOUT_S = 2.0  # T11: a refusal within 2 s, never a hang
DRAIN_S = 10.0  # stop(): how long the signalled nodes get, in all, to deactivate and send their last records
# WHY hidden: dora 0.5.0 builds its version string each time it parses its arguments (binaries/cli/src/lib.rs:23,
# 62-88), so every call (--version, list, start, stop) first runs `uv pip show dora-rs`, then `pip show dora-rs`,
# off PATH. Outside a .venv that is pip's start-up, 0.8-2.6 s on the powered rig, and the skew check refused healthy
# starts 27 of 50 times (M3 verifier r1). like_keras reads only the `dora-cli X` line, so the CLI runs with these two
# shadowed by `false` and the rest of PATH kept: the same line in a median 0.03-0.06 s, and a CLI that really hangs
# still hangs, so T11's 2 s refusal still covers it (tests/test_dora_cli.py).
HIDDEN_HELPERS = ("uv", "pip")


def start(plan, until: float | None):
    run = Run(plan)
    run.start(until)
    return run


def cli() -> str:
    """The dora CLI: the one on PATH, else the one beside this interpreter (the venv's)."""
    return shutil.which("dora") or os.path.join(os.path.dirname(sys.executable), "dora")


def call(*arguments: str, timeout: float, cwd: str | None = None) -> tuple[int, str]:
    """Run the dora CLI with HIDDEN_HELPERS shadowed; on timeout kill its whole process group (a child holding the
    pipe would hang us)."""
    with tempfile.TemporaryDirectory(prefix="lk-dora-cli-") as hidden:
        for helper in HIDDEN_HELPERS:
            os.symlink(shutil.which("false") or "/bin/false", os.path.join(hidden, helper))
        path = os.pathsep.join([hidden, os.environ.get("PATH", os.defpath)])
        process = subprocess.Popen(
            [cli(), *arguments],
            cwd=cwd,
            env={**os.environ, "PATH": path},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            output, _ = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()
            raise
    return process.returncode, output


def check_version() -> None:
    """`dora --version` must be VERSION (CONTRACT §9.2 skew): refused naming both versions, within 2 s."""
    try:
        _, output = call("--version", timeout=SKEW_TIMEOUT_S - 0.2)
    except (subprocess.TimeoutExpired, OSError) as error:
        raise RuntimeError(
            f"`{cli()} --version` did not answer within {SKEW_TIMEOUT_S} s ({type(error).__name__}); like_keras "
            f"needs the dora CLI {VERSION}. Fix: put dora-cli {VERSION} first on PATH"
        ) from None
    found = re.search(r"dora-cli\s+(\S+)", output)
    theirs = found.group(1) if found else output.strip()[:80] or "nothing"
    if theirs != VERSION:
        raise RuntimeError(
            f"the dora CLI on PATH ({cli()}) is {theirs}; like_keras runs dora {VERSION}. "
            f"Fix: install dora-rs-cli=={VERSION} first on PATH"
        )


class Run:
    """A started dataflow, seen from the launching process: it runs no components (CONTRACT §9.2)."""

    def __init__(self, plan) -> None:
        self.plan, self.name, self.uuid, self.directory, self.document = plan, plan.datasheet["system"], None, None, {}

    def start(self, until: float | None) -> None:
        from like_keras.backends.dora import emit

        check_version()
        self.directory, document = self.directory, self.document = emit.write(self.plan, until)
        code, listing = call("list", timeout=30)
        if code == 0 and any(line.split()[1:3] == [self.name, "Running"] for line in listing.splitlines()[1:]):
            raise RuntimeError(f"a dataflow named {self.name!r} is running already: stop it (dora stop --name ...)")
        code, output = call(
            "start", "dataflow.yml", "--name", self.name, "--detach", timeout=120, cwd=str(self.directory)
        )
        found = re.search(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}", output)
        if code != 0 or not found:
            raise RuntimeError(f"dora start refused {self.directory / 'dataflow.yml'}:\n{output[-2000:]}")
        self.uuid = found.group(0)
        try:
            if self.plan.realtime:  # WHY wait: start() returns with every group stepping, as inproc's does
                emit.wait_markers(
                    document["markers"],
                    "started|refused",
                    list(document["nodes"]),
                    30.0,
                    quiet=True,
                    failures_raise=True,
                )
            if until is None:
                return
            if self.plan.realtime:
                time.sleep(until)
            else:  # lockstep: every node marks when it has stepped to `until` (the gateway: received all of it)
                emit.wait_markers(
                    document["markers"], "done", list(document["nodes"]), timeout_s=max(120.0, 20 * until)
                )
        except BaseException:
            self.stop()  # the Deployment never saw this run: stop the dataflow here, never leave it running
            raise

    def stop(self) -> None:
        uuid, self.uuid = self.uuid, None
        if uuid is None:
            return
        from like_keras.backends.dora import emit

        if self.plan.realtime:
            emit.stop_in_order(self.document, DRAIN_S)
        call("stop", uuid, "--grace-duration", "10s", timeout=60)
        document = self.document
        emit.wait_markers(document["markers"], "exit", list(document["nodes"]), 15.0, quiet=True, failures_raise=False)
