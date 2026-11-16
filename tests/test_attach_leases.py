"""Lane dora's test of CONTRACT §15 A36 (consumer-019) where the frozen ruler test (consumer::
test_attach_lease_per_input, writers at 20 Hz, well inside the 0.25 s lease) does not reach: the proxy's heartbeat
(a bare LEASE every lease_s / 3) keeps every input its client holds, however long it writes nothing (a planner sends
a plan every 0.5 s); a bare LEASE from a client that holds nothing takes nothing; and a silent holder loses only its
own inputs. Raw attach frames against the always-on form of kpi/consumer/lease_parts.TwoInputs, inproc, real time.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_attach_leases.py
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time

from robot_msgs.sensor import JointState
from robot_msgs.std import Header

from kpi.consumer.lease_parts import TwoInputs
from like_keras import codec
from like_keras.attach import HELLO, LEASE, REFUSED, WRITE, frame, frames, socket_path

LEASE_S = 0.25


class Client:
    """One attach client speaking the wire directly: HELLO, WRITEs on demand, a LEASE heartbeat while `beating`."""

    def __init__(self, address: str, name: str, ports: list[str]) -> None:
        self.ports, self.refused, self.beating, self.closed = ports, [], True, False
        self.connection = socket.socket(socket.AF_UNIX)
        self.connection.connect(socket_path(address))
        self.connection.sendall(frame(HELLO, json.dumps({"client": name, "lease_s": LEASE_S}).encode()))
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def write(self, port: str, position: float) -> None:
        value = JointState(name=("value",), position=[position], header=Header(stamp=time.time()))
        self.connection.sendall(frame(WRITE, struct.pack("<H", self.ports.index(port)) + codec.lkf(value)))

    def lease(self) -> None:
        self.connection.sendall(frame(LEASE))

    def _loop(self) -> None:
        buffer = bytearray()
        self.connection.settimeout(LEASE_S / 3)
        while not self.closed:
            if self.beating:
                self.lease()
            try:
                chunk = self.connection.recv(65536)
            except TimeoutError:
                continue
            except OSError:
                return
            buffer += chunk
            self.refused += [json.loads(payload) for kind, payload in frames(buffer) if kind == REFUSED]

    def close(self) -> None:
        self.closed = True
        self.thread.join(timeout=2)
        self.connection.close()


def test_the_heartbeat_keeps_every_held_input_and_a_bare_lease_takes_none(tmp_path):
    import like_keras as lk

    name = f"leases_{os.getpid()}"
    s = lk.System(name)
    box = s.add(TwoInputs(path=str(tmp_path / "rows.jsonl")), "box")
    s.expose(box.a, box.b)
    dep = lk.Deployment(s)
    dep.place(s.box, hz=100)
    sheet = dep.describe()
    address, names = sheet["attach"]["socket"], list(sheet["exposed"])
    dep.start()
    clients = []
    try:
        planner, guard, rival = (Client(address, who, names) for who in ("planner", "guard", "rival"))
        clients += [planner, guard, rival]
        time.sleep(0.2)  # the rival's bare heartbeats come first: they must take no lease
        planner.write("a", 1.0)
        guard.write("b", 2.0)
        held_from = time.time() + 0.3
        time.sleep(0.3)
        rival.write("a", -5.0)
        time.sleep(1.2)  # 4.8 leases with no WRITE: only the heartbeats keep a and b
        held_until = time.time()
        planner.beating = False  # silent from here, its socket open: a lapses, b is the guard's and stays
        silent = time.time()
        time.sleep(1.2)
        ended = time.time()  # closing the guard's socket below retracts b: the window ends here
    finally:
        for client in clients:
            client.close()
        dep.stop()
    rows = [json.loads(line) for line in (tmp_path / "rows.jsonl").read_text().splitlines()]
    held = [row for row in rows if held_from <= row["wall"] <= held_until]
    assert len(held) >= 50 and all(row["a"] == 1.0 and row["b"] == 2.0 for row in held), (
        f"the heartbeats did not keep both inputs: {[(row['a'], row['b']) for row in held][::10]}"
    )
    assert [refusal["holder"] for refusal in rival.refused] == ["planner"], rival.refused
    assert not planner.refused and not guard.refused, (planner.refused, guard.refused)
    after = [row for row in rows if silent + LEASE_S + 0.5 <= row["wall"] <= ended]
    assert after and all(row["a"] is None and row["b"] == 2.0 for row in after), (
        f"the silent planner's input a, and only it, must be retracted: {[(row['a'], row['b']) for row in after][:5]}"
    )
