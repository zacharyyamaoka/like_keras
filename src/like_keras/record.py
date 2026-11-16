"""Recording and replay (CONTRACT §8): MCAP files of LKF messages, and `replay`, a source that plays one back.

    dep.record("run.mcap")                         # every edge between groups and every exposed Out
    targets = s.add(lk.replay("run.mcap"), "targets")
    planner = s.add(PickPlanner(), target=targets.targets__target, state=arm.state)

v0 writes MCAP unchunked and without a summary: one channel per port (topic `<path>.<port>`, message encoding
`lkf`, schema = the Arrow C Data preimage, encoding `arrow-c-format`), log_time = when recorded, publish_time =
header.stamp. Producers only append to a list; encoding and writing happen off the step, on the flush.
"""

from __future__ import annotations

import collections
import dataclasses
import os
import struct
import time
from typing import Any

from like_keras.component import Component, Out

MAGIC = b"\x89MCAP0\r\n"
HEADER, FOOTER, SCHEMA, CHANNEL, MESSAGE, DATA_END = 0x01, 0x02, 0x03, 0x04, 0x05, 0x0F


def _text(value: str) -> bytes:
    raw = value.encode()
    return struct.pack("<I", len(raw)) + raw


class Recorder:
    """One MCAP file being written. `channel()` returns the sink a port's sends go to."""

    def __init__(self, path: str | os.PathLike) -> None:
        self._file = open(path, "wb")
        self._file.write(MAGIC + self._record(HEADER, _text("") + _text("like_keras 0.1.0")))
        self._schemas: dict[str, int] = {}
        self._pending: collections.deque = collections.deque()
        self._sequence = self._channels = 0
        self._sinks: dict[str, Any] = {}  # topic -> its sink: a System's facility port is many leaves, one channel

    @staticmethod
    def _record(opcode: int, body: bytes) -> bytes:
        return struct.pack("<BQ", opcode, len(body)) + body

    def channel(self, topic: str, kind: type):
        from like_keras import codec

        if topic in self._sinks:
            return self._sinks[topic]
        if kind.type_name not in self._schemas:
            self._schemas[kind.type_name] = len(self._schemas) + 1
            self._file.write(
                self._record(
                    SCHEMA,
                    struct.pack("<H", self._schemas[kind.type_name])
                    + _text(kind.type_name)
                    + _text("arrow-c-format")
                    + _text(codec.preimage(kind)),
                )
            )
        self._channels += 1
        channel_id = self._channels
        self._file.write(
            self._record(
                CHANNEL,
                struct.pack("<HH", channel_id, self._schemas[kind.type_name])
                + _text(topic)
                + _text("lkf")
                + struct.pack("<I", 0),
            )
        )
        pending = self._pending

        def sink(value: Any, sent: float) -> None:  # any group thread; never blocks
            pending.append((channel_id, time.time_ns(), value))

        self._sinks[topic] = sink
        return sink

    def flush(self) -> None:
        from like_keras import codec

        records = []
        while self._pending:
            channel_id, log_time, value = self._pending.popleft()
            stamp = getattr(getattr(value, "header", value), "stamp_ns", log_time)
            self._sequence += 1
            records.append(
                self._record(
                    MESSAGE, struct.pack("<HIQQ", channel_id, self._sequence, log_time, stamp) + codec.lkf(value)
                )
            )
        self._file.write(b"".join(records))
        # WHY to the file, not the buffer: a recording must hold every round up to the last one when its process is
        # killed (an attach client's health under kill -9, CONTRACT §15 A36, T5); an unflushed small file was empty
        self._file.flush()

    def close(self) -> None:
        if self._file.closed:
            return
        self.flush()
        self._file.write(self._record(DATA_END, struct.pack("<I", 0)) + self._record(FOOTER, bytes(20)) + MAGIC)
        self._file.close()


def read(path: str | os.PathLike) -> tuple[dict[int, tuple[str, str]], list[tuple[int, int, bytes]]]:
    """({channel id: (topic, registry name)}, [(channel id, publish_time ns, LKF frame)]) of a like_keras MCAP."""
    data = open(path, "rb").read()
    if not data.startswith(MAGIC):
        raise ValueError(f"{path} is not an MCAP file")
    schemas, channels, messages, offset = {}, {}, [], len(MAGIC)
    while offset + 9 <= len(data):
        opcode, length = struct.unpack_from("<BQ", data, offset)
        body, offset = data[offset + 9 : offset + 9 + length], offset + 9 + length
        if opcode == SCHEMA:
            schema_id, size = struct.unpack_from("<HI", body)
            schemas[schema_id] = body[6 : 6 + size].decode()
        elif opcode == CHANNEL:
            channel_id, schema_id, size = struct.unpack_from("<HHI", body)
            channels[channel_id] = (body[8 : 8 + size].decode(), schemas.get(schema_id, ""))
        elif opcode == MESSAGE:
            channel_id, _, _, publish_time = struct.unpack_from("<HIQQ", body)
            messages.append((channel_id, publish_time, body[22:]))
    return channels, messages


def replay(path: str | os.PathLike) -> Component:
    """A source whose Outs are a recording's channels: the exposed name when the channel was an exposed port, else
    `<child>__<port>`. Placed in an hz group, it sends each message at the first tick at or after its publish time;
    under lockstep that is its recorded time."""
    import robot_msgs

    channels, _ = read(path)
    registry = robot_msgs.message_types()
    ports, topics = {}, {}
    for topic, type_name in channels.values():
        name = "__".join(topic.split(".")[1:])  # drop the System's own name
        ports[name], topics[name] = Out(registry[type_name]), topic
    cls = type("Replay", (_Replay,), {**ports, "__module__": __name__, "_lk_topics": topics})
    return cls(path=os.fspath(path))


class _Replay(Component):
    _lk_impl = "like_keras:replay"

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def activate(self) -> None:
        channels, messages = read(self.params.path)
        self._pending = {name: collections.deque() for name in type(self)._lk_outs}
        by_topic = {topic: name for name, topic in type(self)._lk_topics.items()}
        for channel_id, publish_time, frame in messages:
            self._pending[by_topic[channels[channel_id][0]]].append((publish_time, frame))

    def forward(self):
        from like_keras import codec

        now_ns, out = round(self.now() * 1e9), []
        for name in type(self)._lk_outs:
            pending = self._pending[name]
            due = pending and pending[0][0] <= now_ns
            out.append(codec.decode(pending.popleft()[1], type(self)._lk_ports[name].type) if due else None)
        return out[0] if len(out) == 1 else tuple(out)

    def _lk_params(self) -> dict[str, Any]:
        return {"path": self.params.path}
