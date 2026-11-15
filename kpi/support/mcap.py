"""A minimal MCAP reader and writer (stdlib), for the recordings like_keras writes (CONTRACT §8: unchunked, no summary).

WHY the ruler's own: the recording facility and replay are measured against it, so it must not be like_keras's.
Records used: Header (0x01), Footer (0x02), Schema (0x03), Channel (0x04), Message (0x05), DataEnd (0x0F); any
other opcode is skipped, as the MCAP spec asks of a reader.

Pinned for v0 (docs/friction/contract-002.md): a schema's `name` is the robot_msgs registry name, its `encoding`
"arrow-c-format" and its `data` the schema-hash preimage; `publish_time` is header.stamp in integer nanoseconds.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

MAGIC = b"\x89MCAP0\r\n"
OP_HEADER, OP_FOOTER, OP_SCHEMA, OP_CHANNEL, OP_MESSAGE, OP_DATA_END = 0x01, 0x02, 0x03, 0x04, 0x05, 0x0F


@dataclass
class Channel:
    id: int
    schema_id: int
    topic: str
    message_encoding: str
    metadata: dict[str, str] = field(default_factory=dict)


@dataclass
class Schema:
    id: int
    name: str
    encoding: str
    data: bytes


@dataclass
class Message:
    channel_id: int
    sequence: int
    log_time: int
    publish_time: int
    data: bytes


@dataclass
class Recording:
    profile: str = ""
    schemas: dict[int, Schema] = field(default_factory=dict)
    channels: dict[int, Channel] = field(default_factory=dict)
    messages: list[Message] = field(default_factory=list)

    def topics(self) -> list[str]:
        return [channel.topic for channel in self.channels.values()]

    def channel(self, suffix: str) -> Channel:
        """The one channel whose topic is `suffix` or ends with `.suffix` (exposed ports may be named either way)."""
        found = [c for c in self.channels.values() if c.topic == suffix or c.topic.endswith("." + suffix)]
        if len(found) != 1:
            raise KeyError(f"{len(found)} channels match {suffix!r}; topics: {self.topics()}")
        return found[0]

    def on(self, suffix: str) -> list[Message]:
        """The messages of the channel ending in `suffix`. A port both exposed and crossing a group may be recorded
        under two topics; that is accepted only when both carry the same bytes."""
        found = [c for c in self.channels.values() if c.topic == suffix or c.topic.endswith("." + suffix)]
        if not found:
            raise KeyError(f"no channel matches {suffix!r}; topics: {self.topics()}")
        streams = [[m for m in self.messages if m.channel_id == channel.id] for channel in found]
        if any([m.data for m in stream] != [m.data for m in streams[0]] for stream in streams[1:]):
            raise KeyError(f"{len(found)} different channels match {suffix!r}: {[c.topic for c in found]}")
        return streams[0]


def _string(buffer: bytes, offset: int) -> tuple[str, int]:
    (length,) = struct.unpack_from("<I", buffer, offset)
    return buffer[offset + 4 : offset + 4 + length].decode("utf-8"), offset + 4 + length


def read(path) -> Recording:
    data = open(path, "rb").read()
    if not data.startswith(MAGIC):
        raise ValueError(f"{path}: not an MCAP file")
    recording = Recording()
    offset = len(MAGIC)
    while offset < len(data):
        if data[offset:] == MAGIC:
            break
        opcode = data[offset]
        (length,) = struct.unpack_from("<Q", data, offset + 1)
        body = data[offset + 9 : offset + 9 + length]
        offset += 9 + length
        if opcode == OP_HEADER:
            recording.profile, _ = _string(body, 0)
        elif opcode == OP_SCHEMA:
            (schema_id,) = struct.unpack_from("<H", body, 0)
            name, cursor = _string(body, 2)
            encoding, cursor = _string(body, cursor)
            (size,) = struct.unpack_from("<I", body, cursor)
            recording.schemas[schema_id] = Schema(schema_id, name, encoding, body[cursor + 4 : cursor + 4 + size])
        elif opcode == OP_CHANNEL:
            channel_id, schema_id = struct.unpack_from("<HH", body, 0)
            topic, cursor = _string(body, 4)
            encoding, cursor = _string(body, cursor)
            (map_bytes,) = struct.unpack_from("<I", body, cursor)
            cursor += 4
            end = cursor + map_bytes
            metadata = {}
            while cursor < end:
                key, cursor = _string(body, cursor)
                value, cursor = _string(body, cursor)
                metadata[key] = value
            recording.channels[channel_id] = Channel(channel_id, schema_id, topic, encoding, metadata)
        elif opcode == OP_MESSAGE:
            channel_id, sequence, log_time, publish_time = struct.unpack_from("<HIQQ", body, 0)
            recording.messages.append(Message(channel_id, sequence, log_time, publish_time, body[22:]))
        elif opcode == OP_FOOTER:
            continue
    return recording


def _pack_string(text: str) -> bytes:
    raw = text.encode("utf-8")
    return struct.pack("<I", len(raw)) + raw


def _record(opcode: int, body: bytes) -> bytes:
    return struct.pack("<BQ", opcode, len(body)) + body


def write(path, schemas: list[Schema], channels: list[Channel], messages: list[Message]) -> None:
    """An unchunked MCAP with no summary, as like_keras v0 writes (used to feed lk.replay a crafted recording)."""
    parts = [MAGIC, _record(OP_HEADER, _pack_string("") + _pack_string("kpi"))]
    for schema in schemas:
        parts.append(_record(OP_SCHEMA, struct.pack("<H", schema.id) + _pack_string(schema.name)
                             + _pack_string(schema.encoding) + struct.pack("<I", len(schema.data)) + schema.data))
    for channel in channels:
        metadata = b"".join(_pack_string(k) + _pack_string(v) for k, v in channel.metadata.items())
        parts.append(_record(OP_CHANNEL, struct.pack("<HH", channel.id, channel.schema_id) + _pack_string(channel.topic)
                             + _pack_string(channel.message_encoding) + struct.pack("<I", len(metadata)) + metadata))
    for message in messages:
        parts.append(_record(OP_MESSAGE, struct.pack("<HIQQ", message.channel_id, message.sequence, message.log_time,
                                                     message.publish_time) + message.data))
    parts.append(_record(OP_DATA_END, struct.pack("<I", 0)))
    parts.append(_record(OP_FOOTER, struct.pack("<QQI", 0, 0, 0)))
    parts.append(MAGIC)
    with open(path, "wb") as file:
        file.write(b"".join(parts))
