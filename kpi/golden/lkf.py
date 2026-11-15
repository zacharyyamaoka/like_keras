"""The ruler's own LKF codec (CONTRACT §7.1, §7.5): numpy + struct, written from the contract, never from like_keras.

WHY a second, independent codec: the golden bytes (T10) are only a ruler if two encoders written apart agree on
them. This one knows the v0 types as a frozen table (SCHEMAS, field order = the robot_msgs dataclass order) and
never imports like_keras, robot_msgs, dora or pyarrow. kpi/golden/test_golden.py checks the table against the
robot_msgs dataclasses and checks the bytes against pyarrow's Arrow validator.

The interpretation of the contract's open points is pinned here and in docs/friction/contract-002.md:
- buffers per node are the Arrow C Data Interface's: struct 1 (validity), primitive and bool 2 (validity, data),
  utf8 3 (validity, int32 offsets, data), list and map 2 (validity, int32 offsets), then the children, depth first;
- a validity buffer is empty (length 0, the C interface's null pointer) when its node has no null slot;
- `buffer lengths[n]` are the unpadded lengths; each buffer starts 8-byte aligned and the padding is zero bytes;
- `array length` 0 (no value) carries no buffers (n = 0);
- a null slot holds the zero value (0, 0.0, false, "" or an empty list); the children of a null struct slot are
  null where nullable, else their zero value;
- the schema hash preimage spells nullable as 1 / 0, names a list's child `item` and a map's children
  `entries` (struct), `key`, `value`, as the Arrow spec does; map entries keep the mapping's iteration order;
- `Mapping[str, Array]` (JointTrajectory.channels, outside §7.1's table) is a map of utf8 to list of float64;
- ServoState.fault is an int64 array (robot_msgs coerces it in __post_init__; the annotation says only Array).
"""

from __future__ import annotations

import hashlib
import struct
from dataclasses import dataclass
from typing import Any

import numpy as np

MAGIC = b"LKF1"
HEADER_BYTES = 24


@dataclass(frozen=True)
class Field:
    name: str
    kind: str  # f64 i64 bool utf8 strs f64s i64s struct map_str map_f64s
    nullable: bool = False
    of: str | None = None  # the nested type for kind "struct"
    item_nullable: bool = False  # strs whose items may be None (JointLimits.position_kind)


def _f(name: str, kind: str, nullable: bool = False, of: str | None = None, item_nullable: bool = False) -> Field:
    return Field(name, kind, nullable, of, item_nullable)


# Field order is the dataclass order in robot_msgs (checked by test_golden.py::test_schema_table_matches_robot_msgs).
SCHEMAS: dict[str, tuple[Field, ...]] = {
    "std/Header": (_f("stamp", "f64"), _f("frame_id", "utf8"), _f("meta", "map_str")),
    "sensor/JointState": (
        _f("name", "strs"),
        _f("position", "f64s"),
        _f("velocity", "f64s", True),
        _f("effort", "f64s", True),
        _f("header", "struct", of="std/Header"),
    ),
    "control/ServoState": (
        _f("name", "strs"),
        _f("position", "f64s"),
        _f("velocity", "f64s", True),
        _f("effort", "f64s", True),
        _f("temperature", "f64s", True),
        _f("voltage", "f64s", True),
        _f("fault", "i64s", True),
        _f("seq", "i64"),
        _f("header", "struct", of="std/Header"),
    ),
    "control/ServoCommand": (
        _f("name", "strs"),
        _f("position", "f64s"),
        _f("velocity", "f64s", True),
        _f("feedforward_effort", "f64s", True),
        _f("stiffness", "f64s", True),
        _f("damping", "f64s", True),
        _f("max_effort", "f64s", True),
        _f("state_stamp", "f64", True),
        _f("state_seq", "i64", True),
        _f("header", "struct", of="std/Header"),
    ),
    "trajectory/JointLimits": (
        _f("joint_names", "strs"),
        _f("min_position", "f64s", True),
        _f("max_position", "f64s", True),
        _f("max_velocity", "f64s", True),
        _f("max_acceleration", "f64s", True),
        _f("max_jerk", "f64s", True),
        _f("max_effort", "f64s", True),
        _f("position_kind", "strs", True, item_nullable=True),
    ),
    "trajectory/JointTrajectory": (
        _f("t", "f64s"),
        _f("channels", "map_f64s"),
        _f("header", "struct", of="std/Header"),
        _f("joint_names", "strs"),
        _f("positions", "f64s"),
        _f("velocities", "f64s", True),
        _f("accelerations", "f64s", True),
        _f("effort", "f64s", True),
        _f("limits", "struct", True, of="trajectory/JointLimits"),
    ),
    # CONTRACT §2.3: the facility types the robot_msgs lane adds.
    "std/LogRecord": (
        _f("header", "struct", of="std/Header"),
        _f("path", "utf8"),
        _f("tick", "i64"),
        _f("level", "i64"),
        _f("message", "utf8"),
    ),
    "std/DiagnosticRecord": (
        _f("header", "struct", of="std/Header"),
        _f("path", "utf8"),
        _f("level", "utf8"),
        _f("code", "utf8"),
        _f("message", "utf8"),
        _f("mode", "utf8"),
        _f("expected_period_s", "f64"),
        _f("uptime_s", "f64"),
        _f("stale_inputs", "strs"),
    ),
    "std/Metrics": (
        _f("header", "struct", of="std/Header"),
        _f("path", "utf8"),
        _f("names", "strs"),
        _f("values", "f64s"),
    ),
}

FORMAT = {"f64": "g", "i64": "l", "bool": "b", "utf8": "u", "strs": "+l", "f64s": "+l", "i64s": "+l",
          "struct": "+s", "map_str": "+m", "map_f64s": "+m"}


# ---------------------------------------------------------------- schema hash (CONTRACT §7.1)

def _children(field: Field) -> list[Field]:
    """The Arrow children of one field, as the C Data Interface names them."""
    if field.kind == "strs":
        return [Field("item", "utf8", field.item_nullable)]
    if field.kind == "f64s":
        return [Field("item", "f64")]
    if field.kind == "i64s":
        return [Field("item", "i64")]
    if field.kind == "struct":
        return list(SCHEMAS[field.of])
    if field.kind == "map_str":
        return [Field("entries", "_entries_str")]
    if field.kind == "map_f64s":
        return [Field("entries", "_entries_f64s")]
    if field.kind == "_entries_str":
        return [Field("key", "utf8"), Field("value", "utf8")]
    if field.kind == "_entries_f64s":
        return [Field("key", "utf8"), Field("value", "f64s")]
    return []


def _format(field: Field) -> str:
    return "+s" if field.kind.startswith("_entries") else FORMAT[field.kind]


def schema_preimage(type_name: str) -> str:
    """`<registry name>|` then `<child name>:<format>:<nullable>;` for every child, depth first."""
    parts = [f"{type_name}|"]

    def walk(fields: list[Field] | tuple[Field, ...]) -> None:
        for field in fields:
            parts.append(f"{field.name}:{_format(field)}:{1 if field.nullable else 0};")
            walk(_children(field))

    walk(SCHEMAS[type_name])
    return "".join(parts)


def schema_hash(type_name: str) -> int:
    """First 8 bytes of SHA-256 over the preimage, read little-endian (shown as `%016x`)."""
    return int.from_bytes(hashlib.sha256(schema_preimage(type_name).encode("utf-8")).digest()[:8], "little")


# ---------------------------------------------------------------- encode

def _validity(values: list[Any]) -> bytes:
    if all(value is not None for value in values):
        return b""
    bits = bytearray((len(values) + 7) // 8)
    for index, value in enumerate(values):
        if value is not None:
            bits[index // 8] |= 1 << (index % 8)
    return bytes(bits)


def _zero(field: Field) -> Any:
    """The placeholder a non-nullable child holds under a null struct slot."""
    if field.kind == "struct":
        return {child.name: (None if child.nullable else _zero(child)) for child in SCHEMAS[field.of]}
    return {"f64": 0.0, "i64": 0, "bool": False, "utf8": "", "strs": (), "f64s": np.zeros(0), "i64s": np.zeros(0, np.int64),
            "map_str": {}, "map_f64s": {}}[field.kind]


def _offsets(counts: list[int]) -> bytes:
    return np.concatenate([[0], np.cumsum(counts, dtype=np.int64)]).astype("<i4").tobytes()


def _encode(field: Field, values: list[Any], out: list[bytes]) -> None:
    """Append the buffers of one node (a column of `values`, None = null) and its children, depth first."""
    kind = field.kind
    out.append(_validity(values))
    if kind in ("f64", "i64"):
        dtype = "<f8" if kind == "f64" else "<i8"
        column = np.zeros(len(values), dtype=dtype)
        for index, value in enumerate(values):
            if value is not None:
                column[index : index + 1] = np.asarray([value], dtype=dtype)  # keeps a NaN's payload bits
        out.append(column.tobytes())
    elif kind == "bool":
        bits = bytearray((len(values) + 7) // 8)
        for index, value in enumerate(values):
            if value:
                bits[index // 8] |= 1 << (index % 8)
        out.append(bytes(bits))
    elif kind == "utf8":
        encoded = [b"" if value is None else str(value).encode("utf-8") for value in values]
        out.append(_offsets([len(item) for item in encoded]))
        out.append(b"".join(encoded))
    elif kind in ("strs", "f64s", "i64s"):
        items: list[Any] = []
        counts = []
        for value in values:
            if value is None:
                counts.append(0)
                continue
            flat = list(value) if kind == "strs" else list(np.asarray(value, dtype="<f8" if kind == "f64s" else "<i8").ravel(order="C"))
            counts.append(len(flat))
            items.extend(flat)
        out.append(_offsets(counts))
        _encode(_children(field)[0], items, out)
    elif kind == "struct":
        children = SCHEMAS[field.of]
        for child in children:
            column = []
            for value in values:
                if value is None:
                    column.append(None if child.nullable else _zero(child))
                else:
                    column.append(value[child.name])
            _encode(child, column, out)
    elif kind in ("map_str", "map_f64s"):
        keys: list[Any] = []
        mapped: list[Any] = []
        counts = []
        for value in values:
            entries = list((value or {}).items())
            counts.append(len(entries))
            keys.extend(key for key, _ in entries)
            mapped.extend(item for _, item in entries)
        out.append(_offsets(counts))
        entries_field = _children(field)[0]
        out.append(b"")  # the entries struct: never null
        key_field, value_field = _children(entries_field)
        _encode(key_field, keys, out)
        _encode(value_field, mapped, out)
    else:
        raise ValueError(f"unknown kind {kind!r}")


def nodes(type_name: str, message: dict[str, Any]) -> list[tuple[int, int, int]]:
    """(length, buffer count, child count) of every Arrow node of one message, depth first: what a C decoder needs to
    rebuild the ArrowArray tree from an LKF frame without knowing the type (kpi/alloc_probe.cpp reads it)."""
    out: list[tuple[int, int, int]] = []

    def walk(field: Field, values: list[Any]) -> None:
        kind = field.kind
        buffers = {"f64": 2, "i64": 2, "bool": 2, "utf8": 3}.get(kind, 2 if kind not in ("struct",) else 1)
        children = _children(field)
        out.append((len(values), buffers, len(children)))
        if kind in ("strs", "f64s", "i64s"):
            items: list[Any] = []
            for value in values:
                if value is not None:
                    items.extend(list(value) if kind == "strs" else list(np.asarray(value).ravel()))
            walk(children[0], items)
        elif kind == "struct":
            for child in children:
                walk(child, [None if value is None else value[child.name] for value in values])
        elif kind in ("map_str", "map_f64s"):
            entries = [entry for value in values for entry in (value or {}).items()]
            out.append((len(entries), 1, 2))
            key_field, value_field = _children(children[0])
            walk(key_field, [key for key, _ in entries])
            walk(value_field, [item for _, item in entries])

    walk(Field("", "struct", of=type_name), [message])
    return out


def encode(type_name: str, message: dict[str, Any] | None) -> bytes:
    """One message (a dict keyed by field name, nested dicts for nested messages) as an LKF frame."""
    buffers: list[bytes] = []
    if message is not None:
        _encode(Field("", "struct", of=type_name), [message], buffers)
    head = MAGIC + struct.pack("<IQq", len(buffers), schema_hash(type_name), 0 if message is None else 1)
    lengths = struct.pack(f"<{len(buffers)}Q", *(len(buffer) for buffer in buffers))
    body = b"".join(buffer + b"\0" * (-len(buffer) % 8) for buffer in buffers)
    return head + lengths + body


# ---------------------------------------------------------------- decode

def split(frame: bytes) -> tuple[int, int, list[bytes]]:
    """(schema hash, array length, buffers) of one LKF frame; checks the magic, the alignment and the padding."""
    if frame[:4] != MAGIC:
        raise ValueError(f"not an LKF frame: magic {frame[:4]!r}")
    count, hash_value, length = struct.unpack_from("<IQq", frame, 4)
    lengths = struct.unpack_from(f"<{count}Q", frame, HEADER_BYTES)
    cursor = HEADER_BYTES + 8 * count
    buffers = []
    for size in lengths:
        if cursor % 8:
            raise ValueError(f"buffer at offset {cursor} is not 8-byte aligned")
        buffers.append(frame[cursor : cursor + size])
        padding = frame[cursor + size : cursor + size + (-size % 8)]
        if padding.strip(b"\0"):
            raise ValueError(f"non-zero padding after the buffer at offset {cursor}")
        cursor += size + (-size % 8)
    if cursor != len(frame):
        raise ValueError(f"frame has {len(frame) - cursor} trailing bytes")
    return hash_value, length, buffers


def _valid(bitmap: bytes, index: int) -> bool:
    return not bitmap or bool(bitmap[index // 8] >> (index % 8) & 1)


def _decode(field: Field, length: int, buffers: list[bytes], cursor: list[int]) -> list[Any]:
    def take() -> bytes:
        buffer = buffers[cursor[0]]
        cursor[0] += 1
        return buffer

    kind = field.kind
    validity = take()
    if kind in ("f64", "i64"):
        column = np.frombuffer(take(), dtype="<f8" if kind == "f64" else "<i8", count=length)
        return [column[index] if _valid(validity, index) else None for index in range(length)]
    if kind == "bool":
        data = take()
        return [bool(data[index // 8] >> (index % 8) & 1) if _valid(validity, index) else None for index in range(length)]
    if kind == "utf8":
        offsets = np.frombuffer(take(), dtype="<i4", count=length + 1)
        data = take()
        return [data[offsets[i] : offsets[i + 1]].decode("utf-8") if _valid(validity, i) else None for i in range(length)]
    if kind in ("strs", "f64s", "i64s"):
        offsets = np.frombuffer(take(), dtype="<i4", count=length + 1)
        items = _decode(_children(field)[0], int(offsets[-1]) if length else 0, buffers, cursor)
        rows: list[Any] = []
        for index in range(length):
            chunk = items[offsets[index] : offsets[index + 1]]
            if not _valid(validity, index):
                rows.append(None)
            elif kind == "strs":
                rows.append(tuple(chunk))
            else:
                rows.append(np.array(chunk, dtype="<f8" if kind == "f64s" else "<i8"))
        return rows
    if kind == "struct":
        columns = {child.name: _decode(child, length, buffers, cursor) for child in SCHEMAS[field.of]}
        return [{name: column[index] for name, column in columns.items()} if _valid(validity, index) else None
                for index in range(length)]
    if kind in ("map_str", "map_f64s"):
        offsets = np.frombuffer(take(), dtype="<i4", count=length + 1)
        total = int(offsets[-1]) if length else 0
        entries_validity = take()
        if entries_validity:
            raise ValueError("a map's entries struct has a validity buffer")
        key_field, value_field = _children(_children(field)[0])
        keys = _decode(key_field, total, buffers, cursor)
        values = _decode(value_field, total, buffers, cursor)
        return [dict(zip(keys[offsets[i] : offsets[i + 1]], values[offsets[i] : offsets[i + 1]], strict=True))
                if _valid(validity, i) else None for i in range(length)]
    raise ValueError(f"unknown kind {kind!r}")


def decode(type_name: str, frame: bytes) -> dict[str, Any] | None:
    """The message dict an LKF frame carries (None for length 0); refuses a schema hash of another type."""
    hash_value, length, buffers = split(frame)
    if hash_value != schema_hash(type_name):
        raise ValueError(f"schema hash {hash_value:016x} is not {type_name}'s {schema_hash(type_name):016x}")
    if length == 0:
        return None
    cursor = [0]
    rows = _decode(Field("", "struct", of=type_name), length, buffers, cursor)
    if cursor[0] != len(buffers):
        raise ValueError(f"{len(buffers) - cursor[0]} buffers left over")
    return rows[0]


# ---------------------------------------------------------------- comparison

def bits_equal(left: Any, right: Any) -> bool:
    """Equality that compares floats by their bits (a NaN's payload included)."""
    if isinstance(left, dict) and isinstance(right, dict):
        return left.keys() == right.keys() and all(bits_equal(left[key], right[key]) for key in left)
    if isinstance(left, (tuple, list)) and isinstance(right, (tuple, list)) and not isinstance(left, np.ndarray):
        return len(left) == len(right) and all(bits_equal(a, b) for a, b in zip(left, right, strict=True))
    if left is None or right is None:
        return left is None and right is None
    if isinstance(left, (float, np.floating, np.ndarray)) or isinstance(right, (float, np.floating, np.ndarray)):
        # WHY flat: the wire carries the C-order flat buffer; the (N, J) shape comes from the schema, not the bytes.
        a, b = np.asarray(left), np.asarray(right)
        if a.dtype.kind != b.dtype.kind and not (a.dtype.kind in "iu" and b.dtype.kind in "iu"):
            return False
        return a.size == b.size and np.ascontiguousarray(a.ravel()).tobytes() == np.ascontiguousarray(b.astype(a.dtype).ravel()).tobytes()
    return left == right
