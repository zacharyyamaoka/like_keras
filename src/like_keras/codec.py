"""The wire (CONTRACT §7): the Arrow C Data layout of robot_msgs types, the schema hash, LKF frames, C++ views.

like_keras owns this layout. It is derived from the robot_msgs dataclass fields, never from a transport or an Arrow
library, so a transport can change underneath without one byte of a recording changing.

    python -m like_keras.codec --cpp [--check]      writes cpp/include/like_keras/msgs/<domain>.hpp

Imported only when a plan is built or a message crosses a process: it brings numpy and robot_msgs (lean core).
"""

from __future__ import annotations

import dataclasses
import functools
import hashlib
import itertools
import pathlib
import struct
import sys
import types
import typing
from collections.abc import Mapping

import numpy as np
import robot_msgs

MAGIC = b"LKF1"
V0_TYPES = (
    "std/Header",
    "std/LogRecord",
    "std/DiagnosticRecord",
    "std/Metrics",
    "sensor/JointState",
    "control/ServoState",
    "control/ServoCommand",
    "trajectory/JointTrajectory",
)
# WHY a table: robot_msgs annotates ServoState.fault as a plain Array and coerces it to int64 in __post_init__, so
# the annotation alone would put it on the wire as float64 (docs/friction/contract-002.md, item 5).
INT64_ARRAYS = {("control/ServoState", "fault")}
NUMBER = {"g": "<f8", "l": "<i8"}


@dataclasses.dataclass(frozen=True)
class Node:
    """One Arrow child: its C Data format (g l b u +l +s +m), nullability and children; `message` on a struct."""

    name: str
    format: str
    nullable: bool = False
    children: tuple[Node, ...] = ()
    message: type | None = None
    sample: bool = False  # an (N, J) array of a trajectory, flattened C-order on the wire


@functools.cache
def schema(cls: type) -> tuple[Node, ...]:
    """The wire children of a robot_msgs type, in dataclass field order."""
    hints = typing.get_type_hints(cls)
    return tuple(
        _node(cls, field.name, hints[field.name], "sample" in field.metadata) for field in dataclasses.fields(cls)
    )


def _node(cls: type, name: str, annotation, sample: bool = False) -> Node:
    args = typing.get_args(annotation)
    nullable = type(None) in args and typing.get_origin(annotation) in (typing.Union, types.UnionType)
    if nullable:
        (annotation,) = [arg for arg in args if arg is not type(None)]
        args = typing.get_args(annotation)
    origin = typing.get_origin(annotation)
    scalar = {float: "g", int: "l", bool: "b", str: "u"}.get(annotation)
    if scalar or origin is typing.Literal:
        return Node(name, scalar or "u", nullable)
    if annotation is np.ndarray:
        item = "l" if (cls.type_name, name) in INT64_ARRAYS else "g"
        return Node(name, "+l", nullable, (Node("item", item),), sample=sample)
    if origin is tuple and len(args) == 2 and args[1] is Ellipsis:
        return Node(name, "+l", nullable, (_node(cls, "item", args[0]),))
    if origin is Mapping and args[0] is str:
        entries = Node("entries", "+s", False, (Node("key", "u"), _node(cls, "value", args[1])))
        return Node(name, "+m", nullable, (entries,))
    if isinstance(annotation, type) and issubclass(annotation, robot_msgs.Msg):
        return Node(name, "+s", nullable, schema(annotation), message=annotation)
    raise TypeError(
        f"{cls.type_name}.{name}: {annotation} has no v0 wire layout (CONTRACT §7.1); "
        "send a type made of numbers, strings, arrays and nested messages"
    )


@functools.cache
def preimage(cls: type) -> str:
    """`<registry name>|` then `<child>:<format>:<nullable>;` for every child, depth first (CONTRACT §7.1)."""
    parts = [f"{cls.type_name}|"]

    def walk(nodes: tuple[Node, ...]) -> None:
        for node in nodes:
            parts.append(f"{node.name}:{node.format}:{int(node.nullable)};")
            walk(node.children)

    walk(schema(cls))
    return "".join(parts)


@functools.cache
def schema_hash(cls: type) -> int:
    """The first 8 bytes of SHA-256 over the preimage, little-endian; it travels once per connection."""
    return int.from_bytes(hashlib.sha256(preimage(cls).encode()).digest()[:8], "little")


# ---------------------------------------------------------------- encode


def buffers(value) -> list[bytes]:
    """The message's Arrow buffers, depth first: validity, offsets, data, then the children (CONTRACT §7.5)."""
    out: list[bytes] = []
    _encode(Node("", "+s", children=schema(type(value)), message=type(value)), [value], out)
    return out


def lkf(value) -> bytes:
    """One message as an LKF frame: the Arrow C Data buffers flattened, 8-byte aligned."""
    parts = buffers(value)
    head = MAGIC + struct.pack(f"<IQq{len(parts)}Q", len(parts), schema_hash(type(value)), 1, *map(len, parts))
    return head + b"".join(part + bytes(-len(part) % 8) for part in parts)


def _validity(values: list) -> bytes:
    if all(value is not None for value in values):
        return b""  # the C interface's null pointer: no null slot
    bits = bytearray((len(values) + 7) // 8)
    for index, value in enumerate(values):
        if value is not None:
            bits[index // 8] |= 1 << (index % 8)
    return bytes(bits)


def _offsets(counts) -> bytes:
    totals = list(itertools.accumulate(counts, initial=0))
    return struct.pack(f"<{len(totals)}i", *totals)


def _zero(node: Node):
    """What a non-nullable child holds under a null struct slot (the reading of docs/friction/contract-002.md)."""
    if node.format == "+s":
        return types.SimpleNamespace(
            **{child.name: None if child.nullable else _zero(child) for child in node.children}
        )
    return {"g": 0.0, "l": 0, "b": False, "u": "", "+l": (), "+m": {}}[node.format]


def _encode(node: Node, values: list, out: list[bytes]) -> None:
    out.append(_validity(values))
    kind = node.format
    if kind in NUMBER:  # struct packs a float's bits as they are: a NaN keeps its payload
        out.append(
            struct.pack(
                f"<{len(values)}{'d' if kind == 'g' else 'q'}", *(0 if value is None else value for value in values)
            )
        )
    elif kind == "b":
        out.append(np.packbits(np.array([bool(value) for value in values]), bitorder="little").tobytes())
    elif kind == "u":
        encoded = [b"" if value is None else value.encode() for value in values]
        out += [_offsets(map(len, encoded)), b"".join(encoded)]
    elif kind == "+l":
        item = node.children[0]
        rows = [() if value is None else value for value in values]
        if item.format in NUMBER:  # WHY whole arrays, not elements: a 7 x 1000 plan is one buffer, NaN payloads kept
            arrays = [np.asarray(row, dtype=NUMBER[item.format]).ravel() for row in rows]
            out += [_offsets(map(len, arrays)), b"", b"".join(array.tobytes() for array in arrays)]
        else:
            out.append(_offsets(map(len, rows)))
            _encode(item, [element for row in rows for element in row], out)
    elif kind == "+s":
        for child in node.children:
            column = [
                getattr(value, child.name) if value is not None else None if child.nullable else _zero(child)
                for value in values
            ]
            _encode(child, column, out)
    elif kind == "+m":
        entries = [list(value.items()) if value is not None else [] for value in values]
        out += [_offsets(map(len, entries)), b""]  # the entries struct is never null
        key, item = node.children[0].children
        pairs = [pair for row in entries for pair in row]
        _encode(key, [name for name, _ in pairs], out)
        _encode(item, [element for _, element in pairs], out)


# ---------------------------------------------------------------- decode


def decode(frame: bytes, cls: type):
    """The robot_msgs message an LKF frame carries (None for length 0). Arrays are read-only numpy views into
    `frame`, handed to robot_msgs with `adopt` (no copy); a schema hash of another type is refused."""
    if frame[:4] != MAGIC:
        raise ValueError(f"not an LKF frame (magic {bytes(frame[:4])!r})")
    count, hash_value, length = struct.unpack_from("<IQq", frame, 4)
    if hash_value != schema_hash(cls):
        raise ValueError(
            f"schema hash {hash_value:016x} is not {cls.type_name}'s {schema_hash(cls):016x}: "
            "the sender's robot_msgs differs from this one"
        )
    if length == 0:
        return None
    view = memoryview(frame)
    sizes = struct.unpack_from(f"<{count}Q", frame, 24)
    parts, cursor = [], 24 + 8 * count
    for size in sizes:
        parts.append(view[cursor : cursor + size])
        cursor += size + (-size % 8)
    return _decode(Node("", "+s", children=schema(cls), message=cls), 1, iter(parts))[0]


def _valid(bitmap, index: int) -> bool:
    return not len(bitmap) or bool(bitmap[index // 8] >> (index % 8) & 1)


def _decode(node: Node, length: int, parts) -> list:
    validity = next(parts)
    kind = node.format
    if kind in NUMBER:
        column = np.frombuffer(next(parts), NUMBER[kind], length).tolist()
    elif kind == "b":
        column = np.unpackbits(np.frombuffer(next(parts), np.uint8), bitorder="little")[:length].astype(bool).tolist()
    elif kind == "u":
        offsets, data = np.frombuffer(next(parts), "<i4").tolist(), bytes(next(parts))
        column = [data[offsets[i] : offsets[i + 1]].decode() for i in range(length)]
    elif kind == "+l":
        offsets, item = np.frombuffer(next(parts), "<i4").tolist(), node.children[0]
        if item.format in NUMBER:
            next(parts)  # the items are never null
            data = robot_msgs.adopt(np.frombuffer(next(parts), NUMBER[item.format], offsets[-1]))
            column = [data[offsets[i] : offsets[i + 1]] for i in range(length)]
        else:
            items = _decode(item, offsets[-1], parts)
            column = [tuple(items[offsets[i] : offsets[i + 1]]) for i in range(length)]
    elif kind == "+m":
        offsets = np.frombuffer(next(parts), "<i4").tolist()
        next(parts)
        key, item = node.children[0].children
        keys, items = _decode(key, offsets[-1], parts), _decode(item, offsets[-1], parts)
        column = [
            dict(zip(keys[offsets[i] : offsets[i + 1]], items[offsets[i] : offsets[i + 1]], strict=True))
            for i in range(length)
        ]
    else:
        fields = {child.name: _decode(child, length, parts) for child in node.children}
        column = [
            _message(node.message, {name: values[i] for name, values in fields.items()}, node.children)
            if _valid(validity, i)
            else None
            for i in range(length)
        ]
    return [value if _valid(validity, i) else None for i, value in enumerate(column)]


def _message(cls: type, values: dict, children: list[Node]):
    """Build the message; a trajectory's sample arrays get back their (N, J) shape from N = len(t)."""
    samples = len(values["t"]) if "t" in values else 0
    for child in children:
        value = values[child.name]
        if child.sample and value is not None and samples:
            values[child.name] = value.reshape(samples, -1)
        elif child.format == "+m" and samples and value:  # trajectory channels: per-sample, first axis N
            values[child.name] = {
                name: array.reshape(samples, -1) if array.size != samples else array for name, array in value.items()
            }
    return cls(**values)


# ---------------------------------------------------------------- C++ views (CONTRACT §7.4)

CPP_TYPES = {"g": "double", "l": "std::int64_t", "b": "bool", "u": "std::string_view"}


def _cpp_struct(cls: type) -> list[str]:
    nodes, name = schema(cls), cls.type_name.split("/")[1]
    trajectory = any(node.sample for node in nodes)
    capacity = "kMaxNames" if cls.type_name.startswith("std/") else "lk::kMaxJoints"
    lines = [
        "",
        f"struct {name} {{",
        f'  static constexpr std::string_view kType = "{cls.type_name}";',
        f"  static constexpr std::uint64_t kHash = 0x{schema_hash(cls):016x}ULL;",
    ]
    if trajectory:
        lines.append("  // read-only views over the received buffers, valid for the step; row-major (N, J); N = t.size()")
    elif any(node.format == "+l" for node in nodes):
        lines.append("  std::size_t size = 0;  // how many joints (names) the arrays below hold")
    for node in nodes:
        item = node.children[0].format if node.children else ""
        if node.format == "+s" and node.message is robot_msgs.std.Header:
            declaration = "Header"
        elif trajectory and node.format == "+l" and item == "u":
            # WHY (CONTRACT §15 A16): a C++ controller maps a plan by name, as its Python peer does, never by column
            lines.append("  std::size_t joints = 0;  // how many names joint_names holds (one per column of the arrays)")
            declaration = f"std::array<std::string_view, {capacity}>"
        elif trajectory:
            declaration = "std::span<const double>" if node.format == "+l" and item == "g" else ""
        elif node.format == "+l":
            declaration = f"std::array<{CPP_TYPES[item]}, {capacity}>"
        else:
            declaration = CPP_TYPES.get(node.format, "")
        if declaration:  # maps (Header.meta, channels) and limits of a plan stay Python-side in v0
            lines.append(f"  {f'std::optional<{declaration}>' if node.nullable else declaration} {node.name}{{}};")
    return lines + ["};"]


def cpp_headers() -> dict[str, str]:
    """`<domain>.hpp` -> its text: namespace rm, one struct per v0 type, fixed capacity (CONTRACT §7.4)."""
    registry = robot_msgs.message_types()
    generator = hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()[:16]
    domains: dict[str, list[str]] = {}
    for name in V0_TYPES:
        domains.setdefault(name.split("/")[0], []).extend(_cpp_struct(registry[name]))
    out = {}
    for domain, body in domains.items():
        text = "\n".join(body)
        needed = {
            "array": "std::array",
            "cstddef": "std::size_t",
            "cstdint": "std::",
            "optional": "std::optional",
            "span": "std::span",
            "string_view": "std::string_view",
        }
        includes = [f"#include <{header}>" for header, token in needed.items() if token in text]
        includes += [
            f"#include <like_keras/{header}>"
            for header, token in (("lk.hpp", "lk::"), ("msgs/std.hpp", "Header"))
            if token in text and domain != "std"
        ]
        constants = ["", "inline constexpr std::size_t kMaxNames = 64;  // facility records: names, metrics, inputs"]
        out[f"{domain}.hpp"] = "\n".join(
            [
                f"// GENERATED by `python -m like_keras.codec --cpp` (generator {generator}); do not edit.",
                f"// The {domain}/ types of robot_msgs {robot_msgs.__version__} as fixed-capacity C++ views (CONTRACT §7.4).",
                "#pragma once",
                "",
                *includes,
                "",
                "namespace rm {",
                *(constants if domain == "std" else []),
                *body,
                "",
                "}  // namespace rm",
                "",
            ]
        )
    return out


def write_cpp(check: bool = False) -> int:
    """Write (or with check, compare) the committed views: CMake never runs Python, CI runs `--cpp --check`."""
    out = pathlib.Path(__file__).resolve().parents[2] / "cpp" / "include" / "like_keras" / "msgs"
    headers = cpp_headers()
    stale = [name for name, text in headers.items() if not (out / name).exists() or (out / name).read_text() != text]
    if check:
        print(f"stale: {' '.join(stale)}" if stale else "the C++ views are current")
        return 1 if stale else 0
    out.mkdir(parents=True, exist_ok=True)
    for name, text in headers.items():
        (out / name).write_text(text)
    print(f"wrote {', '.join(sorted(headers))} to {out}")
    return 0


if __name__ == "__main__":
    if "--cpp" not in sys.argv[1:]:
        sys.exit("usage: python -m like_keras.codec --cpp [--check]")
    sys.exit(write_cpp(check="--check" in sys.argv[1:]))
