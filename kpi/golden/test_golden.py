"""The golden LKF bytes (CONTRACT §7.2, §7.5; T10's Python half).

The ruler's own half runs today: the committed files are what kpi/golden/lkf.py writes, they are valid Arrow
(pyarrow rebuilds and fully validates them) and the schema table matches the robot_msgs dataclasses. The like_keras
half uses only the public API: a component that emits the instance is recorded (encode), and lk.replay plays a
recording of the golden frame back into a component (decode); both must reproduce the bytes bit for bit.
"""

from __future__ import annotations

import dataclasses
import typing

import numpy as np
import pytest

from kpi.golden import arrow_oracle, lkf, make_golden
from kpi.golden.instances import build, emitter
from kpi.support import mcap
from kpi.support.lk import like_keras, recorded

TYPES = sorted(make_golden.INSTANCES)


def as_dict(type_name: str, message) -> dict:
    """A robot_msgs object back in the instance-dict form (numpy arrays, tuples, nested dicts)."""
    out = {}
    for field in lkf.SCHEMAS[type_name]:
        value = getattr(message, field.name)
        if field.kind == "struct" and value is not None:
            value = as_dict(field.of, value)
        elif field.kind in ("map_str", "map_f64s"):
            value = dict(value)
        out[field.name] = value
    return out


def test_committed_goldens_reproduce():
    assert make_golden.main(["--check"]) == 0, "kpi/golden/*.lkf differ from what make_golden.py writes"


@pytest.mark.parametrize("type_name", TYPES)
def test_frame_layout(type_name):
    frame = make_golden.golden_path(type_name).read_bytes()
    hash_value, length, buffers = lkf.split(frame)  # checks magic, 8-byte alignment, zero padding, no trailing bytes
    assert frame[:4] == b"LKF1" and length == 1 and hash_value == lkf.schema_hash(type_name)
    assert lkf.bits_equal(lkf.decode(type_name, frame), make_golden.INSTANCES[type_name])


@pytest.mark.parametrize("type_name", TYPES)
def test_golden_is_valid_arrow(type_name):
    """pyarrow rebuilds each frame from its raw buffers and runs Arrow's full validation; the values survive,
    NaN payload bits included."""
    array = arrow_oracle.rebuild(type_name, make_golden.golden_path(type_name).read_bytes())
    assert lkf.bits_equal(arrow_oracle.to_message(type_name, array), make_golden.INSTANCES[type_name])


def test_nan_payload_and_non_ascii_present():
    frames = b"".join(make_golden.golden_path(name).read_bytes() for name in TYPES)
    assert np.uint64(make_golden.NAN_PAYLOAD_BITS).tobytes() in frames
    assert "épaule_2".encode() in frames and "肘3".encode() in frames


@pytest.mark.parametrize("type_name", TYPES)
def test_schema_table_matches_robot_msgs(type_name):
    """lkf.SCHEMAS is the robot_msgs dataclass, field for field: name, order, and `X | None` as nullable."""
    import robot_msgs

    registry = robot_msgs.message_types()
    assert type_name in registry, f"robot_msgs has no {type_name}"
    cls = registry[type_name]
    hints = typing.get_type_hints(cls)
    fields = [field for field in dataclasses.fields(cls)]
    assert [field.name for field in fields] == [field.name for field in lkf.SCHEMAS[type_name]]
    for field, expected in zip(fields, lkf.SCHEMAS[type_name], strict=True):
        nullable = type(None) in typing.get_args(hints[field.name])
        assert nullable == expected.nullable, f"{type_name}.{field.name}: nullable {nullable}"
    instance = build(type_name, make_golden.INSTANCES[type_name])
    assert lkf.bits_equal(as_dict(type_name, instance), make_golden.INSTANCES[type_name]), "robot_msgs changed a value"


# ---------------------------------------------------------------- like_keras (public API only)
# The emitting component, `emitter`, is in kpi/golden/instances.py (no pyarrow there; ruler-002 §3).


@pytest.mark.parametrize("type_name", TYPES)
def test_like_keras_encodes_golden(type_name, tmp_path):
    """Recording what a component sent (Python encode) gives the golden bytes, bit for bit."""
    lk = like_keras()
    s = lk.System("golden")
    s.add(emitter(lk, type_name)(), "emit")
    dep = lk.Deployment(s, realtime=False)
    dep.place(s.emit, hz=10)
    dep.record(tmp_path / "out.mcap", s.emit.value)
    dep.start(until=0.25)
    dep.stop()
    frames = [message.data for message in mcap.read(tmp_path / "out.mcap").on("value")]
    assert frames == [make_golden.golden_path(type_name).read_bytes()]


@pytest.mark.parametrize("type_name", TYPES)
def test_like_keras_decodes_golden(type_name, tmp_path):
    """lk.replay of a recording holding the golden frame delivers the instance (decode), and recording it again
    gives the same bytes (no field, bit or NaN payload lost on the way)."""
    lk = like_keras()
    cls = __import__("robot_msgs").message_types()[type_name]
    source = tmp_path / "golden.mcap"
    mcap.write(source, [mcap.Schema(1, type_name, "arrow-c-format", lkf.schema_preimage(type_name).encode())],
               [mcap.Channel(1, 1, "golden.emit.value", "lkf")],
               [mcap.Message(1, 0, 0, 0, make_golden.golden_path(type_name).read_bytes())])
    received = []

    class Collect(lk.Component):
        value = lk.In(cls, queue=10)

        def forward(self, value):
            received.append(value)

    s = lk.System("decode")
    replay = s.add(lk.replay(source), "replay")
    s.add(Collect(), "collect", value=replay.emit__value)
    dep = lk.Deployment(s, realtime=False)
    dep.place(s.replay, hz=10)
    dep.place(s.collect)
    dep.record(tmp_path / "again.mcap", replay.emit__value)
    dep.start(until=0.25)
    dep.stop()
    assert len(received) == 1
    assert lkf.bits_equal(as_dict(type_name, received[0]), make_golden.INSTANCES[type_name])
    assert [m.data for m in mcap.read(tmp_path / "again.mcap").on("value")] == [make_golden.golden_path(type_name).read_bytes()]


@pytest.mark.parametrize("type_name", TYPES)
def test_schema_hash_in_describe(type_name):
    """describe() prints each port's schema hash, the same 64 bits as the frame's (shown as %016x)."""
    lk = like_keras()
    s = lk.System("hashes")
    s.add(emitter(lk, type_name)(), "emit")
    dep = lk.Deployment(s)
    dep.place(s.emit, hz=10)
    ports = dep.describe()["components"]["hashes.emit"]["ports"]
    assert ports["value"]["type"] == type_name
    assert ports["value"]["hash"] == f"{lkf.schema_hash(type_name):016x}"


def test_recorded_helper_reads_goldens(tmp_path):
    """The ruler's MCAP writer and reader agree (so a failure above is like_keras's, not the ruler's)."""
    path = tmp_path / "self.mcap"
    frames = [make_golden.golden_path(name).read_bytes() for name in TYPES]
    mcap.write(path, [mcap.Schema(i + 1, name, "arrow-c-format", lkf.schema_preimage(name).encode()) for i, name in enumerate(TYPES)],
               [mcap.Channel(i + 1, i + 1, f"g.{name.replace('/', '_')}", "lkf") for i, name in enumerate(TYPES)],
               [mcap.Message(i + 1, 0, 1, 2, frame) for i, frame in enumerate(frames)])
    for name in TYPES:
        decoded = recorded(path, name.replace("/", "_"))
        assert len(decoded) == 1 and lkf.bits_equal({k: v for k, v in decoded[0].items() if not k.startswith("_")},
                                                     make_golden.INSTANCES[name])
