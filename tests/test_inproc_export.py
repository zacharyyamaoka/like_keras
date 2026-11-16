"""Lane cpp's own test of what inproc hands a native box (CONTRACT §6.3, §7.1): `_export`'s cache of constant
subtrees gives the Arrow arrays the plain walk gives, node for node and byte for byte.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests

WHY (fix round 1, M2): the walk rebuilt the same null subtrees (limits, channels, accelerations, effort) for every
plan, 26 of a 7 x 1000 plan's 42 nodes, on the planner's thread, where A2 and T2 allow 0.1 ms per push.
"""

from __future__ import annotations

import ctypes

import numpy as np
import pytest

from like_keras import codec
from like_keras.backends import inproc

TYPES = ["std/Header", "std/LogRecord", "std/DiagnosticRecord", "std/Metrics", "sensor/JointState",
         "control/ServoState", "control/ServoCommand", "trajectory/JointTrajectory"]  # fmt: skip


def plan(**fields):
    """A2's plan: 7 joints x 1000 samples, positions and velocities, the rest left at its default."""
    from robot_msgs.std import Header
    from robot_msgs.trajectory import JointTrajectory

    t = np.linspace(0.0, 2.0, 1000)
    return JointTrajectory(
        t=t,
        joint_names=tuple(f"joint_{index}" for index in range(7)),
        positions=np.outer(t, np.arange(7.0)),
        velocities=np.outer(np.ones_like(t), np.arange(7.0)),
        header=Header(stamp=1.0, frame_id="base", meta={"returned_ns": "123456789"}),
        **fields,
    )


def messages():
    from kpi.golden import make_golden
    from kpi.golden.instances import build

    golden = [build(name, make_golden.INSTANCES[name]) for name in TYPES]
    return [*golden, plan(), plan(accelerations=np.zeros((1000, 7))), plan(channels={"gain": np.ones(1000)})]


def described(address: int, node) -> list:
    """Every node of an exported array, depth first: its counts and which of its buffer pointers are null."""
    array = inproc._arrow().Array.from_address(address)
    buffers = (ctypes.c_void_p * array.n_buffers).from_address(array.buffers)
    head = [(array.length, array.null_count, array.offset, array.n_children, tuple(bool(at) for at in buffers))]
    children = (ctypes.c_void_p * array.n_children).from_address(array.children) if array.n_children else ()
    return head + [row for child, at in zip(node.children, children, strict=True) for row in described(at, child)]


def exported(value) -> tuple[list, list[bytes]]:
    """What lk_push would receive: the array's shape and its buffers' bytes as the box's decoder reads them."""
    arrow, root = inproc._arrow(), inproc._export(value)
    node = codec.Node("", "+s", children=codec.schema(type(value)), message=type(value))
    parts: list[bytes] = []
    try:
        inproc._parts(node, arrow.Array.from_address(root), parts, arrow, ctypes)
        return described(root, node), parts
    finally:
        arrow.caller(arrow.Array.from_address(root).release)(root)


def walked(value, monkeypatch) -> tuple[list, list[bytes]]:
    """The same, through the plain walk only (every _nodes call goes to _walk, no cache)."""
    with monkeypatch.context() as patched:
        patched.setattr(inproc, "_nodes", inproc._walk)
        return exported(value)


@pytest.mark.parametrize("index", range(11))
def test_cached_export_is_the_walk(index, monkeypatch):
    value = messages()[index]
    reference = walked(value, monkeypatch)
    assert exported(value) == reference  # a cache miss
    assert exported(value) == reference  # a cache hit


def test_a_plan_walks_only_what_it_carries(monkeypatch):
    """Warm, a 7 x 1000 plan walks only the fields that carry values; limits, channels, accelerations and effort
    (26 of its 42 nodes) come from the cache."""
    walk, calls = inproc._walk, {"cached": [], "walked": []}

    def counting(into: str):
        def counted(*arguments):
            calls[into].append(arguments[0].name)
            return walk(*arguments)

        return counted

    exported(plan())
    with monkeypatch.context() as patched:
        patched.setattr(inproc, "_walk", counting("cached"))
        exported(plan())
    with monkeypatch.context() as patched:
        patched.setattr(inproc, "_walk", counting("walked"))
        patched.setattr(inproc, "_nodes", inproc._walk)
        exported(plan())
    skipped = {"limits", "channels", "accelerations", "effort"}
    assert skipped <= set(calls["walked"]) and not skipped & set(calls["cached"]), calls
    assert len(calls["cached"]) < len(calls["walked"]) / 2, calls


def test_a_field_none_then_empty_then_none_exports_as_its_own_walk_each_time(monkeypatch):
    """The cache key holds WHICH values are None: a field None in one message and empty in the next is two different
    arrays (a null versus a valid empty list), never one cached subtree. WHY (M2 verifier, round 2): a key of
    (id(node), len(values)) passed every other case here, and handed the second message the first one's null."""
    from robot_msgs.sensor import JointState

    monkeypatch.setattr(inproc, "_CONSTANT", {})  # cold, so the None pattern is cached first
    for velocity in (None, np.zeros(0), None, np.zeros(0)):
        value = JointState(name=(), position=np.zeros(0), velocity=velocity)
        assert exported(value) == walked(value, monkeypatch), velocity
