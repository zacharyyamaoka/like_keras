"""Lane dora's test of the real-time stop order under CONTRACT §15 A37 (with A28 and A30): which nodes a dora
stop() signals before `dora stop`, and in which waves. The frozen ruler tests (facilities::
test_dora_stop_drains_to_live_readers and its Python twin) cover one box and its readers; this covers the rule on
the shapes they do not: a chain of Python nodes, the reference cell's cycle (arm and planner), readers the gateway
alone reads, and a node on another host.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_dora_stop_order.py
"""

from __future__ import annotations

from like_keras.backends.dora.emit import GATEWAY, stop_order


def node(role: str, *sources: str, host: str | None = None) -> dict:
    """A plan document's node: its role and the `writer/output` of each input."""
    return {
        "role": role,
        "host": host,
        "inputs": {f"in{index}": {"source": source} for index, source in enumerate(sources)},
    }


def test_upstream_first_and_nodes_only_the_gateway_reads_are_left_to_dora_stop():
    nodes = {
        "box": node("python"),  # A37's box: its log and health go to the readers and the recording
        "readers": node("python", "box/x.box.log", "box/x.box.health"),
        GATEWAY: node("gateway", "box/x.box.log", "readers/x.log_reader.health"),
    }
    assert stop_order({"nodes": nodes}) == [["box"]]


def test_a_chain_stops_from_its_source_down():
    nodes = {
        "c": node("python", "b/x.b.value"),
        "b": node("python", "a/x.a.value"),
        "a": node("native"),
        GATEWAY: node("gateway", "a/x.a.health", "b/x.b.health", "c/x.c.health"),
    }
    assert stop_order({"nodes": nodes}) == [["a"], ["b"]]


def test_in_a_cycle_the_native_node_goes_first():
    nodes = {  # the reference cell's target placement: the arm and the planner read each other
        "arm-controller": node("native", "main/x.planner.trajectory"),
        "main": node("python", "arm-controller/x.arm.state"),
        "log": node("python", "arm-controller/x.arm.log", "main/x.planner.log"),
        GATEWAY: node("gateway", "arm-controller/x.arm.state"),
    }
    assert stop_order({"nodes": nodes}) == [["arm-controller"], ["main"]]


def test_a_node_on_another_host_is_never_signalled_from_here():
    nodes = {
        "remote": node("native", host="robot-2"),
        "local": node("python", "remote/x.r.value"),
        "sink": node("python", "local/x.l.value"),
    }
    assert stop_order({"nodes": nodes}) == [["local"]]
