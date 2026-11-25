"""P5, roles and bindings (proposals/api/P5-roles-and-bindings.md): a System wires roles (components with ports and
no forward); `lk.Deployment(system, bind={...})` chooses what runs in their place. Additive: without `bind=` a
deployment, its checks and its datasheet are v0's byte for byte, and every v0 component binds to itself.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_p5_bind.py
"""

from __future__ import annotations

import pathlib

import pytest

import like_keras as lk
from like_keras.record import read
from robot_msgs.std import Header

BUILD = pathlib.Path(__file__).resolve().parents[1] / "build" / "kpi"


class Source(lk.Component):
    """A role: where values come from."""

    value = lk.Out(Header)


class Clock(Source):
    def forward(self):
        return Header(stamp=self.now())


class Fixed(Source):
    """A test double for Source."""

    def forward(self):
        return Header(stamp=42.0)


class Mirror(lk.Component):
    """Not a subclass of Source, but the same ports: an implementation is judged by its ports, as a .so is."""

    value = lk.Out(Header)

    def forward(self):
        return Header(stamp=100.0 + self.now())


class Needy(Source):
    """The role's ports plus an input the node has not got: nothing could feed it."""

    extra = lk.In(Header)

    def forward(self, extra):
        return extra


class Sink(lk.Component):
    value = lk.In(Header)
    out = lk.Out(Header)

    def forward(self, value):
        return value


def cell() -> lk.System:
    with lk.System("cell") as s:
        source, sink = Source(), Sink()
        sink(value=source.value)
        s.expose(sink.out)
    return s


def boxed() -> lk.System:
    """cell with the sink inside a box: a node path can bind the whole box."""
    with lk.System("cell") as s:
        source = Source()
        with lk.System("box") as box:
            sink = Sink()
            box.expose(sink.value, sink.out)
        box(value=source.value)
        s.expose(box.out)
    return s


def stamps(dep: lk.Deployment, path: pathlib.Path, until: float = 0.5) -> list[float]:
    from like_keras import codec

    dep.record(path)
    dep.start(until=until)
    dep.stop()
    channels, messages = read(path)
    return [codec.decode(frame, Header).stamp for channel, _, frame in messages if channels[channel][0] == "cell.out"]


def deployed(system: lk.System, bind: dict | None = None) -> lk.Deployment:
    dep = lk.Deployment(system, realtime=False, bind=bind)
    dep.place(system, hz=10)
    return dep


def test_an_unbound_role_is_refused_before_start_naming_the_node_and_the_fix():
    with pytest.raises(ValueError, match=r"cell.source is a role \(Source: ports and no forward.*Fix: bind an "
                                         r"implementation, Deployment\(..., bind=\{Source: <an implementation>\}\) "
                                         r'or bind=\{"source": ...\}'):
        deployed(cell()).describe()


def test_a_class_key_binds_every_node_of_that_role_and_the_system_reads_the_bound_one(tmp_path):
    system = cell()
    role = system.source
    dep = deployed(system, {Source: Clock()})
    assert isinstance(system.source, Clock) and system.source is not role
    assert stamps(dep, tmp_path / "run.mcap")[:3] == [0.0, 0.1, 0.2]
    sheet = dep.describe()["components"]
    assert sheet["cell.source"]["impl"] == f"{__name__}:Clock"
    assert sheet["cell.source"]["bound_by"] == "bind[Source]" and sheet["cell.source"]["role"] == f"{__name__}:Source"
    assert sheet["cell.sink"]["bound_by"] == "default" and "role" not in sheet["cell.sink"]


def test_sim_real_and_a_test_double_are_each_one_binding_line(tmp_path):
    sim = {Source: Clock()}
    runs = {
        "sim": deployed(cell(), sim),
        "double": deployed(cell(), {**sim, Source: Fixed()}),
        "by_ports": deployed(cell(), {**sim, "source": Mirror()}),  # a node path wins over the class key
    }
    got = {name: stamps(dep, tmp_path / f"{name}.mcap")[:2] for name, dep in runs.items()}
    assert got == {"sim": [0.0, 0.1], "double": [42.0, 42.0], "by_ports": [100.0, 100.1]}


def test_a_profile_is_a_template_each_deployment_builds_its_own_instance():
    profile = {Source: Clock()}
    first, second = cell(), cell()
    deployed(first, profile).describe()
    deployed(second, profile).describe()
    assert first.source is not second.source and profile[Source] not in (first.source, second.source)
    assert profile[Source]._lk.parent is None, "the template itself is never placed"


def test_a_node_path_binds_a_whole_box_and_bindings_resolve_before_the_plan(tmp_path):
    system = boxed()
    dep = deployed(system, {Source: Clock(), "box": Sink()})  # the box (one leaf inside) -> one leaf
    sheet = dep.describe()
    assert sorted(sheet["components"]) == ["cell.box", "cell.source"]
    assert sheet["components"]["cell.box"]["role"] == "System box" and sheet["edges"][0]["dst"] == "cell.box.value"
    assert stamps(dep, tmp_path / "run.mcap")[:2] == [0.0, 0.1]


def test_the_same_system_rebinds_per_deployment():
    system = cell()
    sim, double = deployed(system, {Source: Clock()}), deployed(system, {Source: Fixed()})
    assert isinstance(system.source, Fixed)
    sim.describe()  # each deployment puts its own choice back before it plans
    assert isinstance(system.source, Clock)
    assert double.describe()["components"]["cell.source"]["impl"].endswith(":Fixed")


def test_placing_or_recording_the_role_itself_means_what_runs_there(tmp_path):
    system = cell()
    role, sink = system.source, system.sink
    dep = lk.Deployment(system, realtime=False, bind={Source: Clock()})
    dep.place(role, hz=10)  # the role captured before the deployment: what runs there is placed
    dep.place(sink, hz=10)
    dep.record(tmp_path / "run.mcap", role.value)
    dep.start(until=0.3)
    dep.stop()
    channels, messages = read(tmp_path / "run.mcap")
    assert {topic for topic, _ in channels.values()} == {"cell.source.value"} and len(messages) >= 3


@pytest.mark.parametrize(
    ("bind", "expected"),
    [
        ({"sourse": Clock()}, r"bind\['sourse'\] \(Deployment at test_p5_bind.py:\d+\) names no node of cell; its nodes: "
                              r"sink, source"),
        ({Sink: Sink(), Source: Clock(), Header: Clock()}, r"bind key <class '.*Header'> .* is neither a node path nor"),
        ({Source: Clock(), Clock: Clock()}, r"bind\[Clock\] .* matches no node of cell"),
        ({Source: Clock}, r"bind\[Source\] = a type .* is not a component"),
        ({Source: Sink()}, r"bind\[Source\] = .*:Sink .* does not fit source: value is In\[std/Header\], the node's is "
                           r"Out\[std/Header\]"),
        ({Source: Needy()}, r"does not fit source: its input 'extra' \(In\[std/Header\]\) is not a port of the node"),
    ],
    ids=["unknown-path", "bad-key", "unmatched-class", "not-a-component", "ports-differ", "extra-input"],
)
def test_a_wrong_binding_is_refused_before_start_with_the_deployments_line(bind, expected):
    with pytest.raises(ValueError, match=expected):
        deployed(cell(), bind).describe()


def test_without_bind_the_datasheet_is_v0s():
    def plain() -> lk.System:
        s = lk.System("cell")
        clock = s.add(Clock(), "source")
        s.add(Sink(), "sink", value=clock.value)
        return s

    v0, explicit_none = deployed(plain()).describe(), deployed(plain(), None).describe()
    assert v0 == explicit_none and "bound_by" not in v0["components"]["cell.source"]
    rebound = deployed(plain(), {}).describe()  # bind={} binds nothing; the sheet says so per component
    assert rebound["components"]["cell.source"]["bound_by"] == "default"


def test_a_class_key_inside_a_box_bound_by_path_is_moot_not_unmatched():
    with lk.System("cell") as s:
        with lk.System("box") as box:
            source = Source()
            box.expose(source.value)
        sink = Sink()
        sink(value=box.value)
        s.expose(sink.out)
    dep = deployed(s, {Source: Clock(), "box": Fixed()})  # REAL = {**SIM, "arm": the .so}: SIM's keys still listed
    assert dep.describe()["components"]["cell.box"]["bound_by"] == 'bind["box"]'


@pytest.mark.skipif(not (BUILD / "libarm_controller.so").exists(), reason="build/kpi/libarm_controller.so not built")
def test_python_to_cpp_is_one_binding_line_checked_by_schema_hash():
    from kpi.tasks.arm_cell.arm_controller_py import JtcPy

    from robot_msgs.control import ServoCommand, ServoState
    from robot_msgs.trajectory import JointTrajectory

    class ArmBox(lk.Component):
        trajectory = lk.In(JointTrajectory)
        state = lk.Out(ServoState)
        command = lk.Out(ServoCommand)

    class Hold(lk.Component):
        state = lk.In(ServoState)
        trajectory = lk.Out(JointTrajectory)

        def forward(self, state):
            return None

    with lk.System("cell") as s:
        arm, planner = ArmBox(), Hold()
        planner(state=arm.state)
        arm(trajectory=planner.trajectory)
    dep = lk.Deployment(s, bind={"arm": lk.native(BUILD / "libarm_controller.so")})
    dep.place(s.arm, process="controller", hz=100)
    dep.place(s.planner, hz=10)
    entry = dep.describe()["components"]["cell.arm"]
    assert entry["language"] == "native" and entry["bound_by"] == 'bind["arm"]' and entry["role"].endswith("ArmBox")
    with pytest.raises(ValueError, match=r"does not fit arm: state is In\[control/ServoState\], the node's is Out"):
        lk.Deployment(s, bind={"arm": JtcPy()}).describe()
