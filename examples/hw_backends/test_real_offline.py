"""RealBackend lifecycle tests with synthetic CAN replies and a temporary lock."""
import asyncio
import fcntl
import sys
from types import ModuleType, SimpleNamespace

import pytest
from robot_msgs.control import BackendUnavailable

from examples.hw_backends.backends import RealBackend, RealConfig, hold
from examples.hw_backends.description import TABLE
from examples.pendulum import bus

REGISTERS = dict(POSITION=0.05, VELOCITY=0.0, TORQUE=0.0, TEMPERATURE=25.0, FAULT=0)


@pytest.fixture
def rack(tmp_path, monkeypatch):
    fake = SimpleNamespace(lock=tmp_path / "adapter.lock", transports=[], response=REGISTERS.copy(), fail=None,
                           with_reader=False)
    monkeypatch.setattr(bus, "RACK_LOCK", str(fake.lock))
    module = ModuleType("moteus")
    module.Register = SimpleNamespace(**{name: name for name in REGISTERS})

    class Transport:
        def __init__(self, port):
            self.loop = asyncio.get_running_loop()
            self.commands = []
            self.closed = False
            self.reader = self.loop.create_task(asyncio.Event().wait()) if fake.with_reader else None
            fake.transports.append(self)

        async def cycle(self, commands):
            self.commands.extend(commands)
            if fake.fail in commands:
                raise TimeoutError("synthetic CAN failure")
            return [] if fake.response is None else [SimpleNamespace(values=fake.response)]

        def close(self):
            self.closed = True

    class Controller:
        def __init__(self, *, id, transport):
            self.transport = transport

        def make_query(self):
            return "query"

        def make_position(self, **kwargs):
            return "position"

        async def set_stop(self):
            await self.transport.cycle(["stop"])

    module.Fdcanusb = Transport
    module.Controller = Controller
    monkeypatch.setitem(sys.modules, "moteus", module)
    return fake


def assert_released(rack):
    transport = rack.transports[-1]
    assert transport.closed and transport.loop.is_closed()
    with rack.lock.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)


@pytest.mark.parametrize(("reply", "error"), [
    (None, "POSITION"),
    ({key: value for key, value in REGISTERS.items() if key != "FAULT"}, "FAULT"),
    (REGISTERS | {"POSITION": float("nan")}, "POSITION"),
    (REGISTERS | {"FAULT": 0.5}, "FAULT"),
    (REGISTERS | {"FAULT": 3}, "fault 3"),
])
def test_invalid_opening_reply_stops_before_commands(rack, reply, error):
    rack.response = reply
    with pytest.raises(BackendUnavailable, match=error):
        RealBackend(TABLE, RealConfig())
    assert rack.transports[-1].commands == ["query"]
    assert_released(rack)


def test_query_failure_releases_lock_and_transport(rack):
    rack.fail = "query"
    with pytest.raises(TimeoutError, match="synthetic CAN failure"):
        RealBackend(TABLE, RealConfig())
    assert_released(rack)
    rack.fail = None
    backend = RealBackend(TABLE, RealConfig())
    backend.close(hold=False)
    assert_released(rack)


def test_initial_controller_creation_failure_closes_open_transport(rack, monkeypatch):
    def fail_controller(**kwargs):
        raise RuntimeError("synthetic controller setup failure")

    monkeypatch.setattr(sys.modules["moteus"], "Controller", fail_controller)
    with pytest.raises(RuntimeError, match="synthetic controller setup failure"):
        RealBackend(TABLE, RealConfig())
    assert_released(rack)


@pytest.mark.parametrize("operation", ["read", "write"])
def test_runtime_can_failure_releases_resources(rack, operation):
    backend = RealBackend(TABLE, RealConfig())
    command = hold(backend.read(), 0.0)
    rack.fail = {"read": "query", "write": "position"}[operation]
    with pytest.raises(TimeoutError, match="synthetic CAN failure"):
        if operation == "read":
            backend.read()
        else:
            backend.write(command)
    assert_released(rack)
    backend.close()
    rack.fail = None
    replacement = RealBackend(TABLE, RealConfig())
    replacement.close(hold=False)
    assert_released(rack)


def test_runtime_missing_register_releases_resources(rack):
    backend = RealBackend(TABLE, RealConfig())
    rack.response.pop("TEMPERATURE")
    with pytest.raises(BackendUnavailable, match="TEMPERATURE"):
        backend.read()
    assert_released(rack)


@pytest.mark.parametrize("stop_fails", [False, True])
def test_close_is_idempotent_and_releases_on_stop_failure(rack, stop_fails):
    asyncio.set_event_loop(None)
    backend = RealBackend(TABLE, RealConfig())
    if stop_fails:
        rack.fail = "stop"
        with pytest.raises(TimeoutError, match="synthetic CAN failure"):
            backend.close(hold=False)
    else:
        backend.close(hold=False)
    backend.close(hold=False)
    assert rack.transports[-1].commands == ["query", "stop"]
    assert_released(rack)


def test_close_cancels_and_drains_the_reader(rack):
    rack.with_reader = True
    backend = RealBackend(TABLE, RealConfig())
    reader = rack.transports[-1].reader
    assert not reader.done()
    backend.close(hold=False)
    assert reader.cancelled()
    assert_released(rack)
