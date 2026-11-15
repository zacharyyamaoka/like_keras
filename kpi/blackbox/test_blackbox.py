"""Criterion 3's dynamic half (rule 3): each reference component steps on recorded inputs with dora, pyarrow and
like_keras's dora backend blocked from import, and matches the recorded outputs: the framework under it can go.

Each component runs in a fresh interpreter whose sys.meta_path refuses those modules. The inproc backend is part of
the core (CONTRACT §4.4's default argument), so it is not blocked. Also here: the static scan (kpi/blackbox.py) and
criterion 10's counter-metric, no wall-clock sleep in kpi/twin.
"""

from __future__ import annotations

import json
import subprocess
import sys
import textwrap

import pytest

from kpi.support.lk import KPI, REPO, like_keras, subprocess_env

BLOCKER = textwrap.dedent('''
    import importlib.abc, sys
    BLOCKED = ("dora", "pyarrow", "like_keras.backends.dora")
    class Blocker(importlib.abc.MetaPathFinder):
        def find_spec(self, name, path=None, target=None):
            if any(name == blocked or name.startswith(blocked + ".") for blocked in BLOCKED):
                raise ImportError(f"{name} is blocked: a component must step without a transport")
            return None
    sys.meta_path.insert(0, Blocker())
''')

COMMON = textwrap.dedent('''
    import json, tempfile, types
    from pathlib import Path
    import numpy as np
    from robot_msgs.control import ServoCommand, ServoState
    from robot_msgs.std import Header
    from kpi.tasks.arm_cell.plan import JOINTS, TARGETS, PickTarget, plan_pick, starting_at
    TARGET = PickTarget(name=JOINTS, position=json.loads(TARGETS.read_text().splitlines()[0])["position"])
    STATE = ServoState(name=JOINTS, position=np.linspace(-0.3, 0.3, 7), velocity=np.zeros(7),
                       fault=np.zeros(7, dtype=np.int64), seq=5, header=Header(stamp=0.0))
    def same(a, b):
        return np.asarray(a).tobytes() == np.asarray(b).tobytes()
''')

STEPS = {
    "arm_cell.PickPlanner": '''
        from kpi.tasks.arm_cell.planner import PickPlanner
        plan = PickPlanner()(target=TARGET, state=STATE)
        expected = starting_at(plan_pick(TARGET, start=STATE.joint_state()), 0.05)
        assert same(plan.positions, expected.positions) and plan.header.stamp == expected.header.stamp
    ''',
    "arm_cell.TargetFile": '''
        from kpi.tasks.arm_cell.targets import TargetFile
        source = TargetFile()
        source.activate()
        first = source()
        assert same(first.position, TARGET.position) and first.name == JOINTS
    ''',
    "arm_cell.MoteusBusPy": '''
        from kpi.tasks.arm_cell.arm_controller_py import MoteusBusPy
        with tempfile.TemporaryDirectory() as scratch:
            bus = MoteusBusPy(twin=True, exchange_s=0.0, lock_path=str(Path(scratch) / "lock"))
            bus.activate()
            state = bus(command=None)
            assert same(state.position, np.zeros(7)) and state.seq == 1
            bus.deactivate()
    ''',
    "arm_cell.JtcPy": '''
        from kpi.tasks.arm_cell.arm_controller_py import JtcPy
        from kpi.tasks.arm_cell.jtc_law import JtcCore
        jtc = JtcPy()
        jtc.activate()
        command = jtc(trajectory=None, state=STATE)
        core = JtcCore(); core.hold_here(STATE)
        assert same(command.position, core.sample(0.0, STATE)[0]) and command.state_seq == 5
    ''',
    "arm_cell.GuardPy": '''
        from kpi.tasks.arm_cell.arm_controller_py import GuardPy
        guard = GuardPy()
        guard.activate()
        proposed = ServoCommand(name=JOINTS, position=STATE.position + 0.01, velocity=np.zeros(7),
                                feedforward_effort=np.zeros(7))
        safe = guard(state=STATE, proposed=proposed)
        assert same(safe.position, proposed.position) and same(safe.max_effort, np.full(7, 2.0))
        runaway = guard(state=STATE, proposed=proposed.replace(position=STATE.position + 1.0))
        assert same(runaway.position, STATE.position), "a runaway must hold at the measured position"
    ''',
    "empty.Empty": '''
        from kpi.tasks.empty.empty import Empty
        assert Empty(gain=2.0)() is None
    ''',
    "empty.Talker": '''
        from kpi.tasks.empty.empty import Talker
        assert Talker()() is None
    ''',
    "arm_cell.ArmControllerPy": '''
        from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy
        from kpi.tasks.arm_cell.cell import CONFIG
        system = ArmControllerPy(config=CONFIG)
        assert {"trajectory", "state", "command"} <= set(dir(system))
    ''',
    "relay.Count": '''
        from kpi.tasks.relay.solution.relay import Count
        count = Count()
        count.activate()
        assert same(count().position, [1.0]) and same(count().position, [2.0])
    ''',
    "relay.Double": '''
        from kpi.tasks.relay.solution.relay import Double
        from robot_msgs.sensor import JointState
        assert same(Double()(value=JointState(name=("a",), position=[3.0])).position, [6.0])
    ''',
    "relay.Save": '''
        from kpi.tasks.relay.solution.relay import Save
        from robot_msgs.sensor import JointState
        with tempfile.TemporaryDirectory() as scratch:
            path = Path(scratch) / "out.txt"
            Save(path=str(path))(value=JointState(name=("a",), position=[6.0]))
            assert path.read_text() == "6.0\\n"
    ''',
    "pipeline.Count": '''
        from kpi.tasks.pipeline.solution.pipeline import Count
        count = Count()
        count.activate()
        assert same(count().position, [1.0]) and same(count().position, [2.0])
    ''',
    "pipeline.Double": '''
        from kpi.tasks.pipeline.solution.pipeline import Double
        from robot_msgs.sensor import JointState
        assert same(Double()(value=JointState(name=("a",), position=[3.0])).position, [6.0])
    ''',
    "pipeline.Save": '''
        from kpi.tasks.pipeline.solution.pipeline import Save
        from robot_msgs.sensor import JointState
        with tempfile.TemporaryDirectory() as scratch:
            path = Path(scratch) / "out.txt"
            Save(path=str(path))(value=JointState(name=("a",), position=[6.0]))
            assert path.read_text() == "6.0\\n"
    ''',
}


@pytest.mark.parametrize("component", sorted(STEPS))
def test_steps_with_transports_blocked(component):
    like_keras()
    script = BLOCKER + COMMON + textwrap.dedent(STEPS[component]) + textwrap.dedent('''
        leaked = sorted(name for name in sys.modules if name.split(".")[0] in ("dora", "pyarrow"))
        assert not leaked, f"loaded {leaked}"
        print("ok")
    ''')
    result = subprocess.run([sys.executable, "-c", script], cwd=REPO, env=subprocess_env(), capture_output=True,
                            text=True, timeout=120)
    assert result.returncode == 0 and result.stdout.strip().endswith("ok"), result.stderr[-3000:]


def test_static_rules_clean():
    """kpi/blackbox.py: rules 1, 2, 4, 5 on every reference component; no env read in the library outside backends."""
    result = subprocess.run([sys.executable, str(KPI / "blackbox.py")], capture_output=True, text=True, timeout=120)
    report = json.loads(result.stdout)
    dirty = [f"{c['file']}:{c['name']}: {c['violations']}" for c in report["components"] if c["violations"]]
    assert not dirty and not report["library_env_reads"], dirty + report["library_env_reads"]


def test_no_wall_clock_sleep_in_twin_tests():
    """Criterion 10's counter-metric: a twin run never needs a wall-clock sleep."""
    result = subprocess.run([sys.executable, str(KPI / "blackbox.py")], capture_output=True, text=True, timeout=120)
    assert json.loads(result.stdout)["twin_sleeps"] == []
