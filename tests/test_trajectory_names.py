"""Lane core's test of CONTRACT §15 A16: the generated C++ JointTrajectory view carries the plan's joint names
(`joints` + `joint_names`, string views into the received buffer, valid for the step), in one process and across
processes. kpi/polyglot's frozen test does the same with the golden plan, but its source is a class built inside a
function, which dora cannot rebuild in another process (CONTRACT §4.1; docs/friction/ruler-008.md); this source is
module level, so the dora half runs here.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_trajectory_names.py

The dora case needs a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites); elsewhere it skips.
"""

from __future__ import annotations

import json
import subprocess
import sys

import numpy as np
import pytest

import like_keras as lk
from kpi.polyglot.friction_parts import JointStateLog
from kpi.support.lk import REPO, backend, feature_library
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory

# WHY a 2-byte and a 3-byte UTF-8 name, out of config order: the view must hand back the bytes as sent, by column
NAMES = ("épaule", "coude", "手首")
POSITIONS = np.array([[0.0, 1.0, 2.0], [0.5, 1.5, 2.5]])


class NamedPlan(lk.Component):
    """Sends one plan with NAMES on its second step, nothing on the others."""

    trajectory = lk.Out(JointTrajectory)

    def forward(self):
        self._steps = getattr(self, "_steps", 0) + 1
        if self._steps != 2:
            return None
        return JointTrajectory(t=np.array([0.0, 0.1]), joint_names=NAMES, positions=POSITIONS,
                               header=Header(stamp=self.now()))


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_the_view_names_each_column(backend_name, tmp_path, request):
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    s = lk.System(f"named_plan_{backend_name}")
    source = s.add(NamedPlan(), "source")
    box = s.add(lk.native(feature_library("joint_names"), config={"unused": 0}), "box", trajectory=source.trajectory)
    s.add(JointStateLog(path=str(tmp_path / "names.jsonl")), "sink", value=box.names)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.source, process="source", hz=10)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    try:
        dep.start(until=0.45)
    finally:
        dep.stop()
    found = [json.loads(line) for line in open(tmp_path / "names.jsonl", encoding="utf-8")]
    assert [row["name"] for row in found] == [list(NAMES)], found
    assert found[0]["position"] == POSITIONS[-1].tolist(), found


def test_the_committed_views_are_current():
    """CMake never runs Python (CONTRACT §7.4): the committed headers must be what the generator writes now."""
    checked = subprocess.run([sys.executable, "-m", "like_keras.codec", "--cpp", "--check"], cwd=REPO,
                             capture_output=True, text=True, timeout=60)
    assert checked.returncode == 0, checked.stdout + checked.stderr
