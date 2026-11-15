"""like_keras: the communication API for BAM robots (docs/CONTRACT.md).

Components are black boxes with typed ports; a System wires them from outside; a Deployment places them on threads,
processes and hosts; the backend underneath (inproc today, others beside it) can churn without a component changing.

    import like_keras as lk

    class PickPlanner(lk.Component):
        target = lk.In(PickTarget, queue=8)
        state = lk.In(ServoState, stale=0.1)
        trajectory = lk.Out(JointTrajectory)

        def forward(self, target, state): ...

    s = lk.System("pick_cell")
    planner = s.add(PickPlanner(), target=targets.target, state=arm.state)
    dep = lk.Deployment(s)
    dep.place(s.pick_planner)
    dep.start()

Importing it loads the standard library only; robot_msgs (with numpy) loads when a plan is built.
"""

from like_keras.component import Component, In, Out, native
from like_keras.deployment import VERSION as __version__  # noqa: F401  (the package version)
from like_keras.deployment import Deployment
from like_keras.record import replay
from like_keras.system import System

try:  # WHY here: importing the attach module rebinds the package attribute `attach`; the function must win
    from like_keras.attach import attach
except ModuleNotFoundError as missing:
    if missing.name != "like_keras.attach":
        raise

    def attach(address: str, lease_s: float = 0.25) -> Component:
        """A component mirroring a running deployment's exposed ports, reached at `address` (CONTRACT §4.5)."""
        raise NotImplementedError("lk.attach lands with the attach protocol (CONTRACT §9.3, milestone M3)")


__all__ = ["Component", "In", "Out", "System", "Deployment", "native", "attach", "replay"]
