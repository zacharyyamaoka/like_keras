# ruler-008: A16's dora case places a source the dora backend can never rebuild (lane core, A16-A28 library work, 2026-11-16)

`kpi/polyglot/test_polyglot_friction.py::test_T10_trajectory_joint_names[dora]` cannot pass on any library that
keeps CONTRACT §4.1. Its `inproc` case passes once A16 lands.

## What fails

The test places the golden emitter in a process of its own under dora:

```python
emit = s.add(emitter(lk, "trajectory/JointTrajectory")(), "emit")   # kpi/golden/instances.py: class Emit inside emitter()
...
dep.place(s.emit, process="emit", hz=10)
```

`Emit` is defined inside `emitter()`, so its `__qualname__` is `emitter.<locals>.Emit`. Under dora every process is
a node that rebuilds its components as `module:qualname(**params)` (§4.1: "Closures and lambdas cannot cross a
process; `describe()` refuses them"; §9.2). So `describe()` refuses the plan before anything starts:

```
ValueError: plan_names_dora: 1 problem(s) found before start (CONTRACT §4.3):
- plan_names_dora.emit (added at test_polyglot_friction.py:51) cannot be rebuilt in another process. Fix: define its class at module level and give it JSON-able Params
```

On the library at `c6ab053` it failed earlier, at the build of `joint_names.cpp` (no `joint_names` member), so
M0.3's "fails before the feature" check could not see this second failure behind the first.

Reproduce (device-free sandbox, private namespace):

```bash
cd ~/like_keras && env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/run.py polyglot/test_polyglot_friction.py -k joint_names
```

## What lane core did instead

`tests/test_trajectory_names.py` runs the same check in one process and across processes under a private dora, with a
module-level source (`NamedPlan`) that sends a plan with a 2-byte and a 3-byte UTF-8 name out of config order. Both
pass, so the A16 view decodes names over dora too.

## Proposal

Give the dora case a source dora can rebuild: a module-level part (say `GoldenPlan` in
`kpi/polyglot/friction_parts.py`) that sends `instances.build("trajectory/JointTrajectory",
make_golden.INSTANCES["trajectory/JointTrajectory"])` on its first step, as the other friction tests' parts are module
level for this reason. The assertion stays as it is. Until the ruler lane decides, criterion 8's codec part reads this
one case as a ruler defect, not as a lane failure.
