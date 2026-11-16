# consumer-006: a native box has `log`/`health`/`metrics` ports carrying every component in it; a Python System has none, so the swap changes how the chain's facilities are read (controllers lane, M2′, 2026-11-16)

**The ask.** Make `system.log`, `system.health` and `system.metrics` mean the same thing for a C++ System (`lk.native`) and a Python `lk.System`: ports that carry the records of every component inside it.

## What I tried

The swap test reads the arm's log (to see the controlled stop) and its metrics (`plans_adopted`, `plans_refused`) in both placements:

- `same_box`: `arm = lk.native(libmcn_arm_controller.so)`, a C++ System `moteus_arm` with one child, `chain`;
- `fallback`: `arm = ArmControllerPy(...)`, the Python System `moteus_arm` with one child, `chain`, built the same way on purpose so the record paths match.

## Contract sections

§8 (facilities on every component, zero user lines), §4.2 ("A System is itself a component"; "The exposed ports are the System's ports when it is added to a parent"), §10 T1 (the one-line swap), §11 criterion 5 (0 component or System lines per swap).

## Repro

`lk/friction_repro/system_facilities.py` wires a metrics tap and a log tap to `arm.metrics` and `arm.log`, first for the native arm, then for the Python System:

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX /home/bam/like_keras/.venv/bin/python $PKG/lk/friction_repro/system_facilities.py
```

```
native arm.metrics paths: ['cell.arm', 'cell.arm.chain'] | arm.log paths: ['cell.arm.chain']
ValueError: cell.arm.metrics (at system_facilities.py:14): cell.arm has no port 'metrics'; its inputs: trajectory; its
outputs: state, command. Fix: use one of them
```

Wiring the Python child's port directly from outside is refused as well:

```
ValueError: arm_cell.metrics_tap.record <- arm_cell.arm.chain.metrics (at test_swap.py:60): a writer is an Out of a
sibling or an input arm_cell exposes. Fix: expose arm_cell.arm.chain.metrics first
```

## What I did instead

`lk/tests/test_swap.py` `_facility()` reads the chain's port two ways:

- native: `arm.metrics`;
- Python: `arm.expose(chain_metrics=arm.chain.metrics)`, then `arm.chain_metrics`.

kpi's own `kpi/consumer/test_consumer.py` does the same for the reference cell (`arm.expose(guard_log=arm.guard.log)`). So every consumer that reads a controller's log, health or metrics carries a branch on the placement, which is exactly the swap cost criterion 5 counts as zero. `lk top` sees the Python children anyway, by path, but a wired reader cannot.

## Proposal

- **Library fix (core, `system.py`).** A Python System gets the three facility ports. Each carries the records of every component inside it with their full paths, as a native box's do. Exposing a single child's facility port stays possible.
- **New ruler test.** For the reference cell's arm in `same_box` and `fallback`: `arm.metrics` delivers records whose path sets are equal (`arm_cell.arm.<child>`), and `arm.log` delivers the guard's runaway line in both.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).
