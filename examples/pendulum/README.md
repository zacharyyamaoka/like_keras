# Always-on pendulum

The always-on form of CONTRACT §10 (R2) on the One Dof Test Rack: the core (bus → JTC → guard, 100 Hz) comes up once
and stays up; a Rerun viewer and any number of scripts attach to it through `lk.attach`, one after another, and leave.
Zach, Daily Note 2026-11-28: *"bring up the system and display a live feed of the robot in rerun. Then launch a script,
have it run something, then launch another script. in the mean time though we are constantly seeing the arm"*.

| File | What |
|---|---|
| `core.py` | the core: `PendulumBus` + the reference JTC and guard (kpi/tasks/arm_cell, read-only), exposing `trajectory`, `state`, `command` |
| `bus.py` | the pendulum's I/O box: `bind="twin"` (a damped pendulum behind a moteus-like PD) or `bind="real"` (moteus on fdcanusb 272813EF under the rack's bus lock, targets clamped to home ± a window, returns home before it stops) |
| `viewer.py` | the live feed: an attached, read-only cell logging every `state` and `command` sample to Rerun (web viewer on :9096) |
| `script.py` | one script = one attached cell: plans from the measured state, sends one trajectory under the write lease, leaves |
| `up.sh` | brings the core up on a private dora 0.5 daemon (own network namespace) and forwards Ctrl-C to the core first |

Run (each in its own terminal; the core first):

```bash
bash /home/bam/like_keras/examples/pendulum/up.sh twin 11
```

```bash
PYTHONPATH=/home/bam/like_keras:/home/bam/like_keras/src /home/bam/like_keras/examples/pendulum/.venv/bin/python -m examples.pendulum.viewer
```

```bash
PYTHONPATH=/home/bam/like_keras:/home/bam/like_keras/src /home/bam/like_keras/examples/pendulum/.venv/bin/python -m examples.pendulum.script swing
```

```bash
PYTHONPATH=/home/bam/like_keras:/home/bam/like_keras/src /home/bam/like_keras/examples/pendulum/.venv/bin/python -m examples.pendulum.script sine
```

`real 11` instead of `twin 11` drives CAN 11 on the rack (it refuses if a rack session holds the bus lock). On the
rack, write `~/.cache/rack-commissioning/rack_can11_want` first (bam_ws memory can11-slot-on-demand): CAN 11's swings
shake the full arm's frame. `lk top unix:/run/user/1000/like_keras/pendulum.sock` shows every box's health and timing.

First run, 2026-11-28 (real CAN 11, report `reports/media/lk-pendulum-always-on-2026-11-28.html`, built by
`docs/build_pendulum_always_on_report.py`): 99 s, 9896 samples at 10.00 ms, five scripts incl. one SIGKILLed mid-move,
0 faults, 0.80° RMS while moving, 2.76 N·m peak against the 3.0 N·m cap.

Known gap: like_keras's dora backend gives `place(host=)` to component nodes but not to its gateway node, so a
deployment cannot run on the production daemon (machine `ubuntu`); `up.sh` therefore runs a private one.
