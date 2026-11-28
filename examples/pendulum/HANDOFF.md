# Always-on pendulum core → dispatcher handoff (W3-HWI-28, 2026-11-28)

Written for Robot Activity and the W3 goal loop. It **reuses the evidence of the one real run** and adds no new
hardware operation. Evidence: `/archive/datasets/bam_rig/rack_2026-11-28/lk_pendulum_always_on/`
(`core.mcap`, `core.log`, `events.json`, `sync.json`, `rack_d405.mp4`, `viewer.webm`, `feed.rrd`).
The report is at `reports/media/lk-pendulum-always-on-2026-11-28.html`, and the code is at like_keras `0c4fc41`.

## The real CAN 11 run, precisely

- **When:** 2026-11-28, **06:02:47 → 06:04:30 MST** (`sync.json` webm_epoch0 1795870480.545 + `events.json` 487.2 s …).
  An earlier chat summary said 05:54–05:56; that was wrong.
- **What:** board CAN 11 (JMC60 shoulder pendulum) on fdcanusb `272813EF`. It was the only board that answered on that
  adapter (36 V, mode 0, 15.97° at rest before the run).
- **Limits:**
  - torque cap 3.0 N·m;
  - a target window of home ±40° (`bus.py` STATIONS);
  - the guard refuses a target more than 0.5 rad from the measured position, or a velocity over 1.0 rad/s;
  - a board watchdog of 0.25 s on every command.
- **Sequence:**
  1. swing ±10° (the probe);
  2. swing ±30°;
  3. sine 20° at 0.3 Hz;
  4. a sine SIGKILLed mid-move, after which the core held at 28.8°;
  5. the next script took over from that hold;
  6. SIGTERM to the core.
- **Numbers:**
  - 99 s at 100 Hz, 9,896 samples, dt median 10.00 ms;
  - 5 gaps over 20 ms (max 60 ms), with no real-time CPU priority;
  - 0 faults;
  - 0.80° RMS tracking while moving (peak 2.06°);
  - 2.76 N·m peak torque.

## The state it was left in

- On shutdown the core ran a 2.5 s min-jerk back to home (27.7° → 16.1°), held there 0.5 s, then sent `set_stop`.
  The board read **mode 0 (stopped), fault 0, 16.1°**, which is where it hangs at rest.
- **Nothing is still running** from this work: no core, no private dora, no viewer.
- `~/.cache/rack-commissioning/rack_can11_want` was deleted after the run.
- The rack lock file is unlocked; it has been the same 0-byte file since 11-27 11:46.

## Ownership and lock: what it did, and the gap

- **Locks taken:** only `flock(LOCK_EX|LOCK_NB)` on `~/.cache/rack-commissioning/bus.272813EF.lock`, held from
  activate to deactivate. It refuses to start if the lock is held, and refuses if the board reports a fault.
- **Not taken:**
  - **no rackq lease**;
  - **no Robot Activity hold or job**;
  - **no `arm_grasp_hold` write** (the hold that stops the full arm from grasping while CAN 11 swings).
- **Coordination with the full arm** was by hand: I waited for `fullarm_tview_home` to finish, then wrote
  `rack_can11_want`. This is the integration gap that W3-HWI-LOCK names: adapter-only locking.
- **Robot Activity did not see the run.** In `activity/rack_shoulder_pendulum/2026-11-28.jsonl`, the rows from
  06:02 to 06:04 read `claimed: false`, `state: "no host"`, `idle_reason: "unclaimed"`, and have empty
  `holders`, `lock_holders` and `sibling_holders`. Two separate causes:
  1. **No station token.** The core's command line is
     `python -m examples.pendulum.core --bind real --can-id 11 --backend dora`. It has no `can11`, so
     `station_match("can11")` could never assign the run to this robot.
  2. **No holder seen at all.** Even `sibling_holders` was empty, which means the /proc scan found no holder of the
     port or of the lock. Two possible reasons, neither confirmed:
     - The port was held by a dora node process, not by the `core` process I launched.
     - The run sat inside `bwrap --unshare-net`.

     The lock fd lives in the dora node too. Robot Activity owns this; I made no change to any live service.

## Attach semantics, as built

- The core is one like_keras System at 100 Hz: `bus → JtcPy → GuardPy`.
- Its attach socket is a file at `unix:/run/user/1000/like_keras/pendulum.sock`. It exposes three ports:
  - `trajectory` — an input with a **write lease**, one writer at a time;
  - `state` — an output;
  - `command` — an output: what the guard actually sent.
- **A reader** (the Rerun viewer) only subscribes. It never takes the lease, so any number of readers can come and
  go while the arm moves.
- **A writer** (a script) holds the lease while it sends. When the writer leaves or dies, the lease lapses after
  0.25 s and the core retracts its trajectory. The JTC then **holds where it is**; it does not go home.
- **The board side:** if the core itself dies, the moteus watchdog (0.25 s) drops the board out of position mode.
  The pendulum is then limp and swings to its bottom. Only a clean SIGTERM or Ctrl-C returns it home first.
- **Not built:**
  - a decelerate-to-hold when a writer dies (today it stops instantly, at about 30°/s, overshooting 0.7°);
  - any check of global dispatcher admission before `activate` takes the bus.

## The five-bar equivalent (bam_fb_v2): status only, read from Robot Activity at 07:42 MST

- The full arm **does not run this core.** Its always-on piece is `arm-hold.service`, on its own adapter `0607623A`.
  arm-hold parks the arm between jobs, and jobs take over the arm through rackq / rack-runner hand-offs. It plays the
  role of this core's "hold", but it has no attach socket and no lease.
- **Current reading** (from `latest.json`, not verified independently and not acted on):
  - `state: "held off READY"`, `idle_reason: "blocked on code"`;
  - off READY by J2 +6.7°, C1 −76.1° and W1 +3.2°;
  - fault register 104 on J2 and W1;
  - every board in mode 10;
  - queue depth 4, 0 runnable.
- That rig belongs to its owners: Trajectory Tracking / Sim to Real, and the W2 agent's recovery. I did not touch it.

## What the seam needs (for its one writer, not done here)

This is a proposal for the owner of the dispatcher wrapper. It needs device-free admission and regression tests only.

1. **Admission.** `core.py --bind real` must pass through the same admission a rack job does before `activate`
   opens the bus: a rackq lease, or Robot Activity's hold, plus `arm_grasp_hold` for CAN 11. The wrapper owns that
   check, not like_keras.
2. **A station token.** The core's command line carries the station (for example `--station can11`), so
   `station_match` finds it; the dora node's command line needs it too.
3. **A regression test.** Fixture /proc rows for a dora-node holder of `272813EF` must read as the CAN 11 robot
   being claimed.

Until the dispatcher's owner lands its lock, nobody should run `up.sh real` again. Twin mode stays safe.
