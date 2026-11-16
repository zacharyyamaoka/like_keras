# consumer-012: a native box's last log lines never arrive: nothing drains its log after the last facility round, and deactivate()'s lines are lost (controllers lane, M2′ fix round 1, 2026-11-16)

**The ask.** When a deployment stops (`until` reached, `stop()`, a signal once A18 lands), drain every native box's facility ports once more after the last `deactivate`, in lockstep and in real time, so the lines a C++ component logged in its last partial second, and in `deactivate()`, reach the log port as a Python component's do.

## What I tried

The swap test compares `same_box` (the C++ chain) with `fallback` (its Python peer) in lockstep. An independent verifier (fix round 1) gave both a targets file with one target past j1's limit at 3 Hz for 20 s. `same_box` delivered 13 of the 14 refusal log lines that `fallback` delivered: the last one was missing. The port's hand-back line (`MoteusChain::deactivate()` logs `hand-back: %g hand-off frame(s) sent, ...`) has never reached a log reader in any run.

## Contract sections

§8 (the facility ports carry every record), §5.7 (stop deactivates in reverse order), §15 A10 (native records under lockstep), A18 (which already says "the facility ports are drained once more after the last deactivate", for signals), T1 (a swap must not change what a reader sees).

## Repro

`lk/friction_repro/log_drain.py`: one plan past j1's limit at t = 2.5 s, so the chain logs one refusal line, in lockstep until 3.0 s (the last facility round is at 2.0 s) and until 3.5 s (a round at 3.0 s), in both placements.

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX /home/bam/like_keras/.venv/bin/python $PKG/lk/friction_repro/log_drain.py
```

```
until 3.0, fallback: 2 lines: ['1794828373.31 holding (reason 0, joint -1)', '1794828373.45 trajectory refused: joint 0 sample 0 outside its limits']
until 3.0, same_box: 1 lines: ['1794828373.48 holding (reason 0, joint -1)']
until 3.5, fallback: 2 lines: ['1794828373.61 holding (reason 0, joint -1)', '1794828373.78 trajectory refused: joint 0 sample 0 outside its limits']
until 3.5, same_box: 2 lines: ['1794828373.92 holding (reason 0, joint -1)', '1794828374.02 trajectory refused: joint 0 sample 0 outside its limits']
```

- With `until=3.0`, `same_box` loses the refusal line that `fallback` delivers. With `until=3.5` it arrives.
- `MoteusChain::deactivate()` logs `hand-back: %g hand-off frame(s) sent, arm-hold resumed %g` in every `same_box` run. It is missing from all four lists. (`fallback`'s `deactivate()` logs nothing.)
- Load 25.0 / 26.2 / 28.7, rack powered, cores 22-23. Lockstep, so the result does not depend on the load.

The cause is in `backends/inproc.py`:

- A native box's log reaches its readers only from `_publish`: the facility heap entry once a logical second in lockstep (`_lockstep`: `while heap and heap[0][0] < until`, so with `until=3.0` the last round is at 2.0 s), and the `_facilities` thread once a wall second in real time, which exits as soon as `stopping` is set.
- `Run.stop()` joins the threads, deactivates every group and closes. Nothing drains the box's log ring after that. So the lines from the last partial second and every line `deactivate()` writes stay in the ring and are dropped with the box.
- A Python component's log line reaches its sinks at once (`component._Log`), so `fallback` loses nothing. The same run therefore gives different log text in the two placements.

Health and metrics have the same gap: the last partial second is never published for a native box. The log matters most, because a refusal, a brake or the hand-back is often the last thing a controller says.

## What I did instead

- In the swap test, the scripted run ends its events by 7.0 s, and its last facility round is at 8.0 s, so the comparison does not cross the gap. `lk/tests/test_swap.py` now asserts the two placements' log text is equal there.
- The hand-back stays checked by its effects (`last_targets.json`, `arm_hold.resume`; `test_lifecycle.py`), not by its log line.

## Proposal

- **Library fix (A18, for every stop, not only signals).** In `Run.stop()`, after the last `group.deactivate()`, call each native box's `publish(now)` once more, in lockstep at the logical time of the last step and in real time at the wall time, and flush the recorders. `lk-dora-host` does the same before it closes its outputs.
- **New ruler test.** A native box logs one line in its last partial second and one in `deactivate()`. A log tap sees both lines, in lockstep (`until` between two facility rounds) and in real time, and the same lines from a Python twin of the box.

## Resolution (2026-11-16, dispatcher)

CONTRACT.md §15 A28 (library fix).
