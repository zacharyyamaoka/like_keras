# consumer-011: in same_box, A2's push cost is the Python export, not lk_push: 0.17 ms p50 against 0.013 ms (controllers lane, M2′, 2026-11-16; a measurement on a loaded host, not a judgment)

**The ask.** Make the planner-thread export of a `JointTrajectory` (inproc `_export`) cheaper, or move it off the planner's thread, if A2's "push p99 ≤ 0.1 ms" is to hold for a 7 × 1000 plan outside an idle timing host.

## What I measured

A2's load on the ported chain in `same_box` (`lk/tests/test_rt_path.py`: a 7 × 1000 plan with velocities every 0.5 s for 60 s, the chain at 100 Hz on core 23). The push cost is the planner's `forward` returning to the next step of its group (a plain call), with the GIL probe off: n = 59, p50 0.231 ms, p99 0.375 ms, max 0.408 ms. That was at load 24.6 / 26.7 / 27.2, with the rack powered, nice 19 on cores 22-23, not the timing tier.

Split by `lk/friction_repro/push_cost.py` (500 plans, same shape, the box running in real time):

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX /home/bam/like_keras/.venv/bin/python $PKG/lk/friction_repro/push_cost.py
```

```
export   ms: p50 0.172  p99 0.666  max 2.316
lk_push  ms: p50 0.013  p99 0.033  max 0.080
load 27.7, 29.6, 28.8 (rack powered, cores 22-23, not the timing tier)
```

So the C ABI hand-off is cheap: about 13 µs, the triple buffer plus the decode into the view. About 93 % of the push is building the Arrow C Data tree of the message in Python with ctypes: one struct per field and child: 42 nodes for a JointTrajectory (root, t, channels, header with meta, joint_names, the four sample arrays, limits). The raw node's `send_trajectory` measured 34-41 µs p50 (README), because it encodes one flat RMSG frame in numpy.

## Contract sections

§6.3 ("In-process pushes are zero-copy: the ArrowArray points at the frozen numpy buffers"), §10 A2 / T2 (push p99 ≤ 0.1 ms in-process), §11 criterion 2 ("the PyDLL path included").

## What I did instead

Nothing: the port does not touch the export. Measured and reported, not judged; the timing tier decides.

## Proposal

- **Library fix (inproc).** Cache the per-type node layout of `_export` (buffer counts, child counts, formats are fixed per type), and fill a preallocated ctypes block per message with one `memmove` of the pointer table, instead of walking the schema per message. Or build the tree in C: a small exported `lk_export` in `abi.cpp` that takes the numpy buffer pointers, the only per-message data.
- **New ruler test (timing tier).** Report `_export` and `lk_push` apart in T2, so a regression in either is visible.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).
