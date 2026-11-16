# consumer-002: a latest In's view dies under a component that is still using it, so every plan is copied on the RT thread (controllers lane, M2′, 2026-11-16)

**The ask.** Give a C++ component a way to keep a received message past the step it arrived in, or a way to convert it off the RT thread. Today the RT thread must copy every trajectory into storage it owns, which is the work the raw-dora node kept off its control thread.

## What I tried

The raw node's hand-off (`mcn/handoff.hpp`) works like this:

- The dora thread decodes and validates each trajectory, maps its joints, clips its effort, and **allocates** a `Plan` for it.
- It posts the Plan pointer to a lock-free one-slot `PlanMailbox`.
- The control thread runs that Plan for as long as it lasts, often seconds.
- When the control thread is done with a Plan, it hands it back on a `retired` SPSC ring, and the dora thread frees it.

I wanted the lk equivalent: keep the `rm::JointTrajectory` view that `trajectory.take()` returns while the plan runs.

## Contract sections

§7.4 says "the buffers stay alive until the RT thread retires the message", but nothing lets a component say it has not retired it. §5.1 (slots, the graveyard). §6.1 says "forward touches only preallocated storage".

## Repro (a plan still running, its buffers released)

`lk/friction_repro/view_lifetime.py`, on bam_ws branch `claude/cpp-controller-nodes-f9a521`, drives `libmcn_arm_controller.so` through the C ABI, stepped. The run:

1. Push plan A, 10 s long. The chain adopts it.
2. Push three more plans. Each starts 0.2 rad off, so the chain refuses it and **keeps running A**.
3. After each push, print whether inproc still keeps A's message alive, and the chain's command against A's own sample.

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
SBX="nice -n 19 ionice -c3 taskset -c 22-23 bwrap --dev-bind / / --ro-bind /archive /archive --dev /dev --bind /dev/shm /dev/shm --bind /run/user/1000 /run/user/1000 -- env BAM_LANE_NO_DEVICES=1"
$SBX /home/bam/like_keras/.venv/bin/python $PKG/lk/friction_repro/view_lifetime.py
```

```
t=0.13 A pushed and adopted; A alive: True; command j1 = 0.000800
t=0.23 refused plan 1 pushed; A alive: True; command j1 = 0.001800 (A's own sample: 0.001800)
t=0.33 refused plan 2 pushed; A alive: True; command j1 = 0.002800 (A's own sample: 0.002800)
t=0.43 refused plan 3 pushed; A alive: False; command j1 = 0.003800 (A's own sample: 0.003800)
```

The chain still commands A's samples, but A's numpy buffers are gone after the third push. `cpp/src/rt.cpp` `Box::push` releases the triple buffer's back slot ("the RT thread reads it no more"). The RT step also swaps a fresh value in at the start of **every** step, without asking the component. So a span held from an earlier step points into freed memory **two pushes after the step moved off it**. Only the chain's own copy kept this run correct.

## What I did instead

`lk/arm_controller.cpp` copies every admitted trajectory on the RT thread (`MoteusChain::admit` → `mcn::fill_plan`):

- It keeps a pool of 4 `mcn::Plan`s, each reserved in `activate` for `plan_samples` (4096) × 16 joints: about 4 × 1.6 MB.
- On the RT thread it copies, validates (finite values, strictly increasing times, joint limits) and clips effort.
- A trajectory longer than `plan_samples` is refused, where the raw node accepts any size.
- To make this possible I factored the raw node's `make_plan` into an allocation-free `fill_plan` (`mcn/plan.cpp`), shared by both paths. That was the one change to raw-dora files for this note.
- Measured: the chain step's p50 is about 9 µs and its 1 s window max about 36 µs in lockstep, under A2's load of a 7 × 1000 plan every 0.5 s. That window includes the copy and the validation, which now run inside the step.
- `lk/tests/test_rt_path.py` shows the copy allocates nothing on the RT thread (0 allocations over 60 s). The probe counts allocations when the mutant's fresh Plan per trajectory is in place.

The raw node's mailbox + retire ring, rebuilt here as a pool on the RT thread, is consumer code the contract promised to make unnecessary. Its O(N·J) copy now runs on the RT thread instead of the I/O thread.

## Proposal (pick one)

- **`In<T>::keep()`, a library fix.** It returns a handle that pins the current message until the component drops it. The box retires pinned messages only when they are released, via the graveyard as today. Bound the number of pins per port, and refuse a push into a full set (counted).
- **A host-side conversion, a contract change.** An `In<rm::JointTrajectory, Plan>` whose `Plan` is built by a component-provided function on the **pushing** thread inside `lk_push`, where allocation is allowed, and handed to the RT thread as a pointer, then retired. That is exactly the raw node's PlanMailbox. It keeps validation and O(N·J) work off the RT thread.
- **New ruler test (either way).** A C++ box that keeps the first trajectory for 2 s while N more are pushed must read the first one's values intact (ASan build). Today that test would have to fail.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).

## Verified by the consumer (2026-11-16, re-port onto A16-A28 and M3′)

Resolved by A17, with one remainder filed as consumer-014. `trajectory` is `lk::In<rm::JointTrajectory, PlanIn>`. `PlanIn::from` runs on the pushing thread: name map, validation, effort clip and copy (`mcn::fill_plan`, the raw node's). The RT thread swaps the plan's vectors with a pool slot's, so there is no copy and no allocation. The slot's old buffers stay behind in the converted value, which like_keras frees off the RT thread.
- `lk/tests/test_rt_path.py` (A2's load, 60 s, shipped twin): RT thread 0 allocations and 0 locks after the warm-up, never allocated in its life; `plans_converted_on_rt` 0 and `plans_copied_on_rt` 0. The second is the conversion's buffer pointer, checked on the RT thread.
- The copy mutant (each plan copied into a fresh Plan on the RT thread) is caught by both probes: 66 allocations and 13-14 copies in 8 s.
- Remainder (consumer-014): `from` is static, so the config it needs comes through a process-wide pointer, one chain per process.
- bam_ws `claude/cpp-controller-nodes-f9a521`, `src/core/mdp/agent/actuators/moteus_controller_node/lk`; `lk/tests` green 3 of 3 back to back in the lane's sandbox (rack powered, cores 22-23, not the timing tier).
