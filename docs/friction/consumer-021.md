# consumer-021: a push whose conversion throws, or that comes outside A31's activate/deactivate window, is refused and counted nowhere (controllers lane, adapting to A31-A37, 2026-11-16)

**The ask.** Count every push the box refuses, per input, in the box's metrics in both hosts (a `refused.<input>` beside `drops.<input>`, or the attach socket's `refused_writes.<input>` name), and log the first refusal of each kind on the box's `log` port, with the exception's `what()` when the conversion threw. The raw node's plan mailbox counts every plan it drops or refuses, and its exit report says why.

## What I tried

A31 lets `PlanIn::from` take its chain, so the controller's process-wide config pointer is gone, and with it the port's own "trajectory refused: the chain is not active" path: like_keras now refuses a push outside `activate()`..`deactivate()` itself (`cpp/src/rt.cpp` `Box::admit`). I checked what a deployer sees of such a refusal, and of the other one `admit()` makes: a conversion that throws ("Converted::from refused it (validation): counted as a refused push", `cpp/src/rt.cpp:736` at adbac3b).

## Contract sections

§15 A17 and A31 (the conversion and its window), §8 (metrics: `drops.<port>` for queues), §6.3 (`lk_push` returns -1 when refused).

## Repro

`lk/friction_repro/refused_conversion.cpp` (a box whose `Checked::from` throws on a negative stamp) and `refused_conversion.py` (20 plans at 10 Hz, real time, inproc; every other one has a negative stamp), like_keras adbac3b:

```
$SBX ~/like_keras/.venv/bin/python $PKG/lk/friction_repro/refused_conversion.py <build>/librefused_conversion.so <x>.mcap
refused.box refused a JointTrajectory on trajectory          <- stderr, the first refusal only (inproc.py _push)
sent 20 plans, 10 refused by the conversion
last metrics round of refused.box: {'steps': 335.0, ..., 'overruns': 21.0, 'drops.log': 0.0, 'unreleased': 0.0}
last metrics round of refused.box.taker: {'steps': 335.0, ..., 'input_age_s.trajectory': 1.56, 'taken': 10.0}
log lines: []
```

- Ten refusals, no counter: the box record has `drops.log` and `unreleased`, the child record `input_age_s.trajectory`. `Box::admit` returns false and `Box::push` returns -1; nothing is bumped on that path (the "counted" in the comment at `rt.cpp:736` is not true).
- The one line inproc prints is the host's own Python log of the box, not the box's `log` port, so a recording and an attach client never see it, and it is printed once per box whatever follows.
- Under dora, `cpp/hosts/dora_host.cpp:417` and `:526` (adbac3b) call `push` and drop its return value: no line at all.

## What I did instead

The controller's conversion never throws. It returns a `PlanIn` whose `refusal` says why (unknown joint, named twice, shape, too long, and `mcn::fill_plan`'s checks), and the RT thread logs the line and counts `plans_dropped` (`lk/arm_controller.cpp` `MoteusChain::admit`). That keeps the raw node's accounting, but every component with a converting input has to rebuild it, and a push the window refuses (a plan arriving during a stop, or while a restarted node activates) still leaves no trace.

## Proposal

- **Library fix.** `Box::push` bumps a per-input refused counter on every `-1`, published in the box record as `refused.<input>` in both hosts; the first refusal per input and reason is logged on the box's `log` port off the RT thread (the exception's `what()`, or "outside activate/deactivate"). The comment at `rt.cpp:736` then holds.
- **New ruler test.** A conversion that throws on every other push: the box's metrics count exactly the refused ones, in inproc and dora, and a recording of `log` has a line naming the reason.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A41/A42); its frozen test comes in ruler M0.7 (A43).

## Verified by the consumer (2026-11-16, final pass on like_keras 41ce82d)

Fixed for what the port needs (A42, `41ce82d`). Built and run against a `git archive` of like_keras `41ce82d` in the lane's sandbox; rack powered, cores 22-23, not the timing tier (repros at load 9, suite runs at load 18-31). bam_ws `claude/cpp-controller-nodes-f9a521` @ `a99338289`.

- The repro now counts and says every refusal, on the box's own record and log:
  ```
  $SBX ~/like_keras/.venv/bin/python $PKG/lk/friction_repro/refused_conversion.py <build>/librefused_conversion.so <x>.mcap
  sent 20 plans, 10 refused by the conversion
  last metrics round of refused.box: {'steps': 352.0, ..., 'unreleased': 0.0, 'refused.trajectory': 10.0}
  log lines: ['a push on trajectory was refused: its conversion threw (1 refused so far)',
              'a push on trajectory was refused: its conversion threw (6 refused so far)',
              'a push on trajectory was refused: its conversion threw (10 refused so far)']
  ```
  The count is exact (10 of 10). The lines go to the input owner's log (`refused.box.taker`): the first, then at most one a second. inproc's own stderr-only line is gone.
- Under dora, the box record of the port's `target` and always-on runs carries `refused.trajectory` and `refused.verdict`. `lk/twin_demo.py` and `lk/ab_compare.py` now print them; they read 0 in every demo and A/B run of this pass, as no plan reached the box outside its window.
- A42's warning during a stop or restart changed nothing in the port's tests: 32 passed, 1 skipped, 3 of 3 back to back. T1's 13 log lines are still identical in `target`, `same_box` and `fallback`, so no refusal line appeared in any placement. The always-on and A36 recordings of a suite run carry `refused.* = 0` and no refusal line.
- Not done, and not needed here: the line gives a fixed reason ("its conversion threw"), not the exception's `what()`. The A42 lane explained why (log entries hold their text by pointer). The port's conversion never throws: it returns a refusal, and the RT thread logs it with its reason and counts it in `plans_dropped`, the same line in both languages (T1). Not filing.
