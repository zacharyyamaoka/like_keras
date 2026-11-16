# consumer-005: a component cannot know its tick period, so the bus's open-time exchange budget check cannot be ported (controllers lane, M2′, 2026-11-16)

**The ask.** Let a component read its group's period (or hz) from `activate()` on: `double period() const`, 0 for a rate-less group.

## What I tried

The raw node runs ArmPlant's open-time gate before its first command (`mcn/control_loop.cpp` `ControlLoop::budget_check`): 20 query exchanges on the bus, refused when their p95 is over 0.8 × the tick period. WHY (raw README): 7 boards on one fdcanusb take about 15 ms per exchange. 100 Hz then overruns every other tick, and 50 Hz fits only while the p95 stays under 16 ms. The gate refuses a placement the bus cannot serve before anything moves.

## Contract sections

§4.4: the rate is the Deployment's (`place(hz=...)`), by design. §6.1: the Component API has `now()` but nothing about the schedule. §5.2: overruns are counted, after the fact.

## Repro

```bash
grep -n "hz\|period" /home/bam/like_keras/cpp/include/like_keras/lk.hpp
```

prints nothing. The rate lives only in `lk_place.hz`, which a component cannot read (`cpp/src/box.hpp` `Box::place`).

## What I did instead

- The port (`lk/arm_controller.cpp`) does not run the budget check. On the twin it is moot. On a real bus the port is hold-only anyway: the raw node's motion gate for the missing interlocks is kept and keyed on `Bus::real()`.
- The node's own INI has a `rate_hz`, but using it would let the INI and the placement disagree silently. A Deployment that places the chain at 100 Hz on a bus that needs 15 ms would only show up as `overruns` in metrics, after the arm has already been commanded.

## Proposal

- **Contract change (§6.1, §4.1), with a library fix.** `double Component::period() const` (C++) and `self.period()` (Python): the group's period in seconds, 0.0 for a rate-less group, valid from `activate()` on. It adds one public name (§3). The alternative is a field of the params `activate` sees.
- **New ruler test.** A C++ and a Python component placed at 50 Hz read 0.02 in `activate`. One that refuses in `activate` when `period()` is below a param fails `start()` naming the reason, before any step (T7's shape).

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).
