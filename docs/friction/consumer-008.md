# consumer-008: the guard deadman and the stop/brake requests cannot be ported: no v0 type for a guard's verdict, and a C++ input cannot be optional (controllers lane, M2′, 2026-11-16)

**The ask.**

- An optional C++ input: `InOpts{.optional = true}`. It may be left unwired by a parent; it reads `nullptr` and gets no slot, like A11's Python-only optional.
- A v0 type a guard can send: "I judged the state stamped t", plus stop/brake requests.

## What I tried

Two raw-node features ride on its `command` input as JSON ops (WIRE.md):

- **The guard deadman** (`guard_timeout_s`; 0.3 s in the real config). A trajectory runs only while a Python guard keeps confirming `{"op": "alive", "state_stamp": t}`, that it judged a joint_state no older than the timeout. A blind guard stops the arm moving (`guard_silent`). WHY (raw README, verifier finding 8): the real config leaves the joint limits to Python guards, and the publish path went silent for up to 3.07 s under load.
- **`{"op": "stop"}` / `{"op": "brake"}`**: a guard's controlled stop or brake.

`mcn::ControllerChain::guard(now, guard_seen)` and `request(op)` exist and are reused by nothing in the port.

## Contract sections

§7.2 (the v0 codec types: ServoState, ServoCommand, JointTrajectory, JointState, the facility records), §6.1 (`InOpts{queue, stale, lagged}`), §4.3 check 1 ("an input left unbound (unless exposed)"), §15 A11 (optional inputs, Python only).

## Repro

```bash
grep -n "struct InOpts" -A4 /home/bam/like_keras/cpp/include/like_keras/lk.hpp
```

```
23:struct InOpts {  // same meaning as Python In(...): queue = 0 is latest; stale = 0 never goes stale
24-  int queue = 0;
25-  double stale = 0.0;
26-  bool lagged = false;
27-};
```

A `guard` input on the C++ System would have to be wired or exposed by every cell that holds the arm, the reference `pick_cell` included, even where no guard runs. The off switch would be a config value (`guard_timeout_s = nan`) that the wiring cannot see. Unwired, `describe()` refuses it: `cell.arm.guard is not bound ... bind it ... or expose it` (the same check as `trajectory` in consumer-007's repro).

No v0 type means "the state stamped t was judged safe":

- `DiagnosticRecord` (header.stamp = the evaluated state's stamp, level ok/warn/error) is the closest in meaning, but it is a facility record type;
- a `ServoState` echo is a lie about its contents.

## What I did instead

Not ported. The twin config runs with the deadman off (`guard_timeout_s` NaN), as before. The port is hold-only on a real bus (the motion gate), so nothing moves unwatched. The deadman is a prerequisite for trajectories on the real arm (raw README, "Before trajectories on the arm"), so this blocks M3′'s real-arm leg, not M2′.

## Proposal

- **Contract change (§6.1), with a library fix.** `InOpts{.optional = true}`: unwired is legal and reads `nullptr`. `describe()` lists it as optional, and an attach client may write it (A11's semantics, in C++).
- **Contract change (§7.2) and robot_msgs.** A v0 type for a guard verdict: `robot_msgs.control.GuardVerdict{header (stamp = the judged state's stamp), state_seq, verdict: "ok" | "stop" | "brake", reason: str}`. Or ratify `DiagnosticRecord` as the verdict type, with `header.stamp` = the judged state's stamp and `level` error meaning stop.
- **New ruler test.** The reference arm with an optional `verdict` input: unwired, the cell runs as today. Wired to a guard that stops confirming, a running plan stops within `guard_timeout_s` + one tick, in `same_box` and `target`.
