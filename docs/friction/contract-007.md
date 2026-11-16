# contract-007: readings taken in fix round 1 after the first consumer (lanes cpp, core, dora; 2026-11-16)

The verifier's four findings on the library package (A23, A28 twice, §4.5 under A22) are fixed on `build/v0`, one commit each, with no change to `docs/CONTRACT.md`, `kpi/` or the public surface. Three of the fixes read the contract where it is silent; the dispatcher should ratify or amend them. Most important first.

1. **How a host tells a native box which exposed inputs it wires (§6.3, §15 A23).** An exposed optional queue input that nothing writes must not hold the gather of §5.2 forever, but the box cannot see the deployment's graph.
   - **Reading:** an optional queue input gates a step only once its host has pushed to it, a retraction included; both hosts retract each queue input they wire before `lk_start` (`inproc`'s `NativeGroup.activate`, `lk-dora-host`'s main). Before `lk_start` a retraction is otherwise a no-op, so no host changes meaning. Written in `abi.h`.
   - **Why not a field in `lk_place`:** `kpi/alloc_probe.cpp` and `kpi/bench/bench_chain.cpp` declare the six-field struct themselves, so a seventh field would be read past their copies.
   - A port neither wired nor exposed inside the box gets no ring at all and reads `nullptr`. Latest optional inputs are unchanged.
2. **The order of a real-time stop under dora (§4.4 `stop()`, §9.2, §15 A28).** `stop()` now SIGTERMs each native node of the run on this machine first (its pid in its `started` marker, checked against `/proc/<pid>/cmdline`), waits up to 10 s for its `drained` marker, then runs `dora stop`. A `lk-dora-host` that took a signal keeps its process until dora's Stop (at most 5 s), so dora's `restart_policy: always` never restarts a controller mid-stop.
   - **Amend if wanted:** §9.2's "`stop()` runs `dora stop --name <system>`" gains "after the native nodes have sent their last records".
3. **When a native box's log reaches readers (§8, §15 A28, T1).** §8 says the C++ log ring is "drained by the process's facilities thread"; how often was unwritten, and once a second left a reader a line short of a Python twin's.
   - **Reading:** under lockstep the log port is taken after every step, on the stepping thread (inproc and `lk-dora-host`), so a reader sees the line at the step that logged it, as with a Python component. In real time inproc's facilities thread takes it every 0.1 s; health and metrics stay at 1 Hz.
   - What a box says in `deactivate()` still reaches recordings and attach (the final drain), not reader components in the same `inproc` run, in either language: their groups have stopped by then.

Also fixed, outside the four findings: a SIGTERM that unwinds `start()` while it starts a thread made `stop()` refuse to join that thread (exit 1, nothing deactivated; the `until` cases of `tests/test_signals.py` failed in 2 of 7 runs on this host, and a new case that signals inside `Thread.start()` fails every time without the fix).
