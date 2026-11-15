# like_keras v0: judging the five proposals (2026-11-15)

*Synthesis of three independent judges (`consumer-rt`, `zach-fit`, `longevity`, all Opus 5.5) scoring the five proposals in this folder against the frozen weighted criteria in [criteria.md](criteria.md). The frozen build contract that follows from this pick is [../../CONTRACT.md](../../CONTRACT.md).*

## Verdict

- **P1, the thin facade, wins on the mean of the three judges: 76.4, against P2 75.5, P3 74.9, P4 73.9 and P5 67.8.** Two of the three judges rank it first. `consumer-rt` ranks it fourth (73.1) and P3 first (77.1).
- **The top four sit inside 2.5 points, which is inside the judges' own disagreement.** The pick therefore rests on where each proposal loses points and whether a graft can win them back, not on the total alone.
- **P1 loses almost all of its deficit on one criterion, speed (5.67, the lowest in the field).** Every speed fault the judges found has a known fix taken from a runner-up: an Arrow wire instead of the opaque frame, dora's cxx API instead of its C API, and a public `robot_msgs` adopt. P3 and P4 lose on things a graft cannot fix: P3 needs Python to build C++ and a generator per type; P4 compiles the core everywhere and misses the size target by construction.
- **Material qualification: P1's C++ dora host, as written, is not buildable as specified.** It calls dora 0.5.0's C node API from two threads, and that API aliases one `&mut DoraContext` with no lock and no timeout (verified by `consumer-rt` in `apis/c/node/src/lib.rs` at v0.5.0). The controller lane also hit a second limit while building against it: the C API reads only `UInt8` inputs, and any other Arrow type reaches a `todo!()` that aborts the node (`bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node/cpp/include/mcn/wire.hpp:3`). The contract moves the host onto the cxx API, whose `init_dora_node()` returns separate `Events` and `OutputSender` handles.
- **No proposal fails a hard gate by construction** (all three judges agree).

## How the numbers were made

- Each judge scored every proposal 0–10 on each of the 12 criteria, using the paper proxy in `criteria.md`. A weighted total is `Σ wᵢ·sᵢ / 10`, which runs 0–100.
  - `criteria.md` specifies a 1–5 paper scale (`Σ wᵢ (sᵢ − 1) / 4`). The judges used 0–10 instead. That changes the absolute numbers, not the ranking.
- The workflow script's own weighted totals came out as 0 for every proposal. The judges labelled the criteria differently (`1-consumer-fit` against the full criterion name), and the script's lookup matched neither. Every total here is recomputed from the raw per-criterion scores in the workflow journal (`wf_a62a7de7-368/journal.jsonl`, entries 35–37).
- Self-scores in the proposals are ignored. The `longevity` judge estimates they were inflated by about 7 (P3) to 11 (P5) points.

## The criteria (fixed before the options)

| # | Criterion | Weight | v0 target (score 1) | Zero point (score 0) |
|---|---|---|---|---|
| 1 | Consumer fit: the controller case end to end | 15 | 9/9 consumer tests | 4/9 |
| 2 | Speed: adds nothing on top of the transport | 13 | ≤ max(10 µs, 5%) over raw dora p99; C++ ≤ 100 ns, Python ≤ 1 µs per edge; 0 extra copies; import ≤ 20 ms | +100% p99; 1 µs / 10 µs per edge; 1 extra copy; 97 ms |
| 3 | Black-box ports, wired from outside | 11 | 100% of reference components clean | 80%, or any violation in the controller task |
| 4 | API succinctness | 11 | ≤ 8 top-level and ≤ 30 total names; ≤ 6 params; ≤ 0.4× raw dora, ≤ 1.2× plain Python | 11 / 43 names; 10 params; 1.0× / 2.0× |
| 5 | Backend-swap cost | 9 | 0 component or System lines; ≤ 1 Deployment line per swap | 20 lines per swap, or any component edit |
| 6 | Insulation from dora churn | 8 | dora named in 1 module; skew refused in ≤ 2 s; conformance green on 2 backends | dora in core or a component, or a skew hangs |
| 7 | Codebase compactness | 8 | core ≤ 700, each backend ≤ 350, C++ runtime ≤ 900, total ≤ 2,500 SLOC | total 5,000 |
| 8 | Python + C++ reach | 7 | 5/5; ≤ 2 files and ≤ 10 non-body lines per C++ component | a hand-written dora node per C++ component |
| 9 | Standard component facilities | 6 | 7/7 in both languages, no user code, ≤ 30% of SLOC | 3/7, or any facility needs user code |
| 10 | Testability with twins and sim | 5 | 4/4, at ≥ 10× real time | edits or wall-clock sleeps needed |
| 11 | Network attach (the IP-camera idea) | 4 | 4/4 on the same host | needs a restart or a dataflow edit |
| 12 | Wiring checked before run | 3 | 100% of ~20 mis-wirings caught, 0 false positives | 50% |
| | **Total** | **100** | | |

Hard gates, pass/fail and never traded: **equivalence**, **fallback untouched**, **real-time hot path** (0 allocations per tick), **lean core** (only `robot_msgs` at runtime).

## Score matrix

Mean of the three judges, 0–10. In brackets: `consumer-rt` / `zach-fit` / `longevity`. Bold is the best in the row.

| # | Criterion | w | P1 thin facade | P2 Drake diagram | P3 spec compiler | P4 executor core | P5 channels only |
|---|---|---|---|---|---|---|---|
| 1 | Consumer fit | 15 | 7.67 (7/8/8) | **7.83** (7.5/8/8) | 7.00 (7/7/7) | 7.67 (8/7/8) | 6.67 (7/6/7) |
| 2 | Speed (adds nothing) | 13 | 5.67 (5/6/6) | 6.67 (7/6/7) | **7.33** (8/7/7) | 7.00 (7/7/7) | 6.67 (6/7/7) |
| 3 | Black-box ports, wired outside | 11 | **9.00** (9/9/9) | **9.00** (9/9/9) | **9.00** (9/9/9) | **9.00** (9/9/9) | 6.67 (7/6/7) |
| 4 | API succinctness | 11 | **8.00** (8/8/8) | 7.33 (7/8/7) | 6.00 (6/6/6) | 6.00 (6/6/6) | **8.00** (8/8/8) |
| 5 | Backend-swap cost | 9 | **9.00** (9/9/9) | 7.00 (7/7/7) | 8.33 (9/8/8) | 8.67 (9/8/9) | 8.00 (8/8/8) |
| 6 | Insulation from dora churn | 8 | **8.67** (8/9/9) | 7.67 (8/7/8) | 6.67 (7/6/7) | 8.00 (9/7/8) | 6.67 (7/6/7) |
| 7 | Codebase compactness | 8 | **7.00** (7/7/7) | 6.67 (7/6/7) | 6.33 (7/6/6) | 4.67 (5/4/5) | **7.00** (7/7/7) |
| 8 | Python + C++ reach | 7 | 7.67 (7/8/8) | **8.00** (8/8/8) | **8.00** (8/8/8) | 7.67 (8/7/8) | 6.33 (6/6/7) |
| 9 | Standard facilities | 6 | 6.67 (6/7/7) | 7.67 (7/8/8) | **8.00** (8/8/8) | 7.00 (7/7/7) | 4.00 (4/4/4) |
| 10 | Testability (twins, sim) | 5 | 8.00 (8/8/8) | **9.00** (9/9/9) | 8.67 (9/8/9) | 8.33 (9/8/8) | 8.33 (8/8/9) |
| 11 | Network attach | 4 | 6.00 (6/6/6) | 5.00 (5/5/5) | **7.33** (7/8/7) | 7.00 (7/7/7) | 4.33 (3/5/5) |
| 12 | Wiring checked before run | 3 | 8.00 (8/8/8) | 9.00 (9/9/9) | **9.33** (9/10/9) | 7.67 (7/8/8) | 7.00 (7/7/7) |

## Weighted totals (0–100)

| Proposal | consumer-rt | zach-fit | longevity | **Mean** | Rank | Borda (4-3-2-1-0) |
|---|---|---|---|---|---|---|
| P1 thin facade | 73.1 | 78.0 | 78.0 | **76.4** | 1 | 9 |
| P2 Drake diagram | 75.2 | 74.8 | 76.6 | **75.5** | 2 | 8 |
| P3 spec compiler | 77.1 | 73.5 | 74.1 | **74.9** | 3 | 7 |
| P4 executor core | 76.1 | 70.4 | 75.1 | **73.9** | 4 | 6 |
| P5 channels only | 67.1 | 65.8 | 70.4 | **67.8** | 5 | 0 |

Rankings: `consumer-rt` P3 > P4 > P2 > P1 > P5; `zach-fit` P1 > P2 > P3 > P4 > P5; `longevity` P1 > P2 > P4 > P3 > P5.

**Where P1 wins and loses against each runner-up** (weighted points, mean of judges):

| Against | P1 gains on | P1 loses on |
|---|---|---|
| P2 (−0.9 behind P1) | swap cost +1.8, dora insulation +0.8, succinctness +0.7 | speed −1.3, facilities −0.6, testability −0.5 |
| P3 (−1.5) | succinctness +2.2, dora insulation +1.6, consumer fit +1.0, swap +0.6 | speed −2.2, facilities −0.8, attach −0.5 |
| P4 (−2.5) | succinctness +2.2, compactness +1.9, dora insulation +0.5 | speed −1.7, attach −0.4 |
| P5 (−8.6) | black box +2.6, consumer fit +1.5, dora insulation +1.6, facilities +1.6 | speed −1.3 |

Speed is P1's only large loss against every rival. That is what the grafts below target.

## Each judge's verdict

### consumer-rt (ranks P3 first, about 77)

- **Why P3:** one Python declaration generates the C++ ports, params and codec, plus the Python fallback's interface. The RT chain is static, non-virtual generated code with `loan()` and a graveyard ring, and dora stays off the RT thread. The spec doubles as the IP-camera datasheet.
- **P3's real costs:** C++ builds need Python (commit the generated spec and headers); "equivalence by construction" is false (graft P2's lockstep mode); cross-machine skew is unsolved (graft P4's hello and init watchdog).
- **Verified flaws elsewhere:** P1's host uses the dora C API, where `next_event` and `send_output` alias `&mut` on one context with no timeout, and its opaque `UInt8` frame copies twice. P5 wires inside Component classes, attaches only through zenoh, and sends with `send_data_sample`, which dora 1.x removes.
- **Correction in P5's favour:** in dora 0.5.0, Python `Node.next(&self)` releases the GIL (`allow_threads`) and `send_output(&self)` uses a separate field, so one Python thread can block in `next()` while another sends.
- **Advice:** build behind P1's normative triggering rules and conformance suite; land the frozen `kpi/` ruler first; measure the RT→I/O wake and the `ctypes.CDLL` GIL re-acquire in the raw-dora A/B before trusting anyone's "adds nothing".

### zach-fit (ranks P1 first, 78.0)

- **Why P1:** it leads on the heaviest criteria: one `.so`, one System, one-line swaps and 6 top-level names. Its dora C++ seam is the C node API, which dora 1.0 kept with no changed signature.
- **Its speed story is overstated.** The opaque frame gathers a `JointTrajectory` before dora copies it (2 copies against raw dora's 1), and `robot_msgs.frozen_array` copies again on receive. Fix both: send Arrow child buffers, and add a public `robot_msgs` adopt.
- **Grafts:** P2's lockstep conformance mode and Diagnostics-Record health; P3's self-describing `.so` plus `serve()`.
- **Every proposal silently needs new `robot_msgs` types**, and **all five dropped Zach's Keras call-site kwargs wiring** (`comp1(buzz=comp2.o.fizz)`, "subscribe not publish"). Rule on that before the build lane starts.

### longevity (ranks P1 first, 78.0)

- **Why P1:** the smallest surface, one-line swaps because processes collapse under `inproc`, and the Python fallback outside `bam_rack_twin`. Keep its C ABI host, with a WHY comment, because the C ABI survives dora 1.0.
- **Fix before M1:** the in-band header forces a Python gather, the C++ ring hop adds a copy, and the "no re-validation" decode depends on a `robot_msgs` API that does not exist.
- **Reject** P4's compiled core everywhere and P3's Python-at-C++-build codegen (both tax longevity), and P5's composites-as-components (breaks the frozen "wired from outside" ruler).

## Fatal flaws, consolidated

| Proposal | Flaw | Found by | Status in the contract |
|---|---|---|---|
| P1 | C++ host on dora's C API: `dora_next_event` and `dora_send_output` from two threads alias `&mut DoraContext`; `next_event` has no timeout; the C API reads only `UInt8` | consumer-rt (source); controller lane (`wire.hpp:3`) | **Fixed:** cxx API, only `init_dora_node`, `next_event`, `event_type`, `event_as_arrow_input`, `send_arrow_output` |
| P1 | "0 extra copies" false: opaque frame gathers the struct-of-arrays, then dora copies again; C++ out path is ring memcpy plus dora's copy | all three | **Fixed:** the wire is the Arrow C Data layout of each type; Python builds it over the numpy buffers with no copy |
| P1 | Receive without re-validation needs a `robot_msgs` trusted constructor; `adopt` is "Internal use only" | zach-fit, longevity | **Fixed:** public `robot_msgs.adopt` is a barrier change before M1 |
| P1 | C++ `In` has only `get()`/`put()`, so "is this a new trajectory?" relies on pointer identity | consumer-rt | **Fixed:** `get()` on a `queue` input returns this step's message or `nullptr` |
| P1 | Health inferred from the log level; C++ params parsed by the author | consumer-rt, zach-fit | **Fixed:** `DiagnosticRecord` health set by the framework; `LK_PARAMS` reflection |
| P1 | Equivalence across processes has no mechanism | zach-fit | **Fixed:** lockstep mode in every backend |
| P2 | `Dora("0.5.0")` in every user deployment; in-process → process is 2 lines; attach scheduled last; Python decode 10–30 µs; string-keyed C++ params | zach-fit, consumer-rt | Not the base |
| P3 | C++ interfaces and chain wiring authored in Python, generated at CMake configure, so every C++ build needs Python, `like_keras` and `robot_msgs`; "equivalence by construction" false; cross-machine skew unsolved; per-message hash in dora metadata | all three | Not the base |
| P4 | Compiled core everywhere (nanobind + cargo-linked dora even for the all-Python fallback); C++ ~1,100 vs 900, total ~2,850 vs 2,500; `order()` silently turns a backwards edge into a one-tick delay | all three | Not the base |
| P5 | Wiring inside Component classes (systems are components whose `__init__` calls `connect`); attach only on zenoh; recording needs user code; C++ send on `allocate_data_sample`, which dora 1.0 removed; every constructor runs in every process | all three | Not the base |

## The one disagreement that matters: dora's C API or its cxx API

`longevity` and `zach-fit` scored P1's C ABI host as its best longevity choice, because dora's 1.0 migration guide says of the C node API: "No call signature changed." `consumer-rt` found it unsound. The evidence resolves it for the cxx API:

1. **Threading.** In v0.5.0 the C API's `dora_next_event` and `dora_send_output` both cast one context pointer to `&mut DoraContext` with no lock, and `next_event` blocks with no timeout. A receive thread plus a send thread is undefined behaviour; a lock would stall every send until the next input arrives. The 1.0 guide adds "a documented threading contract" to the C API, but v0 pins 0.5.0.
2. **Types.** The controller lane, building against the same C API today, found it reads only `UInt8` inputs (any other Arrow type aborts the node). So the C API forces an opaque frame, which forces the gather copy the judges flagged.
3. **The cxx subset survives 1.x.** `init_dora_node()` returns `DoraNode { events: Box<Events>, send_output: Box<OutputSender> }`, two separate handles (read from `apis/c++/node/src/lib.rs` at v0.5.0). The 1.0 guide removes `init_dora_node_from_id`, `init_dora_node_flexible`, `node_id`, `dataflow_id` and the `DataSampleHandle` family, and it does not list `next_event`, `event_as_arrow_input` or `send_arrow_output` as changed. like_keras uses none of the removed calls; the contract bans them by name.

Residual risk: the guide is silent on `send_arrow_output`, which is not the same as a guarantee. The conformance suite's v1 rehearsal (criterion 6, reported not scored) checks it.

## Recommendation

**Build P1 (thin facade) as the base, with the grafts below.** It has the smallest surface and the cheapest swaps, it keeps the fallback cleanest, and every point it loses is recoverable. With the speed grafts applied, the judges' own reasoning puts P1's speed near P3's 7.3, which would move P1 about 2 points clear of the field.

### Grafts taken

| # | Graft | From | Why | Where |
|---|---|---|---|---|
| G1 | The C++ dora host uses dora's **cxx** API, a receive thread and a send thread on the split `Events`/`OutputSender` handles; never the C API, never `allocate_data_sample`, `DataSampleHandle` or `init_dora_node_from_id` | P2, P4 (consumer-rt) | The C API is unsound with two threads and reads only `UInt8`; the cxx subset survives 1.x | CONTRACT §6.4 |
| G2 | **The wire is the Arrow C Data layout of each `robot_msgs` type**, owned by like_keras, with the schema hash sent once in a hello, never per message | P3, P5 (layout); P4 (hash in hello) | 0 extra copies at ≥ 4 KiB (Python wraps numpy buffers with `pa.Array.from_buffers`); no dependence on dora's metadata format, which 1.0 changed | CONTRACT §7 |
| G3 | Public `robot_msgs.adopt`, used when the schema hash matches | P2, P3 | Zero-copy receive and no 10–30 µs re-validation | CONTRACT §2, §7 |
| G4 | **Lockstep mode** in every backend, and the conformance suite written before either backend | P2 (mode); P1 (suite first) | The equivalence gate cannot pass across a `latest` feedback edge without it | CONTRACT §5.5 |
| G5 | `In(..., lagged=True)` declared by the component, so the tick order is derived; replaces P1's `System.feedback` and the consumer's `tick_order` | P2 | The one-tick lag is a fact about the bus box (moteus sends the command and the query in one frame), not about the wiring | CONTRACT §4.1, §5.3 |
| G6 | Health is a `robot_msgs.std.DiagnosticRecord` port set by the framework (UNKNOWN zero value, `expected_period_s`, STALE from `In(..., stale=)`) | P2 (record); P4 (`stale=`) | Log-derived health was coarse; follows the vault's Diagnostics Record | CONTRACT §8 |
| G7 | `lk_hello` on connect (like_keras, ABI, backend and dora versions, per-port schema hashes) and a 2 s watchdog around `init_dora_node` | P4 | Version skew refused in ≤ 2 s, including cross-machine (T11) | CONTRACT §6.4 |
| G8 | Wiring errors name `path.port`, the fix and the `file:line` of the offending call | P3 | Cheap (`inspect` at wire time) and the criterion asks for it | CONTRACT §4.3 |
| G9 | `describe()` is a content-hashed device datasheet, served on the attach socket and read by `lk ls` | P3 | The IP-camera idea needs a datasheet; it is the plan P1 already builds | CONTRACT §4.4, §9.3 |
| G10 | **Keras call-site kwargs wiring** in the subscribe direction: `s.add(PickPlanner(), state=arm.state)`, and `s.wire(arm, trajectory=planner.trajectory)` for cycles | vault (Zach's decided form, flagged by zach-fit) | "Only define the inputs for components … like subscribe not publish" (Apple Note *A start*); all five proposals dropped it | CONTRACT §4.2 |
| G11 | Objects, not strings: `backend=like_keras.backends.dora` with the version pin inside the backend; `lk.native` / `lk.attach` / `lk.replay` instead of one string-sniffing `lk.load` | zach-fit | No string enums; a dora upgrade edits 0 user lines | CONTRACT §4 |
| G12 | In-process pushes and takes go through `ctypes.PyDLL` (GIL held, never blocking); `mlockall` only in a dedicated controller process; the RT → I/O wake counted in every overhead number | consumer-rt | `CDLL` releases and re-takes the GIL on every call and can wait up to the 5 ms switch interval behind a spinning planner | CONTRACT §6.3, §11 |
| G13 | A placement group holds one language | P5, P1 | Python never runs on an RT thread; a GIL stall cannot reach the chain | CONTRACT §5.1 |
| G14 | Generated C++ message views are committed, with a hash check in CI; CMake never runs Python | fix for P3's flaw, applied to P1's `lk_messages()` | The controller `.so` must build on a box with no Python | CONTRACT §7.4 |

### Grafts considered and not taken

- **P4's single C++ edge for all dora I/O, Python included.** It would insulate the Python side from dora's Python API, but it puts a compiled, cargo-linked extension under every Python process. v0 keeps dora's own Python package behind `backends/dora/host.py`; `consumer-rt` verified it is safe to receive and send from two threads.
- **P2's typed `Dora("0.5.0")` in the user's deployment** (`longevity`). The pin lives inside the backend instead, so a dora upgrade edits 0 user lines (`zach-fit`); `describe()` and `lk ls` print it.
- **A `Claim` message in `robot_msgs`** (`zach-fit`, `longevity`). A claim message works only if every trajectory names its sender, which means `header.meta` strings (P5's flaw). The contract arbitrates where the writers are known instead: an exposed input takes writes from one lease holder at a time, at the attach gateway. No new type, and the components stay unaware of how many clients exist.
- **P5's explicit `t` argument to `forward`** (`longevity`). `self.now()` is set by the framework before every step, so it is equally deterministic, and `forward`'s arguments stay exactly the input names.
- **`robot_msgs` owning the C++ view generator** (`longevity`, from P4). The views are spans into like_keras's Arrow layout, which criterion 6 assigns to like_keras. A plain `robot_msgs` C++ binding can follow when a second C++ consumer exists.
- **P3's `serve()` as an API name.** An always-on node is a Deployment started under a systemd unit; it needs no extra name.

### Why not the others as the base

- **P3** has the best speed, facility and attach scores, but its C++ components cannot be written in C++: interfaces and chain wiring live in a Python `box.py` and are generated at CMake configure. That inverts consumer spec (b), puts Python on every C++ build host, and starts the two-generator path that rosidl grew into.
- **P2** is the closest runner-up and supplies four grafts. As a base it costs 2 Deployment lines per process swap, puts the dora version in every user file, and schedules attach last.
- **P4** is excellent on the RT executor, but it makes the compiled core mandatory even for the all-Python fallback (against "use more vanilla python") and misses compactness by construction.
- **P5** is the leanest idea, but its systems are components that wire their children in `__init__`, which is the "wiring within a component" Zach ruled out on Nov 15.

## Decisions Zach might overturn

1. **P1 over P3: C++ components are written in C++, not declared in Python and generated.** Undo: tell the build lane to take [proposal-P3-spec-compiler.md](proposal-P3-spec-compiler.md) as the base; the contract's §4–§7 port semantics carry over unchanged.
2. **Network attach is same-host only in v0** (a unix socket per deployment, the OBSBOT gateway pattern). The LAN "IP camera" is v1. Undo: move about 5 weight points to criterion 11 (assumption 5 in `criteria.md`) and add TCP to the attach module's v0 scope.
3. **Kwargs subscribe wiring is the primary Python form** (`s.add(planner, state=arm.state)`), with no publish-direction `wire(src, dst)` in Python. Undo: add `System.wire(out, *ins)` beside it; one method, no other change.
