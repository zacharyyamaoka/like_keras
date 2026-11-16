# contract-005: what the C++ lane read into §6 and §8, and one size miss (lane CPP, M2, 2026-11-15)

The C++ runtime is on `build/v0` (`fe60099`). Where the contract was silent, the runtime took the readings below. **The dispatcher should ratify each one or amend the contract.** Most important first.

## 1. The C++ runtime is twice its size target (§11 criterion 7)

- **Measured.** After `kpi/surface.py --size`, which formats with clang-format Google, 120 columns:

  | File | SLOC |
  |---|---|
  | `cpp/src/rt.cpp` | 639 |
  | `cpp/src/abi.cpp` | 573 |
  | `cpp/include/like_keras/lk.hpp` | 242 |
  | `cpp/include/like_keras/abi.h` | 46 |
  | **`cpp_runtime` part** | **1500** (target 850, which also covers `cpp/hosts/`) |
  | `cpp/src/box.hpp` (the box both `.cpp` files share; counted under "other") | 185 |

  The runtime also grew `backends/inproc.py` from 411 to about 670 SLOC with the native host and its Arrow bridge (target 300).
- **Where the lines go:**
  - the C ABI, the Arrow codec of the 8 v0 types and the LK_PARAMS JSON: about 570;
  - the facilities: about 150;
  - the box build, which checks the wiring and derives the order: about 110;
  - the RT thread, the slots and stepping: about 300.
- **The decision.** Either amend the target (about 1,700 for the runtime with the dora host), or order a compaction pass.
  - A pass could drop the JSON parser for a flat `key=value` params string. That is a §6.1 change.
  - It could also generate the per-type codec tables from `codec.py` into the committed views. That is lane core's generator.
  - I did not cut features to meet the number.

## 2. Facilities cross the C ABI as extra out ports (§6.3, §8)

- **The reading.** The out ports after the exposed ones are the box's `log`, `health` and `metrics`, in that order. Each `lk_take` call returns one record.
  - A round is every record (one per component, or every log line drained), then 0 once, then the next round starts.
  - A record's `path` is the component's name in the box. The host prefixes the box's own path.
  - `lk_describe()` adds `"facilities": ["log", "health", "metrics"]`.
- **What is built where.** The RT thread only bumps counters and fixed-bin histograms (8 bins per octave, relaxed stores). Percentiles and text are built on the thread that takes the records. That is the Python facilities thread under `inproc`, and should be `lk-dora-host`'s TX thread at M3.
- **Why.** §6.3 has no call for facilities, and §8 requires them on both backends with zero user lines.
- **Status.** Criterion 9 on `cpp-inproc` passes 6 of 7. Introspection fails only at `lk ls`, which belongs to the dora lane.

## 3. `lk_push(box, port, NULL)` retracts that input (§5.4, §9.3)

- After a retraction, a latest input reads as `nullptr` until its next value, and a queue is emptied.
- The attach gateway (M3) needs a way to retract, and §6.3 had no call for it.

## 4. `lk_describe()` constructs the System from its default `Params`

- §4.5 says "no construction, no I/O". Ports only exist once the components are constructed, so the reading is narrower: **no activation and no I/O**.
- The System is built with default Params, its exposed ports and the Params defaults are read, and it is destroyed. No `activate` runs, and no thread or device opens.
- A factory whose ports depend on its params is outside v0.

## 5. Smaller readings

- **Stale inputs under lockstep.** `lk_push` carries no time, so an input's arrival stamp is the box's last step time under lockstep, and the box clock under real time. A `stale=` on an exposed native input is therefore judged up to one period early under lockstep. No v0 component has one.
- **A full queue ring.** The reader trims to n, dropping the oldest, and counts it in `drops.<port>`. The ring holds 2n. When it is full, the writer refuses the newest, also counted, so the writer never waits.
- **An exposed Out's strings.** The RT thread copies them into a 4 KiB arena per slot, because a view may point into an input the box has since retired. A message whose strings exceed 4 KiB is not published, and is counted.
- **SCHED_FIFO refused by the kernel.** The thread runs SCHED_OTHER on the same absolute deadlines. The refusal is logged at warning, which turns `health` to `warn` for one period. See `ruler-004` on what that means for the timing tier.
- **A native box's warnings and errors** also go to the `like_keras` logger, as a Python component's do (`component._Log`).
- **More joints than `kMaxJoints` (16) on an input.** The push is refused (`lk_push` returns -1, logged once), never cut.
- **Not in v0.** A System added as a child of a System in C++, and an exposed `Out<JointTrajectory>` (§7.4: sample-major types are In only). Both are refused with a message at `lk_create`.
- **Push cost under `inproc`.**
  - Exporting A2's 7 × 1000 plan as Arrow C Data takes about 100 µs p50 on the planner's thread here. It is zero-copy, but it walks 28 nodes, about half of them the empty `limits`, `channels` and `meta` subtrees the frozen layout requires.
  - `lk_push` itself takes 2 µs.
  - This is not the timing tier, and A2's budget is 0.1 ms. If the timing tier misses it, the next lever is to skip the Python walk for null subtrees.

## Resolution (2026-11-15, dispatcher)

See CONTRACT.md §15 (amendments A1-A7). Decided: A5 (targets stay; compaction milestone M4), A6 (§2-5 readings ratified).
