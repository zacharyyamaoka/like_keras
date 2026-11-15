# contract-004: three exits in §12 M1 that M1 cannot meet (lane core, fix round 1, 2026-11-15)

An independent verifier ran M1 (`84c1e21`) and found these. Lane core re-measured them at `1c42c81` against the frozen ruler `b76d5b2`. Each one is a contract decision, so lane core does not edit `CONTRACT.md`. **The dispatcher should amend or ratify each item.** Most important first.

## 1. "criterion 3" scores 0 until M2, and until the ruler fixes `cell.py`

- `kpi/score.py:black_box` zeroes criterion 3 when any `arm_cell` row is dirty.
- **The C++ rows `Jtc`, `MoteusBus` and `Guard`** are clean only when `kpi/polyglot::test_cpp_needs_no_python` passes on the built `.so`. That needs `lk.hpp` and the C ABI, which lane cpp lands at M2.
- **The `ArmControllerPy` row** is red since fix round 1. `like_keras.backends` now resolves a backend through `sys.meta_path`, so the step's Blocker refuses `cell.py`'s module-level `from like_keras.backends import dora`. Before the fix, the lookup bypassed the Blocker (ruler-002 item 2). The ruler lane owns that fix.
- **What M1 can show:**
  - the 13 Python blocked-step rows (`BLOCKED_STEPS`);
  - the static scan (`kpi/blackbox.py`: rules 1, 2, 4 and 5, and no environment read in the library outside `backends`);
  - import-linter.
- **Amend:** in §12 M1's exit, change "criterion 3" to "criterion 3: the Python blocked-step rows, the static scan and import-linter". Or ratify that criterion 3 scores at M2.

## 2. "facilities (Python)" cannot be fully green at M1

- `kpi/facilities::test_introspection[python-inproc]` runs `lk ls <socket>` (`python -m like_keras.cli ls`) against the attach socket. Both are lane dora's (§1: `src/like_keras/{attach,cli}.py`), and lane dora lands them at M3.
- Criterion 9 also scores a facility only when all four combinations pass: python/cpp × inproc/dora. So criterion 9 cannot count any facility before M3, whatever M1 does.
- `test_health[python-inproc]` is wall-clock sensitive in the default tier; see ruler-003.
- **Amend:** change M1's exit to "facilities on python-inproc, except introspection's `lk ls` step", or move that step to M3's exit.

## 3. contract-003 item 1 is still open: the dora stand-in in lane dora's path

- The frozen ruler imports `like_keras.backends.dora` at module level in:
  - `kpi/tasks/arm_cell/cell.py`;
  - all eight `kpi/tasks/arm_cell/swaps/*/*/deploy.py`;
  - the two dora graphs of `kpi/validgraphs`.
- So M1's equivalence gate (the inproc half), A3, criterion 10 and criterion 12 need the module to import.
- Lane core wrote a capability-only stand-in at `src/like_keras/backends/dora/__init__.py`. Its `start()` refuses. Lane dora replaces it whole at M3.
- **Choose one:**
  - amend §12 so M1 lands a capability-only stub;
  - have the ruler lane move those imports inside the functions that use them, as `kpi/tasks/relay/solution/relay.py` does, and core deletes the stand-in.
