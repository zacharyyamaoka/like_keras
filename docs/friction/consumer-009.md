# consumer-009: LK_PARAMS cannot hold a per-joint table, so the controller's config is a file path the datasheet cannot show (controllers lane, M2′, 2026-11-16)

**The ask.** Let `LK_PARAMS` reflect a `std::vector` (or a fixed `std::array` plus a count) of a nested `LK_PARAMS` aggregate, and `std::vector<double>`.

## What I tried

The controller's real config is a node table plus one row per board: 7 boards × 20 keys (CAN id, sign, offset, gains, caps, limits, hold torques, twin model). See `config/twin.ini` and `config/bam_fb_v2.ini`, and `mcn::NodeConfig` and `mcn::JointConfig` in `mcn/config.hpp`. I declared it as params.

## Contract sections

§6.1 (`LK_PARAMS` reflects an aggregate's fields), §8 (params "shown in describe()"), §11 criterion 5's counter-metric ("describe() prints the full placement (no hidden env or YAML)").

## Repro

```bash
PKG=~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node
g++ -std=c++20 -fsyntax-only -I/home/bam/like_keras/cpp/include $PKG/lk/friction_repro/params_vector.cpp 2>&1 | grep error
```

```
/home/bam/like_keras/cpp/include/like_keras/lk.hpp:262:12: error: no matching function for call to 'lk_fields(std::vector<Joint>&, lk::detail::Fields&)'
```

## What I did instead

- The component's params are `{node_ini: <path>, plan_samples: 4096}`. `activate()` parses the node's own INI with `mcn::load_config`.
- That keeps one config file for both placements: the Python peer parses the same file (`lk/node_ini.py`).
- But `describe()` and `lk ls` show a path, not the joint table, caps and limits that make the arm safe. A changed INI changes the controller without changing its datasheet's params or hash.
- It also forces I/O into `activate` rather than `lk_create`. Correct by A6, but the config's errors only surface at `start()`.

## Proposal

- **Library fix (`lk.hpp` `Fields`, `abi.cpp`'s JSON).** Add `operator()(const char*, std::vector<Nested>&)` and `std::vector<double>&`. They are filled at `lk_create`, never on the RT thread, so allocation there is fine. Defaults emit `[]`. A fixed `std::array<Nested, N>` with a separate count serves components that want no heap.
- **New ruler test.** A component with `std::vector<Row>` params round-trips through `lk.native(..., config={...rows...})`, and `describe()` shows every row. An unknown key inside a row is refused naming `rows[2].key`.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A16-A27); the frozen test comes in ruler M0.3 (A27).
