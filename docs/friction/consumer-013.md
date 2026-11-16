# consumer-013: a component .so exports more than the C ABI: every static library it links, whole, and 5 std:: symbols from the runtime itself (controllers lane, M2′ fix round 1, 2026-11-16)

**The ask.** Make `lk_component()` keep its own promise (`cmake/like_keras.cmake`: "Hidden visibility: a .so exports the C ABI and nothing else") for everything linked into the .so, not only the component's own sources.

## What I tried

`lk_component(mcn_arm_controller arm_controller.cpp)` plus `target_link_libraries(mcn_arm_controller PRIVATE mcn_core)`, where `mcn_core` is the raw-dora node's static library (the chain, the twin, the bus). That is the normal way to reuse a controller's code in a component, and contract §6.2 has no rule against it.

## Contract sections

§6.2 (a component is one .so built by `lk_component`), §6.3 (the C ABI, `abi.h`), criterion 8 (C++ boilerplate).

## Repro

```bash
nm -D --defined-only ~/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/moteus_controller_node/lk/build/libmcn_arm_controller.so | wc -l    # at 29c1ebfa5
for f in ~/like_keras/build/kpi/lib{cpp_gain,arm_controller,empty}.so; do echo "$f: $(nm -D --defined-only $f | awk '$3 ~ /^lk_/' | wc -l) lk_*, $(nm -D --defined-only $f | awk '$3 !~ /^lk_/' | wc -l) others"; done
```

```
192
/home/bam/like_keras/build/kpi/libcpp_gain.so: 10 lk_*, 5 others
/home/bam/like_keras/build/kpi/libarm_controller.so: 10 lk_*, 5 others
/home/bam/like_keras/build/kpi/libempty.so: 10 lk_*, 5 others
```

- The port's .so exported 192 symbols: the 10 `lk_*` entry points and 182 of `mcn::` (`ControllerChain`, `TwinTransport`, `DirectFdcanusb`, `BusLock`, `MoteusCodec`, ...), `mjbots::moteus::` and `std::` instantiations. `lk_component` sets hidden visibility on the component target only; a static library keeps the visibility it was compiled with, default.
- Even the empty reference component exports 5 non-ABI symbols, all from lk's own runtime: 3 weak `std::` instantiations (`std::deque<std::string>::emplace_back`, `_M_push_back_aux`, `std::set<std::string>` insert) and 2 GNU-unique ones (`std::__detail::__waiter_pool_base::_S_for(void const*)::__w`, `std::__detail::__from_chars_alnum_to_val_table<false>::value`). Hidden visibility does not reach them: libstdc++ declares `namespace std` with default visibility.

Why it matters: two component .so files loaded into one process (two controllers in `same_box`, or a controller beside a bridge that links the same `mcn_core` at another commit) bind each other's exported symbols. The first loaded wins, so the second runs the first one's `mcn::` code against its own data layout. Nothing reports it.

## What I did instead

In the port's own CMake (`lk/CMakeLists.txt`, fix round 1): `CMAKE_CXX_VISIBILITY_PRESET hidden` and `CMAKE_VISIBILITY_INLINES_HIDDEN ON` before `add_subdirectory(../cpp)`, so `mcn_core` compiles hidden, and a linker version script (`lk/c_abi_only.map`: `global: lk_*; local: *;`) on the component and its alloc mutant. Now `nm -D --defined-only libmcn_arm_controller.so` lists exactly the 10 `lk_*` entry points, and the whole lk suite passes on it. Cost: 4 CMake lines and a 6-line map file per consumer that links a static library.

## Proposal

- **Library fix.** `lk_component()` links every component with a version script that like_keras ships (`global: lk_*; local: *;`, since every C ABI entry point is `lk_*`), and with `-Wl,--exclude-libs,ALL`. Then any .so it builds exports the C ABI and nothing else, whatever it links, and no consumer writes those lines. The runtime's own 5 symbols go with it.
- **New ruler test.** For every .so that `lk_component` builds in `kpi/` (cpp_gain, arm_controller, empty, echo, ...) and for one that links a static library with default visibility, `nm -D --defined-only` lists only `lk_*`. Today the count of other symbols is 5 for the reference components.
