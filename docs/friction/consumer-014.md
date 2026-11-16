# consumer-014: `Converted::from(const T&)` is static, so a conversion that needs the component's config needs a process-wide global (controllers lane, re-port onto A16-A28, 2026-11-16)

**The ask.** Let an `In<T, Converted>` conversion see the component that owns the input (its params, its config built in `activate()`), on the pushing thread, as A17 intends. Today `Converted::from` gets only the message.

## What I tried

A17 moved the raw node's PlanMailbox into like_keras, and I used it as intended: the controller's `trajectory` input is now `lk::In<rm::JointTrajectory, PlanIn>`, and `PlanIn::from` runs on the pushing thread. It maps the plan's columns by `joint_names` (A16), validates it (finite times, strictly increasing, positions within each joint's limits, finite velocities and effort), clips its effort to each joint's torque cap, and copies it into an owned `mcn::Plan` (`mcn::fill_plan`, shared with the raw node). The RT thread then takes the plan over with a vector swap, so it neither copies nor allocates.

Every one of those steps except the copy needs the **chain's config**: joint names and their order, position limits, torque caps, path tolerances, and `plan_samples`. The config is the component's params (A24), turned into an `mcn::NodeConfig`. `from` is a static function with one argument, the message, so it cannot reach the component.

## Contract sections

§15 A17 ("`In<T, Converted>`: a C++ input may name a conversion type with a static `Converted from(const T&)` ... called on the pushing thread inside `lk_push`"), §4.1/§6.1 (a component never reads the environment or a registry), §15 A24 (the params are the per-joint table).

## Repro

`lk/arm_controller.cpp` on bam_ws `claude/cpp-controller-nodes-f9a521`, `PlanIn::from` and `g_admission`:

```cpp
// WHY a process-wide pointer: Converted::from(const T&) is static (CONTRACT §15 A17), and mapping a plan by name,
// checking its limits and clipping its effort need the chain's config, which only the component has.
std::atomic<const Admission*> g_admission{nullptr};
PlanIn PlanIn::from(const rm::JointTrajectory& view) {
  const Admission* admission = g_admission.load(std::memory_order_acquire);
  if (admission == nullptr) { in.refusal = Refusal::kInactive; return in; }
  ...
```

The cost of the global shows in a test, `lk/tests/test_lifecycle.py::test_a_second_chain_in_the_process_is_refused`. A second chain in one process must be refused, or one chain's plans would be validated against the other chain's joints:

```
$SBX ~/like_keras/.venv/bin/python -c "import cell; first = cell.same_box(); first.start(); \
  second = cell.same_box(); second.start(until=0.5)"        # from $PKG/lk
RuntimeError: arm_cell.arm: moteus_arm.chain.activate(): one moteus chain per process: another one is active here (it owns its bus) (CONTRACT §5.7)
```

## What I did instead

- A process-wide `std::atomic<const Admission*>` is published in `activate()` and cleared in `deactivate()`. `from` reads it. Before activation, or after deactivation, the conversion marks the plan "the chain is not active", and the RT thread logs and drops it.
- A second active chain in the same process is refused in `activate()`. That is harmless for this chain, since one chain owns one bus, as the raw node does. It is still a limit the library imposes, not one the domain does: two arms on two buses in one process could not use a config-dependent conversion.
- One race is left unsolved. `from` could still hold the pointer while the box is destroyed. Today that is safe only because no host pushes during `lk_destroy`.

A second finding, a pattern rather than a defect, is for the docs. A17 says a component "keeps what it needs past that by moving it out". A plan must outlive its input slot: it runs until the next one starts, and a plan scheduled for `header.stamp` waits several ticks. So the converted value's member must be `mutable`, and whatever the component gives back must be freed **off** the RT thread. I swap the plan's vectors with a pool slot's (`std::vector::swap`, no allocation), so the pool slot's old buffers stay behind in the converted value, and like_keras frees them with it on a host thread. Measured: 0 allocations, 0 copies and 0 conversions on the RT thread over 60 s of A2's load, 3 runs (`lk/tests/test_rt_path.py`). The copy mutant, which copies into a fresh `Plan` on the RT thread, is caught with 66 allocations and 13-14 copies.

## Proposal

- **Library fix (preferred).** The conversion takes its owner: `static Converted from(const T&, const Owner&)`, where `Owner` is the component class that declares the input. like_keras passes the component, and calls it only between `activate()` and `deactivate()`; outside that window the push is refused as now. No new public name. The config is then the component's own, and the global and the one-per-process limit go away.
- **Contract.** §15 A17's text says when `from` may run (only while the owner is operational), and documents the swap pattern for a value the component keeps past its slot.
- **New ruler test.** Two boxes of one `.so` in one process, each converting with its own params (e.g. a different joint set), must each validate against their own.
