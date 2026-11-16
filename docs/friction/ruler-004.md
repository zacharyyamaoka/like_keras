# ruler-004: two consumer tests that cannot pass as written (lane CPP, M2, 2026-11-15)

Two frozen tests cannot pass on a correct runtime. Both fixes are one line in `kpi/` and belong to the ruler lane; no builder has touched `kpi/`. The evidence scripts are scratch copies of the ruler's tests, each with that one line changed.

## 1. The T10 tests leave three of the echo box's inputs unbound (criterion 8 part 4; M2 exit T10)

- **What happens.** `test_T10_codec_roundtrip[*]` and `test_T10_trajectory_view` wire one input of `libecho.so` and leave the other three unbound. `describe()` then refuses with `t10.box.servo_state_in is not bound`, exactly as CONTRACT §4.3 check 1 says it must ("an input left unbound (unless exposed)").
- **The fix in the ruler** (`kpi/polyglot/test_polyglot.py`, after the `s.add(lk.native(...echo...))` line of each test):

  ```python
  s.expose(*[getattr(box, name) for name in ("joint_state_in", "servo_state_in", "servo_command_in", "trajectory") if name != fed])
  ```

  `fed` is the input the test wires.
- **Evidence the data path is right.**
  - With only that line added, all four tests pass, bit for bit.
  - The run is `test_t10_exposed.py` in `/tmp/claude-1000/-home-bam-bam-ws--claude-worktrees-cpp-controller-nodes-f9a521/8d63efe5-dad3-48eb-87e4-ce978b961621/scratchpad/lk-build/cpp/t10/`: 4 passed.
  - The recorded bytes of `value` and `<type>_out` equal `kpi/golden/<type>.lkf` for JointState, ServoState and ServoCommand.
  - The trajectory digest matches, NaN payload included.
- **The alternative** is for the dispatcher to amend §4.3 so that an unbound input of a native box reads as none, which it does in the C++ runtime anyway. I recommend the ruler fix: it keeps one rule for every component.

## 2. A2's "a native thread exists" check runs after the RT thread was joined

- **What happens.** `test_A2_same_planner_in_one_process` lists `/proc/self/task` after `dep.stop()`, and `stop()` has joined the RT thread by then (CONTRACT §6.3: `lk_stop` stops the RT thread).
  - So `native` is empty, and the test fails with "no native thread found".
  - It can only pass if some unrelated native thread is alive, such as OpenBLAS's pool when `OPENBLAS_NUM_THREADS` > 1. That would pass for the wrong reason.
- **The fix.** Move the `native = [...]` line into the `try:` block, after `python_threads = _python_thread_ids()`, while the deployment runs.
- **Evidence.**
  - `a2_sampled.py` in the scratch folder above runs A2 for 60 s with that one change.
  - While running there is 1 native thread, and it ran no Python: `sys.monitoring` saw no PY_START on it.

## Not a ruler defect, for whoever runs the timing tier

- A2 and T3 assume the RT thread really gets SCHED_FIFO.
- In this lane's sandbox `ulimit -r` is 0, so the kernel refuses `fifo=80`. The box logs `SCHED_FIFO 80 refused (errno 1): the RT thread runs SCHED_OTHER`.
- On the loaded host (rack powered, cores 22-23) a 45-57 ms stall of that thread made the JTC skip ahead past `guard.runaway_rad` (0.5 rad). The guard latched its hold for the rest of the run, and every later plan "missed its start".
- In a 6 s run without such a stall, every plan started on the first tick at or after its stamp, and was at the law's 0.01 s sample one tick later (`a2_which.py`).
- **So the timing tier must grant rtprio** (`ulimit -r 99`, or CAP_SYS_NICE) before A2 or T3 can say anything.

## Resolution (2026-11-15, dispatcher)

See CONTRACT.md §15 (amendments A1-A7). Decided: A7, the ruler lane fixes it in the M0.1 commit.
