# ruler-009: conformance's activate-refusal case under dora expects `start()` to return, which A34 now forbids (lanes cpp, core, dora; A31-A37 library work, 2026-11-16)

`kpi/conformance/test_semantics.py::test_activate_raising_sends_nothing[dora]` (frozen at M0) cannot pass on a library that does what §15 A34 decides, and its M0.5 companion `kpi/consumer/test_consumer_friction.py::test_refused_activate_is_final_under_dora` cannot pass on one that does not. Criterion 6 (conformance on both backends) falls 1.0 → 0.8 because of it.

## What fails

The conformance case places `RefuseActivate` (a Python component whose `activate` raises) in its own dora process and calls `dep.start()` outside `pytest.raises`, then sleeps 2 s and asserts the reader got no data:

```python
        if make.backend == "inproc":
            with pytest.raises((Exception, SystemExit)):  # inproc: the launching process is the one refusing
                dep.start(until=0.5)
        else:
            dep.start()
            time.sleep(2.0)
```

Its docstring says why: "inproc: start raises; dora: the process exits non-zero". A34 changes the dora half: "a refused `activate()` is final under dora too ... and `Deployment.start()` raises naming the refusal and stops the dataflow, in real time and lockstep alike". So `dep.start()` now raises, as it does under inproc:

```
RuntimeError: a dora node refused:
refuser: activate refused: RuntimeError: refused on purpose (kpi conformance)
```

What the case judges still holds: the refused component sent no data frame (the node never reaches its first step, and with A34 it is no longer restarted at all).

Before this round it passed only because a Python node's real-time refusal marked `refused` without `failed`, so `start()` returned, and dora restarted the node after every exit, each time refusing again: consumer-017's loop, in Python (not counted here).

Reproduce (device-free sandbox, private namespace):

```bash
cd /home/bam/like_keras && nice -n 19 ionice -c3 taskset -c 22-23 env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/run.py conformance -k test_activate_raising_sends_nothing
```

## Proposed ruler change

In the dora branch, expect the raise as the inproc branch does, and keep the assertion on the reader:

```python
        with pytest.raises(Exception, match="refused on purpose"):
            dep.start(until=0.5 if make.backend == "inproc" else None)
```

Until the ruler lane makes it, that one case is a ruler defect, never a lane failure (as A29 read ruler-008).

## Not changed in the library

Keeping `start()` silent for a Python refusal would pass this case and fail A34 for every Python component, leave the Python fallback (`ArmControllerPy`) behaving unlike its C++ twin under dora, and leave an idle refused node in a dataflow `start()` reported as started. The lane test `tests/test_dora_refusal.py` pins the Python half of A34.

## Resolution (2026-11-16, dispatcher)

CONTRACT.md §15 A40: the ruler lane fixes it in the next ruler commit.
