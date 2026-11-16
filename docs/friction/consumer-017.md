# consumer-017: under dora, a controller refused in activate is restarted 30-100 times a second and `Deployment.start()` reports success (controllers lane, M3′ T7, 2026-11-16)

**The ask.** A refused `activate()` (§5.7: the bus is owned by someone else, the exchange budget does not fit, a real bus without interlocks) must reach the deployer, as it does in-process, where `start()` raises naming why. It must not become a hot restart loop on the robot's host.

## What I tried

T7 in `target`. A first deployment of the controller holds the twin's own bus lock (`twin_bus_lock`, in the run's directory). A second deployment of the same `.so` on the same lock, system `arm_cell_2`, then calls `start()`:

- in `same_box` (and in a second same_box process) `start()` raises `RuntimeError: ... REFUSED: another rack process holds the bus lock ...`;
- under dora `start()` returned normally. The native node refused, exited 1, was restarted by dora (`restart_policy: always`, §9.2), and refused again, and again.

## Contract sections

§5.7 ("activate raising: the group never starts, the process exits non-zero having sent no data frame (T7)"), §9.2 (emission: `restart_policy: always` on every node; `start()` returns once every node has activated, A12), §10 T7.

## Repro

`lk/tests/test_target.py::test_T7_a_second_node_is_refused_target` (bam_ws `claude/cpp-controller-nodes-f9a521`), sandboxed with a private dora:

```
T7 target: the second node refused activate 90 times in 3 s (dora restarts it), sent 0 frames; the first sent 426 commands meanwhile
T7 target: the second node refused activate 290 times in 3 s (dora restarts it), sent 0 frames; the first sent 407 commands meanwhile
```

That is two runs, at load 13 and 16.

The second dataflow's markers show why `start()` did not raise. `backends/dora/__init__.py` waits for `started|refused` and raises only on `failed`. `cpp/hosts/dora_host.cpp:601-603` marks `refused` and calls `refuse(1, ..., host.stepped != 0)`, so `failed` is written only under lockstep:

```
markers/<run>/refused.arm-controller   moteus_arm.chain.activate(): REFUSED: another rack process holds the bus lock ...
markers/<run>/exit.arm-controller
```

In my first version of the test I expected `start()` to raise. When it did not, the test's `finally` never stopped the second dataflow. The nodes then outlived the private daemon's `SIGTERM` and kept failing to send for 40 s (`Broken pipe`, 11 314 log lines) until I stopped them by PID. That part is my test's fault, fixed in `dora_private.PrivateDora.stop()`. It shows how quietly a refused deployment keeps running.

## What I did instead

- The T7 test now asserts what the ruler asserts: the second node sends 0 frames however often it is restarted, and the first keeps commanding. It counts the refusals in the node's log.
- The refusal itself is safe. Every attempt refuses before the port opens, and on the twin it hands back nothing.
- On the rack, a restart every 10-33 ms would mean a process start, a `dlopen`, a hello and a bus-lock attempt, 30-100 times a second, on cores the powered arm shares. It would also bury the one line that says why.

## Proposal

- **Library fix.** In real time too, `Deployment.start()` raises when a node marked `refused` and never `started`, naming the refusal (the marker's text), and stops the dataflow, as the inproc backend does. A refusal is final in both modes: the host writes `failed` on an activate refusal whatever `stepped` is.
- **Contract (§9.2).** State that supervision restarts a node that *crashed* (T6) but not one that *refused*. With dora 0.5's policies (`always`, `on-failure`, `never`), that means the host must exit a refusal with a code like_keras treats as final, or the launcher must stop the dataflow on the first `refused` marker.
- **New ruler test.** T7 in `target`: `start()` of the second deployment raises within 2 s, naming the refusal, and the second node was started at most once.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A31-A36); its frozen test comes in ruler M0.5 (A38).
