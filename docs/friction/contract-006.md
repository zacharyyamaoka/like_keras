# contract-006: what lane dora read into §6.4, §9.2 and §9.3, and three lines it needs from lane core (M3, 2026-11-16)

From lane DORA while building M3 (`src/like_keras/backends/dora/`, `cpp/hosts/`, `src/like_keras/{attach,cli}.py`). Items 1-3 need lane core (their files); the rest are readings the dispatcher should ratify or amend. Most important first.

## Needs lane core (their files; nothing of theirs was edited)

1. **An attach proxy's inputs need no writer.** `lk.attach(address)` mirrors every exposed port, so its `In`s exist before anyone knows whether this client writes. `kpi/attach` and `kpi/support/parts.py:observe()` attach read-only (`s.add(lk.attach(...), "arm")` and wire only `arm.state` / `arm.command`), and `system.graph()` then refuses the proxy's unwired `trajectory` as `unbound`. Three of the four criterion-11 tests, T5 `attached` and T8 fail at `describe()` on that alone.
   - **Done on lane dora's side:** `attach()` marks each proxy `In` with `optional = True` (`attach.py`).
   - **Needed in `system.py`** (`graph()`, the `unbound` branch), one condition (the `elif` stays, so an unwired optional input never reaches the branch below that reads `binding.src`):
     ```python
     elif binding is None:
         if not getattr(type(leaf)._lk_ports[name], "optional", False):
             failures.append(problem("unbound", port=ref, name=name, at=leaf._lk.site))
     ```
     An unwired optional input gets no slot, and the proxy's `forward(**inputs)` simply never sees it.
2. **`health.code` must be able to say `LeaseHeld`** (§4.5, T8: "its health goes error with code LeaseHeld naming the holder"). `inproc.Run._health` hard-codes `code = "Exception"` whenever `state.error` is set, and a component has no other way to choose its code.
   - **Done on lane dora's side:** on a `REFUSED` frame the proxy logs at error level, then sets `self._lk.error = "write refused: the lease is held by <holder> (...)"` and `self._lk.code = "LeaseHeld"`; both clear one second after the last refusal.
   - **Needed in `inproc.py`** (`_health`), one expression, plus `self.code = None` in `ComponentState.reset()`:
     ```python
     level, code, message = "error", getattr(state, "code", None) or "Exception", state.error
     ```
3. **`attach.serve(run)` subscribes through `run._box(leaf).sinks[port]`.** contract-003 §3 gives the attach seam `run.inputs` and `run.outputs`, but a `Latest` cannot wake the socket thread, and `lk top` needs every component's health and metrics, which `run.outputs` does not hold. `serve()` therefore appends its own sink to each exposed `Out` and to every leaf's `health` / `metrics` (native boxes included: `_box` returns the `NativeGroup`). The dora Python host does the same for its transport sinks, and re-reads `box.state.log_sinks` after adding them.
   - **Ask:** bless `Run._box(leaf).sinks` as the seam (or add `run.subscribe(leaf, port, sink)`), so a core compaction (M4) keeps it.

## Readings (the dispatcher should ratify or amend)

4. **§6.4's list of dora calls is unbuildable as written; lk-dora-host also uses `event_as_arrow_input_with_info` and the metadata API.**
   - In dora 0.5.0's cxx API, `event_as_arrow_input` returns neither the input's id nor its metadata (`apis/c++/node/src/lib.rs:316`). A node with two inputs therefore cannot tell them apart. §9.2's lockstep tag `lk_t` rides in that metadata, so lockstep cannot read it either.
   - **Done:** `event_as_arrow_input_with_info` (id + metadata), `new_metadata`, `Metadata::set_int` / `get_int` (`lk_t`, `lk_h`), and `send_arrow_output`'s metadata overload (the same `cxx_name`). Everything §6.4 bans stays banned: the C node API, `allocate_data_sample` / `DataSampleHandle` / `send_data_sample*`, `init_dora_node_from_id` / `_flexible`, `node_id`, `dataflow_id`, operators. This list is the WHY comment at the top of `dora_host.cpp`.
   - **Amend:** §6.4's list gains those five names.
5. **Lockstep across processes sends promises, not one frame per sender step (§5.5).**
   - "The frame of the upstream's last step before t" cannot be known for a rate-less sender (the planner): its next step depends on what arrives.
   - **What is built:**
     - Each data frame carries `lk_t`, its sender's step time in int64 ns.
     - The last frame an output sends in a step also carries `lk_h`: "nothing more before this time".
     - An output with nothing to send that step gets an empty frame carrying the new promise.
     - A process steps at t once every input has promised t. A rate-less group's triggering inputs also bound what the process itself may promise.
     - A step's frames are handed to its slots only when due: strictly before t, or at t for a rate-less queue. Handed over any earlier, drop-oldest would discard values a reader is still owed.
     - This holds the §5.4 visibility rule exactly. `kpi/conformance -k dora` is 17/17, the equivalence gate 16/16 three runs in a row, and T1 holds.
   - **Limit:** two processes whose rate-less groups trigger each other would wait on each other forever. No v0 graph has that shape.
   - **Amend:** §5.5's dora bullet should read "frames carry `lk_t`; the last frame of a step, or an empty one, carries the sender's promise `lk_h`".
6. **The init watchdog and dora's start barrier.** dora 0.5.0's `init_dora_node()` returns only once every node of the dataflow has connected. The 2 s watchdog therefore also bounds how long the slowest peer may take to start. On this host the Python nodes made it every time (rack powered, cores 22-23, not the timing tier). A loaded timing box may need more than 2 s: exit 4 then means "a peer was slow", not only "a skewed daemon".
   - In Python, `Node()` holds the GIL while it waits, so no Python timer can fire. The Python hosts use `faulthandler.dump_traceback_later(2.0, exit=True)`, a C watchdog: it dumps the stack to the node's log and exits 1, where §6.4 asks the C++ host for exit 4.
   - **Decide:** keep 2 s, or bound the barrier separately (for example 10 s) from the skew check.
7. **lk_hello is an acknowledged handshake.** Each hello carries a per-incarnation `nonce` and `seen` (peer → the nonce this node verified). A node is ready once every peer's latest hello names its nonce, and it answers any nonce it has not seen with its own hello.
   - **Why:** a node restarted by `restart_policy` (T5, T6) must hear its peers again, and nothing may be sent before the peer that must receive it is listening.
   - Version and hash mismatches are final: the node exits 3 with a `failed` marker, so a waiting launcher raises at once. A peer that never answers also exits 3, without the marker, and dora restarts it.
8. **lk-dora-host reads `<process>.host`, not plan.json.** emit.py writes that file beside plan.json, holding the plan's facts for that one process as plain `key value` lines: `so`, `params`, `place`, `in`, `out` with each type's Arrow tree, and `peer`. The C++ host then needs no JSON parser, and plan.json stays the source of truth. Its command line is still `lk-dora-host <plan.json> <process>`.
9. **`start()` in real time returns once every node has activated** (or refused). It waits for `markers/<run>/started.<node>` or `refused.<node>` for up to 30 s, so a deployment is stepping when `start()` returns, as inproc's is.
   - Before this, `kpi/conformance::test_now_is_scheduled_time[dora]` saw 22 of 50 steps.
   - A node whose components cannot be rebuilt marks `failed`, and `start()` raises with its message.
10. **The gateway writes the recordings, and decides when a lockstep run is complete.** The launcher runs no components (§9.2), so `Deployment.record()` under dora is wired to the gateway node.
    - Under lockstep the gateway marks itself done once every input has promised the end, and only then does the launcher stop the dataflow. Before this, a recording lost the last frames still in flight, about one run in three.
    - Only the gateway's main thread writes a recording. Two writers once interleaved batches out of order.
    - The backend creates a recording's directory, as the gateway lives in another process.
11. **Under dora lockstep a native group needs `hz`.** lk-dora-host steps the box on its ticks. A rate-less native group is refused at `start()` with that fix. The same goes for two native groups in one process.
12. **The attach server's own records.** The socket publishes health and metrics under the System's own path, which has no facility ports of its own: `refused_writes.<input>` (§9.3), `clients`, and `InputStale` for an exposed Out that stopped arriving for five periods of its producing group (T4: "InputStale visible on lk top during the stall"). `lk top` shows them with the components' records.
13. **dataflow.yml is written as JSON.** JSON is YAML 1.2, so dora reads it, and like_keras needs no YAML library (the lean core). `kpi/churn::test_nodes_only` reads it with `yaml.safe_load`.
