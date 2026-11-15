# contract-001 — robot_msgs lane (§2.3): three readings the contract left open

Lane robot_msgs, 2026-11-15. Landed on `/home/bam/robot_msgs` main @ `061d7f5` (fast-forward, not pushed, not tagged). The field names, order and annotated types are exactly §2.3's; golden tests in `tests/test_log_diagnostics_metrics.py` pin them. Three places where the text allowed more than one reading, and the one that landed. Each is a contract amendment candidate, not a blocker.

## 1. `DiagnosticRecord.code` "required when level != ok" vs the zero value

§2.3 gives `level = "unknown"` and `code = ""` as defaults *and* says the code is required whenever `level != "ok"`. Read literally, the zero value `DiagnosticRecord(header=h, path=p)` cannot be constructed, because robot_msgs validates at construction (its DESIGN principle 5).

**Landed:** a code is required for `"warn"` and `"error"`; `"unknown"` may carry one (`"PortBusy"`) or none, so the zero value constructs. io-diagnostics' own `validate()` also asks an unknown record for a code, but it is a separate conformance check, never a constructor.

**Amend to:** "required when level is warn or error; give one for unknown when the reason is known."

## 2. `DiagnosticRecord.mode` domain

§2.3 lists `initialization | operational | offline` (the §5.7 lifecycle). The type takes the field from io-diagnostics' DiagnosticStatus v0, whose `MODES` also has `maintenance` and `software_update`.

**Landed:** all five io-diagnostics modes are accepted, so a RobotDx record maps one to one. like_keras sends only the three. Anything else is refused at construction.

## 3. `robot_msgs.adopt`: what it accepts

§2.3: "marks a read-only array as library-owned". The old `_core.adopt` froze anything in place, unchecked. Made public unchecked, it would let a message alias memory a caller can still write.

**Landed:**
- A read-only array is kept as it is.
- A writeable array that owns its memory is frozen in place (the old rule).
- Refused with `ValueError`: a writeable view (`owndata` False, so its memory is another array's or a buffer's), and a view of a writeable array.
- It owns the array's **root**. numpy collapses a view's `.base` to the array holding the memory, so a row of an adopted `(N, J)` reshape points at the flat buffer, never at the reshape (measured). Adopting a view therefore hands over the whole buffer it views, and every view of that buffer is kept without a copy.

**What the core lane needs to know:**
- `inproc` over `ctypes` (`np.ctypeslib.as_array`) gives a *writeable* view of foreign memory, which is refused. Call `view.setflags(write=False)` first; that is the explicit "nothing writes this now" promise.
- `np.frombuffer(bytes)` and pyarrow `to_numpy(zero_copy_only=True)` are read-only and pass as they are.

The unchecked internal path is now `robot_msgs._core.freeze_new`.

## Not changed, noted

- `header: Header` is required (no default) on all three types, as §2.3 writes it. The existing robot_msgs types default it to `Header()`.
- ROS: the three are not mapped to `rcl_interfaces/Log` or `diagnostic_msgs/DiagnosticStatus`. They raise "no standard ROS message" like `ServoState` does.
