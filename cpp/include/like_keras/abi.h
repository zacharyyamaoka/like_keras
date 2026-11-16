/* like_keras/abi.h: the C ABI every component .so carries (CONTRACT §6.3, LK_ABI_VERSION 1). For hosts, not for
 * component authors: the transport backends' native hosts (cpp/hosts), inproc (ctypes), kpi's probes. Not public (§3).
 *
 * Port indices: `in_port` / `out_port` is the position of an exposed port among those of its direction, in the order
 * of lk_describe()'s "ports". The out ports after the exposed ones are the box's facilities, in this order: log
 * (std/LogRecord), health (std/DiagnosticRecord), metrics (std/Metrics), one record per component per take round
 * (lk_take returns them one by one, then 0, then starts the next round); `path` is the component's name in the box,
 * which the host prefixes with the box's own path. A metrics round starts with the box's own record, path "": its
 * whole step, drops.log (log lines lost), unreleased (retired messages never released) and, while its RT thread runs,
 * sched_policy and sched_priority (CONTRACT §15 A19). After lk_stop the facility ports still hold the box's last
 * records until lk_destroy (what it logged in its last partial second and in deactivate(), a last health round, mode
 * offline, and the last metrics): a host takes them once more in between (§15 A28).
 * lk_push(box, port, NULL) retracts that input (CONTRACT §5.4): a latest input reads as none until its next value, a
 * queue is emptied.
 * An optional queue input (§15 A23) gates a step (§5.2) only once its host has pushed to it, a retraction included:
 * a host retracts each queue input it wires before lk_start, and one it leaves unwired reads NULL and gates nothing.
 *
 * A native host's boundary (CONTRACT §15 A35), after lk_start and only in real time:
 * lk_raise puts thread `thread` of process `process` (0: this one) on the box's scheduling ladder, at rtkit's
 * SCHED_RR `below` the RT thread's priority, and only while the RT thread is realtime: a thread of this process is a
 * host I/O thread (the box record's io_sched_policy and io_sched_priority read the weakest, live), another process
 * gets RLIMIT_RTTIME soft < hard first. 1: realtime now. A host that blocks SIGXCPU in every thread before any exists
 * (a dedicated process) gets a sentinel thread that demotes a realtime thread past the soft limit instead of letting
 * the hard limit kill the process.
 * lk_lower demotes every realtime thread of `process` now; `final` 1: and none is raised again, the rtkit calls in
 * flight for it are waited for (1 s, then ended) and it is swept again. lk_stop does that itself for this process
 * (with a sentinel) and every process lk_raise raised: no grant outlives the stop.
 * lk_sent reports that the value just taken from exposed Out `out_port` went out (1) or was refused (0); from the
 * first report the box record carries drops.<exposed name> (the box's steps that never went out) and
 * send_gap_max_s.<exposed name> (the worst gap between two sends).
 */
#ifndef LIKE_KERAS_ABI_H
#define LIKE_KERAS_ABI_H

#include <stddef.h>
#include <stdint.h>

#define LK_ABI_VERSION 1

#ifndef ARROW_C_DATA_INTERFACE
#define ARROW_C_DATA_INTERFACE
/* The Arrow C Data Interface's array, as the Arrow project declares it (no Arrow library is linked). */
struct ArrowArray {
  int64_t length;
  int64_t null_count;
  int64_t offset;
  int64_t n_buffers;
  int64_t n_children;
  const void** buffers;
  struct ArrowArray** children;
  struct ArrowArray* dictionary;
  void (*release)(struct ArrowArray*);
  void* private_data;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lk_box lk_box;
typedef struct {
  double hz;       /* 0: rate-less, a step per queued message */
  const int* cpus; /* the RT thread's affinity; NULL or ncpus 0: the process's */
  int ncpus;
  int fifo;        /* SCHED_FIFO priority 1-99, 0: none; refused -> rtkit's SCHED_RR, else SCHED_OTHER (§15 A19) */
  int lock_memory; /* mlockall: only a dedicated controller process (a native host in cpp/hosts) sets it */
  int stepped;     /* lockstep: no RT thread; the host calls lk_step */
} lk_place;

uint32_t lk_abi_version(void);
const char* lk_describe(void);                                               /* static JSON, see the header comment */
lk_box* lk_create(const char* params_json, char* err, size_t err_len);       /* construct; no I/O; NULL + err */
int lk_start(lk_box* box, const lk_place* place, char* err, size_t err_len); /* activate, then the RT thread */
int lk_step(lk_box* box, double now);                                        /* stepped only: one step, this thread */
int lk_push(lk_box* box, uint32_t in_port, struct ArrowArray* msg);          /* never blocks; moves msg in */
int lk_take(lk_box* box, uint32_t out_port, struct ArrowArray* out);         /* never blocks; 1 = moved into *out */
int lk_wait(lk_box* box, int timeout_ms); /* I/O threads: 1 = an exposed Out has news */
void lk_stop(lk_box* box);                /* stop the RT thread, deactivate */
void lk_destroy(lk_box* box);
int lk_raise(lk_box* box, int process, int thread, int below); /* hosts only, see above (§15 A35) */
int lk_lower(lk_box* box, int process, int final);
void lk_sent(lk_box* box, uint32_t out_port, int sent);

#ifdef __cplusplus
}
#endif

#endif /* LIKE_KERAS_ABI_H */
