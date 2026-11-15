/* like_keras/abi.h: the C ABI every component .so carries (CONTRACT §6.3, LK_ABI_VERSION 1). For hosts, not for
 * component authors: the transport backends' native hosts (cpp/hosts), inproc (ctypes), kpi's probes. Not public (§3).
 *
 * Port indices: `in_port` / `out_port` is the position of an exposed port among those of its direction, in the order
 * of lk_describe()'s "ports". The out ports after the exposed ones are the box's facilities, in this order: log
 * (std/LogRecord), health (std/DiagnosticRecord), metrics (std/Metrics), one record per component per take round
 * (lk_take returns them one by one, then 0, then starts the next round); `path` is the component's name in the box,
 * which the host prefixes with the box's own path.
 * lk_push(box, port, NULL) retracts that input (CONTRACT §5.4): a latest input reads as none until its next value, a
 * queue is emptied.
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
  int fifo;        /* SCHED_FIFO priority 1-99, 0: none; refused by the kernel -> SCHED_OTHER, logged at warning */
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

#ifdef __cplusplus
}
#endif

#endif /* LIKE_KERAS_ABI_H */
