/* A fake native component for kpi/miswire and kpi/validgraphs: the C ABI of CONTRACT §6.3, describe only.
   Built once per variant (-DFAKE_OK, -DFAKE_SKEW, -DFAKE_ABI2). It can be described and wired, never started:
   lk_create refuses, naming why. Plain C, no like_keras, so it builds before the C++ runtime exists. */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "fake_native_describe.h"

struct ArrowArray;
typedef struct lk_box lk_box;
typedef struct { double hz; const int* cpus; int ncpus; int fifo; int lock_memory; int stepped; } lk_place;

#if defined(FAKE_SKEW)
#define FAKE_DESCRIBE FAKE_DESCRIBE_SKEW
#elif defined(FAKE_ABI2)
#define FAKE_DESCRIBE FAKE_DESCRIBE_ABI2
#else
#define FAKE_DESCRIBE FAKE_DESCRIBE_OK
#endif

uint32_t lk_abi_version(void) {
#if defined(FAKE_ABI2)
  return 2;
#else
  return 1;
#endif
}
const char* lk_describe(void) { return FAKE_DESCRIBE; }
lk_box* lk_create(const char* params_json, char* err, size_t err_len) {
  (void)params_json;
  if (err != NULL && err_len > 0) snprintf(err, err_len, "kpi fake native: describe only, never started");
  return NULL;
}
int lk_start(lk_box* box, const lk_place* place, char* err, size_t err_len) {
  (void)box; (void)place;
  if (err != NULL && err_len > 0) snprintf(err, err_len, "kpi fake native: describe only, never started");
  return -1;
}
int lk_step(lk_box* box, double now) { (void)box; (void)now; return -1; }
int lk_push(lk_box* box, uint32_t in_port, struct ArrowArray* msg) { (void)box; (void)in_port; (void)msg; return -1; }
int lk_take(lk_box* box, uint32_t out_port, struct ArrowArray* out) { (void)box; (void)out_port; (void)out; return 0; }
int lk_wait(lk_box* box, int timeout_ms) { (void)box; (void)timeout_ms; return 0; }
void lk_stop(lk_box* box) { (void)box; }
void lk_destroy(lk_box* box) { (void)box; }
