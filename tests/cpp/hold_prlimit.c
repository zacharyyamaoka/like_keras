/* Lane cpp's hold for tests/test_io_ladder.py's raise_after_stop scenario (CONTRACT §15 A35): LD_PRELOADed into the
 * probe (tests/io_ladder_probe.py), it holds the one prlimit() that sets RLIMIT_RTTIME on process LK_HOLD_PID. That is
 * the last thing Box::raise() does for another process before busctl() decides whether to send rtkit's call, so while
 * it is held lk_stop ends realtime for that process, sweeps it and settles with no call in flight; once LK_HOLD_RELEASE
 * exists the raise goes on, and only busctl()'s check of ladder.ended keeps its call from going out. It marks
 * LK_HOLD_MARK while it holds, and holds 20 s at most. Every other call passes straight to the kernel.
 * WHY a hold and not chance (raw node, fix round 6, slow_busctl.hpp): a race shown only by chance is no test, and no
 * busctl stand-in can hold a call that is never spawned. */
#define _GNU_SOURCE
#include <fcntl.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static void hold(pid_t pid, int resource, const void* new_limit) {
  const char* held_pid = getenv("LK_HOLD_PID");
  const char* mark = getenv("LK_HOLD_MARK");
  const char* release = getenv("LK_HOLD_RELEASE");
  if (held_pid == NULL || mark == NULL || release == NULL || new_limit == NULL || resource != RLIMIT_RTTIME ||
      pid <= 0 || pid != (pid_t)atoi(held_pid))
    return;
  const int marked = open(mark, O_WRONLY | O_CREAT | O_CLOEXEC, 0644);
  if (marked >= 0) close(marked);
  const struct timespec tick = {0, 10 * 1000 * 1000};
  for (int waited = 0; waited < 2000 && access(release, F_OK) != 0; ++waited) nanosleep(&tick, NULL);
}

int prlimit(pid_t pid, enum __rlimit_resource resource, const struct rlimit* new_limit, struct rlimit* old_limit) {
  hold(pid, resource, new_limit);
  return (int)syscall(SYS_prlimit64, pid, resource, new_limit, old_limit);
}

int prlimit64(pid_t pid, enum __rlimit_resource resource, const struct rlimit64* new_limit,
              struct rlimit64* old_limit) {
  hold(pid, resource, new_limit);
  return (int)syscall(SYS_prlimit64, pid, resource, new_limit, old_limit);
}
