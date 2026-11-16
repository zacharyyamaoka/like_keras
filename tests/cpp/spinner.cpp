// Lane cpp's spinner for tests/test_rt_ladder.py (CONTRACT §15 A19), in the raw-dora node's rttime_probe shape (fix
// round 4): once, spin_at_s after its first step, it spins its own CPU time until its thread is no longer realtime
// (the library demoted it on SIGXCPU) or for cap_s, past the hard RLIMIT_RTTIME, where the kernel's SIGKILL ends the
// process if nothing demoted it. WHY not a wall-clock spin: on a shared core it may never cross the limit.
#include <sched.h>
#include <time.h>

#include <like_keras/lk.hpp>

struct Spinner : lk::Component {
  struct Params {
    double spin_at_s = 1.0;
    double cap_s = 0.3;
  };
  explicit Spinner(const Params& params) : params_(params) {}

  void forward() override {
    if (first_ < 0.0) first_ = now();
    if (spun_ || now() - first_ < params_.spin_at_s) return;
    spun_ = true;
    const double begin = cpu_s();
    while (cpu_s() - begin < params_.cap_s && realtime()) {
    }
    metric("spun_s", cpu_s() - begin);
  }

  static bool realtime() {
    const int policy = ::sched_getscheduler(0) & ~SCHED_RESET_ON_FORK;
    return policy == SCHED_FIFO || policy == SCHED_RR;
  }
  static double cpu_s() {
    timespec time{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time);
    return static_cast<double>(time.tv_sec) + 1e-9 * static_cast<double>(time.tv_nsec);
  }

  Params params_;
  double first_ = -1.0;
  bool spun_ = false;
};
LK_PARAMS(Spinner::Params, spin_at_s, cap_s)

lk::System spinner(const Spinner::Params& params) {
  lk::System s{"spinner"};
  s.add<Spinner>("spinner", params);
  return s;
}
LK_EXPORT(spinner)
