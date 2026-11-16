// CONTRACT §15 A19 (consumer-004): the RT thread's scheduling ladder is the library's (SCHED_FIFO, else rtkit's
// SCHED_RR, else SCHED_OTHER, reported in the box's metrics; under rtkit RLIMIT_RTTIME soft < hard, and SIGXCPU
// demotes instead of killing). No hook for components: this box never touches its own thread. It only ticks, and
// once, spin_at_s after its first step, if its process has a finite RLIMIT_RTTIME (rtkit granted a realtime policy),
// it spins its CPU past the soft limit and short of the hard one, so the library's demotion is exercised.
// Driven by kpi/blackbox/rt_ladder_child.py.
#include <like_keras/lk.hpp>
#include <sys/resource.h>
#include <time.h>

struct Ticker : lk::Component {
  struct Params {
    double spin_at_s = 1.5;
  };
  explicit Ticker(const Params& params) : params_(params) {}

  void forward() override {
    if (first_ < 0.0) first_ = now();
    if (spun_ || now() - first_ < params_.spin_at_s) return;
    spun_ = true;
    rlimit limit{};
    if (getrlimit(RLIMIT_RTTIME, &limit) != 0 || limit.rlim_cur == RLIM_INFINITY) return;  // nothing to cross
    const double soft_s = static_cast<double>(limit.rlim_cur) * 1e-6;
    const double hard_s = limit.rlim_max == RLIM_INFINITY ? soft_s + 0.1 : static_cast<double>(limit.rlim_max) * 1e-6;
    const double spin_s = 0.5 * (soft_s + hard_s);
    if (cpu_s() < 0.0) return;
    const double begin = cpu_s();
    while (cpu_s() - begin < spin_s) {
    }
    metric("spun_s", spin_s);
  }

  static double cpu_s() {
    timespec time{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) != 0) return -1.0;
    return static_cast<double>(time.tv_sec) + 1e-9 * static_cast<double>(time.tv_nsec);
  }

  Params params_;
  double first_ = -1.0;
  bool spun_ = false;
};
LK_PARAMS(Ticker::Params, spin_at_s)

lk::System rt_ladder(const Ticker::Params& params) {
  lk::System s{"rt_ladder"};
  s.add<Ticker>("ticker", params);
  return s;
}
LK_EXPORT(rt_ladder)
