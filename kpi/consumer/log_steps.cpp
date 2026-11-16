// CONTRACT §15 A30 (contract-007 item 3), T1, ruler M0.4: under lockstep a reader gets a native box's log line at
// the step that logged it, as it gets a Python component's, so a log tap reads a swap the same way. This box logs
// three lines on the logical clock: "line 0" (info) at 0.3 s, before the first facility round; "line 1" (warning) at
// 1.0 s, on a round; "line 2" (error) at 1.7 s, after the last round of a run that ends at 1.9 s. Its Python twin is
// kpi/consumer/friction_parts.py's LogSteps; kpi/consumer/test_consumer_friction.py drives both.
#include <like_keras/lk.hpp>

struct LogSteps : lk::Component {
  void forward() override {
    // WHY - 1e-9: logical times are sums of ticks; 15 x 0.02 need not be exactly 0.3 in binary
    if (next_ >= 3 || now() < kAt[next_] - 1e-9) return;
    const double index = next_;
    if (next_ == 0) {
      log.info("line %g", index);
    } else if (next_ == 1) {
      log.warning("line %g", index);
    } else {
      log.error("line %g", index);
    }
    ++next_;
  }
  static constexpr double kAt[3] = {0.3, 1.0, 1.7};
  int next_ = 0;
};

struct LogStepsParams {
  int unused = 0;  // LK_PARAMS reflects at least one field
};
LK_PARAMS(LogStepsParams, unused)

lk::System log_steps(const LogStepsParams&) {
  lk::System s{"log_steps"};
  s.add<LogSteps>("talker");
  return s;
}
LK_EXPORT(log_steps)
