// Lane core's box for tests/test_log_readers.py (fix round 1 after the first consumer, item 3; CONTRACT §15 A28,
// consumer-012): it logs one line, once, the first step at least at_s after its first step, between two facility
// rounds. tests/test_log_readers.py's LogOnce is its Python twin.
#include <like_keras/lk.hpp>

struct LogOnce : lk::Component {
  struct Params {
    double at_s = 0.45;
  };
  explicit LogOnce(const Params& params) : params_(params) {}
  void forward() override {
    if (first_ < 0.0) first_ = now();
    if (said_ || now() - first_ < params_.at_s - 1e-9) return;
    said_ = true;
    log.error("brake held: %s", "J2");
  }
  Params params_;
  double first_ = -1.0;
  bool said_ = false;
};
LK_PARAMS(LogOnce::Params, at_s)

lk::System log_once(const LogOnce::Params& params) {
  lk::System s{"log_once"};
  s.add<LogOnce>("says", params);
  return s;
}
LK_EXPORT(log_once)
