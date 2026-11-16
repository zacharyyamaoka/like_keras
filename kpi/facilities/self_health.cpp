// CONTRACT §15 A22 (consumer-007): health(level, code) in both languages, held until called again; health reports
// the worse of the component's level and the facility's own; log.error sets health error / ErrorLogged; the C++ Log
// takes string-literal %s arguments. Four phases on the logical clock (kpi/facilities/test_facilities_friction.py runs it in
// lockstep, with health rounds at 1 s, 2 s, ...), the same as kpi/facilities/friction_parts.py's SelfHealth.
#include <like_keras/lk.hpp>

struct SelfHealth : lk::Component {
  void forward() override {
    const double t = now();
    if (phase_ == 0) {
      health("error", "Refused");  // rounds 1 and 2: error / Refused, held
      phase_ = 1;
    } else if (phase_ == 1 && t >= 2.5) {
      health("warn", "Degraded");  // rounds 3 to 5: warn / Degraded, replacing it
      phase_ = 2;
    } else if (phase_ == 2 && t >= 5.5) {
      health("ok", "");  // rounds 6 and 7: ok
      phase_ = 3;
    } else if (phase_ == 3 && t >= 7.5) {
      log.error("refused: %s", "StartTooFar");  // round 8: error / ErrorLogged, the worse of ok and the log's
      phase_ = 4;
    }
  }
  int phase_ = 0;
};

struct SelfHealthParams {
  int unused = 0;  // LK_PARAMS reflects at least one field
};
LK_PARAMS(SelfHealthParams, unused)

lk::System self_health(const SelfHealthParams&) {
  lk::System s{"self_health"};
  s.add<SelfHealth>("subject");
  return s;
}
LK_EXPORT(self_health)
