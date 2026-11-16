// CONTRACT §15 A22 with §4.5, ruler M0.4: health reports the worse of the component's own health(level, code) and
// the facility's; on a TIE the component's code wins and the facility's message is kept. WHY (library verifier,
// round 1): clearing the message on a tie left the attach proxy's LeaseHeld health without the holder's name (§4.5:
// "naming the holder"), since the name travels only in its log.error line. Three phases on the logical clock
// (lockstep, health rounds at 1 s, 2 s, ...), the same as kpi/facilities/friction_parts.py's TieHealth:
//   round 1  health("error", "Braked")                          error / Braked, no message
//   round 2  log.error("brake held: %s", "J2") at 1.5 s         a tie of errors: Braked, "brake held: J2"
//   round 3  health("warn", "Slipping") and a warning at 2.5 s  a tie of warnings: Slipping, "slipping: J3"
//   round 4  nothing logged                                     warn / Slipping, held, no message
#include <like_keras/lk.hpp>

struct TieHealth : lk::Component {
  void forward() override {
    const double t = now();
    if (phase_ == 0) {
      health("error", "Braked");
      phase_ = 1;
    } else if (phase_ == 1 && t >= 1.5) {
      log.error("brake held: %s", "J2");
      phase_ = 2;
    } else if (phase_ == 2 && t >= 2.5) {
      health("warn", "Slipping");
      log.warning("slipping: %s", "J3");
      phase_ = 3;
    }
  }
  int phase_ = 0;
};

struct TieHealthParams {
  int unused = 0;  // LK_PARAMS reflects at least one field
};
LK_PARAMS(TieHealthParams, unused)

lk::System tie_health(const TieHealthParams&) {
  lk::System s{"tie_health"};
  s.add<TieHealth>("subject");
  return s;
}
LK_EXPORT(tie_health)
