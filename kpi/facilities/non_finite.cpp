// CONTRACT §15 A33 (consumer-016): lk_describe() prints a non-finite params default as NaN, Infinity or -Infinity
// (what Python's json and strtod read), and lk.native() reads them back. This box's natural defaults are the
// consumer's: NaN means "off" (a guard timeout, a run scale), +-inf an unbounded limit. It sends its params once, as
// a JointState (off, upper, lower, gain), so a test sees what lk_create gave it. Driven by
// kpi/facilities/test_facilities_friction.py.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

#include <limits>

struct NonFiniteParams {
  double off = std::numeric_limits<double>::quiet_NaN();
  double upper = std::numeric_limits<double>::infinity();
  double lower = -std::numeric_limits<double>::infinity();
  double gain = 0.5;
};
LK_PARAMS(NonFiniteParams, off, upper, lower, gain)

struct NonFinite : lk::Component {
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit NonFinite(const NonFiniteParams& params) : params_(params) {}

  void forward() override {
    if (sent_) return;
    sent_ = true;
    rm::JointState& out = echo.write();
    out.size = 4;
    out.name[0] = "off";
    out.name[1] = "upper";
    out.name[2] = "lower";
    out.name[3] = "gain";
    out.position[0] = params_.off;
    out.position[1] = params_.upper;
    out.position[2] = params_.lower;
    out.position[3] = params_.gain;
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }

  NonFiniteParams params_;
  bool sent_ = false;
};

lk::System non_finite(const NonFiniteParams& params) {
  lk::System s{"non_finite"};
  auto& box = s.add<NonFinite>("box", params);
  s.expose(box.echo);
  return s;
}
LK_EXPORT(non_finite)
