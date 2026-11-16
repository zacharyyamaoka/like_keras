// CONTRACT §15 A20 (consumer-005): period(), the group's period in seconds (0.0 when rate-less), readable from
// activate() on, so a controller can refuse at open time when one bus exchange cannot fit in its tick (T7's shape:
// refused in activate, before any step). This box reports what period() said in activate and in forward; with
// min_period_s above the period it refuses in activate. The same as kpi/consumer/friction_parts.py's PeriodProbe.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

#include <stdexcept>

struct PeriodProbe : lk::Component {
  struct Params {
    double min_period_s = 0.0;
  };
  lk::Out<rm::JointState> report{this, "report"};

  explicit PeriodProbe(const Params& params) : params_(params) {}

  void activate() override {
    in_activate_ = period();
    if (in_activate_ < params_.min_period_s) {
      throw std::runtime_error("one bus exchange does not fit in a tick: the period is below min_period_s");
    }
  }

  void forward() override {
    rm::JointState& out = report.write();
    out.size = 2;
    out.name[0] = "activate";
    out.name[1] = "forward";
    out.position[0] = in_activate_;
    out.position[1] = period();
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }

  Params params_;
  double in_activate_ = -1.0;
};
LK_PARAMS(PeriodProbe::Params, min_period_s)

lk::System period_probe(const PeriodProbe::Params& params) {
  lk::System s{"period_probe"};
  auto& probe = s.add<PeriodProbe>("probe", params);
  s.expose(probe.report);
  return s;
}
LK_EXPORT(period_probe)
