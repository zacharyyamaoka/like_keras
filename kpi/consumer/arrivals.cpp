// CONTRACT §15 A25 (consumer-010): the C++ half of the arrival count. take() is "only what arrived since the last
// take()", so a plan sent twice counts twice whatever its content; kpi/consumer/friction_parts.py's Arrivals must
// count the same with lk.In(T, fresh=True). Each step sends (arrived 0/1, arrivals so far) as a JointState.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <like_keras/msgs/trajectory.hpp>

struct Arrivals : lk::Component {
  lk::In<rm::JointTrajectory> plan{this, "plan"};
  lk::Out<rm::JointState> count{this, "count"};

  void forward() override {
    const bool arrived = plan.take() != nullptr;
    arrivals_ += arrived ? 1 : 0;
    rm::JointState& out = count.write();
    out.size = 2;
    out.name[0] = "arrived";
    out.name[1] = "arrivals";
    out.position[0] = arrived ? 1.0 : 0.0;
    out.position[1] = static_cast<double>(arrivals_);
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }

  long arrivals_ = 0;
};

struct ArrivalsParams {
  int unused = 0;  // LK_PARAMS reflects at least one field
};
LK_PARAMS(ArrivalsParams, unused)

lk::System arrivals(const ArrivalsParams&) {
  lk::System s{"arrivals"};
  auto& counter = s.add<Arrivals>("arrivals");
  s.expose(counter.plan, counter.count);
  return s;
}
LK_EXPORT(arrivals)
