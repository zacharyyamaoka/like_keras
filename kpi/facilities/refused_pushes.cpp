// CONTRACT §15 A42 (consumer-021): a push the box refuses (a conversion that throws, a push outside the
// activate/deactivate window, more than kMaxJoints) is counted per input as refused.<port> in the component's metrics
// and logged on the box's log port (the first, then rate-limited), in both hosts. This box's input `sample` is an
// In<T, Converted> (A17) whose conversion is a plan mailbox's validation on the pushing thread: it throws on a negative
// value. The box echoes every value it takes, so a test sees which pushes were admitted. Driven by
// kpi/facilities/test_facilities_friction.py.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

#include <stdexcept>

struct Checked {
  double value = 0.0;

  static Checked from(const rm::JointState& message) {  // A17: on the pushing thread, never the RT thread
    if (message.size == 0 || !(message.position[0] >= 0.0))
      throw std::invalid_argument("kpi A42: a negative sample is refused");
    return Checked{message.position[0]};
  }
};

struct TakerParams {
  double unused = 0.0;
};
LK_PARAMS(TakerParams, unused)

struct Taker : lk::Component {
  lk::In<rm::JointState, Checked> sample{this, "sample"};
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit Taker(const TakerParams&) {}

  void forward() override {
    const Checked* checked = sample.take();
    if (checked == nullptr) return;
    rm::JointState& out = echo.write();
    out.size = 1;
    out.name[0] = "sample";
    out.position[0] = checked->value;
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }
};

lk::System refused_pushes(const TakerParams& params) {
  lk::System s{"refused_pushes"};
  auto& taker = s.add<Taker>("taker", params);
  s.expose(taker.sample, taker.echo);
  return s;
}
LK_EXPORT(refused_pushes)
