// Lane core's box for tests/test_servo_truth.py: a real-side ServoState source and echo. ServoState.truth is a nullable
// nested GroundTruth (robot_msgs c72d664, its str -> float meta since d85aa92). Unwired, the box sends its own state
// and leaves `truth` null, as every C++ component is real-side; wired, it echoes what it received, truth included.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/control.hpp>

struct ServoTruth : lk::Component {
  lk::In<rm::ServoState> measured{this, "measured", {.optional = true}};
  lk::Out<rm::ServoState> state{this, "state"};
  void forward() override {
    ++steps_;
    rm::ServoState& out = state.write();
    if (const rm::ServoState* received = measured.get()) {
      out = *received;  // its strings point into the received buffers: the exposed Out pins them (CONTRACT §7.4)
      return;
    }
    out = rm::ServoState{};
    out.size = 2, out.name[0] = "épaule", out.name[1] = "coude", out.position[0] = 0.25, out.position[1] = -steps_;
    out.seq = static_cast<std::int64_t>(steps_), out.header.stamp = now(), out.header.frame_id = "base";
    out.header.meta.size = 1, out.header.meta.keys[0] = "side", out.header.meta.values[0] = "real";
  }
  double steps_ = 0.0;
};

struct TruthParams {
  int unused = 0;
};
LK_PARAMS(TruthParams, unused)

lk::System servo_truth(const TruthParams&) {
  lk::System s{"servo_truth"};
  auto& box = s.add<ServoTruth>("box");
  s.expose(box.measured, box.state);
  return s;
}
LK_EXPORT(servo_truth)
