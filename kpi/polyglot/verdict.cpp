// CONTRACT §15 A23 (consumer-008): a C++ input may be optional (InOpts{.optional = true}): left unwired it is legal
// and reads nullptr. A guard's verdict is a robot_msgs DiagnosticRecord: header.stamp is the stamp of the state it
// judged, level "ok" confirms, level "error" means stop. This box is the raw node's guard deadman written against
// that seam: unguarded (the input never received anything) it moves every step; from the first verdict on it moves
// only while the newest verdict is "ok" and judged a state no older than guard_timeout_s. It sends its position as a
// JointState. Driven by kpi/polyglot/test_polyglot_friction.py in same_box and target.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <like_keras/msgs/std.hpp>

struct Deadman : lk::Component {
  struct Params {
    double guard_timeout_s = 0.3;
    double step_rad = 0.001;
  };
  lk::In<rm::DiagnosticRecord> verdict{this, "verdict", {.optional = true}};
  lk::Out<rm::JointState> command{this, "command"};

  explicit Deadman(const Params& params) : params_(params) {}

  void forward() override {
    const rm::DiagnosticRecord* judged = verdict.get();
    if (judged != nullptr) guarded_ = true;  // from the first verdict on, motion needs a fresh "ok"
    // WHY + 1e-9: logical times are sums of ticks; 1.25 - 0.95 is not exactly 0.3 in binary
    const bool confirmed = judged != nullptr && judged->level == "ok" &&
                           now() - judged->header.stamp <= params_.guard_timeout_s + 1e-9;
    if (!guarded_ || confirmed) position_ += params_.step_rad;
    rm::JointState& out = command.write();
    out.size = 1;
    out.name[0] = "j1";
    out.position[0] = position_;
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }

  Params params_;
  bool guarded_ = false;
  double position_ = 0.0;
};
LK_PARAMS(Deadman::Params, guard_timeout_s, step_rad)

lk::System guarded_arm(const Deadman::Params& params) {
  lk::System s{"guarded_arm"};
  auto& arm = s.add<Deadman>("deadman", params);
  s.expose(arm.verdict, arm.command);
  return s;
}
LK_EXPORT(guarded_arm)
