// CONTRACT §15 A23 (consumer-008) for a QUEUE input, ruler M0.4: an optional C++ In left unwired is legal and reads
// nullptr, so it must never hold the gather of §5.2 (a component with queue inputs steps when each holds a message).
// Two children, each reporting every step it runs as a JointState [steps so far, 1 if its input read nullptr else 0,
// the value it read]:
//   idle   an optional queue input neither wired nor exposed inside the box: it steps every tick and reads nullptr
//   gated  an optional queue input exposed as `extra`: left unwired by the deployment it steps every tick and reads
//          nullptr; wired, it gates as any queue input does: one step per message, reading that message (A30)
// Driven by kpi/polyglot/optional_queue_child.py (kpi/polyglot/test_polyglot_friction.py), in and across processes.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

namespace {
void report(rm::JointState& out, double steps, const rm::JointState* read, double stamp) {
  out.size = 3;
  out.name[0] = "steps";
  out.name[1] = "read_nullptr";
  out.name[2] = "value";
  out.position[0] = steps;
  out.position[1] = read == nullptr ? 1.0 : 0.0;
  out.position[2] = read == nullptr ? -1.0 : read->position[0];
  out.velocity.reset();
  out.effort.reset();
  out.header.stamp = stamp;
}
}  // namespace

struct Idle : lk::Component {
  lk::In<rm::JointState> spare{this, "spare", {.queue = 4, .optional = true}};
  lk::Out<rm::JointState> idle_steps{this, "idle_steps"};
  void forward() override { report(idle_steps.write(), ++steps_, spare.get(), now()); }
  double steps_ = 0.0;
};

struct Gated : lk::Component {
  lk::In<rm::JointState> extra{this, "extra", {.queue = 4, .optional = true}};
  lk::Out<rm::JointState> gated_steps{this, "gated_steps"};
  void forward() override { report(gated_steps.write(), ++steps_, extra.get(), now()); }
  double steps_ = 0.0;
};

struct OptionalQueueParams {
  int unused = 0;  // LK_PARAMS reflects at least one field
};
LK_PARAMS(OptionalQueueParams, unused)

lk::System optional_queue(const OptionalQueueParams&) {
  lk::System s{"optional_queue"};
  auto& idle = s.add<Idle>("idle");
  auto& gated = s.add<Gated>("gated");
  s.expose(gated.extra, idle.idle_steps, gated.gated_steps);
  return s;
}
LK_EXPORT(optional_queue)
