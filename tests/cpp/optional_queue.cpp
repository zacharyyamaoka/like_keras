// Lane cpp's box for tests/test_optional_queue.py (CONTRACT §15 A23, fix round 1 after the first consumer, item 1): an
// optional QUEUE input, exposed. Left unwired by the deployment it must gate nothing and read nullptr; wired, it gates
// the gather of §5.2 as any queue input does. `ticks` reports each step: its count, 1 when `extra` read nullptr, and
// the position of the message it read.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

struct Stepper : lk::Component {
  lk::In<rm::JointState> extra{this, "extra", {.queue = 4, .optional = true}};
  lk::Out<rm::JointState> ticks{this, "ticks"};
  void forward() override {
    ++steps_;
    const rm::JointState* message = extra.get();
    rm::JointState& out = ticks.write();
    out.size = 3, out.name[0] = "steps", out.name[1] = "extra_null", out.name[2] = "extra_position";
    out.position[0] = steps_, out.position[1] = message == nullptr ? 1.0 : 0.0;
    out.position[2] = message == nullptr ? 0.0 : message->position[0];
    out.velocity.reset(), out.effort.reset(), out.header.stamp = now();
  }
  double steps_ = 0.0;
};

struct QueueParams {
  int unused = 0;
};
LK_PARAMS(QueueParams, unused)

lk::System optional_queue(const QueueParams&) {
  lk::System s{"optional_queue"};
  auto& stepper = s.add<Stepper>("stepper");
  s.expose(stepper.extra, stepper.ticks);
  return s;
}
LK_EXPORT(optional_queue)
