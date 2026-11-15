// T10's C++ half: per-joint messages echoed unchanged, and a JointTrajectory view read back as numbers.
// Driven through lk.native by kpi/polyglot/test_polyglot.py: Python -> C++ -> Python must be bit-exact.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/control.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <like_keras/msgs/trajectory.hpp>

template <class T>
struct Echo : lk::Component {
  lk::In<T> input{this, "input"};
  lk::Out<T> output{this, "output"};
  void forward() override {
    if (const T* in = input.take()) output.write() = *in;
  }
};

// The view's own reading of a trajectory, as a JointState: t[0], t[N-1], positions[1] (the NaN payload), the last
// position, header.stamp, N, J, velocities[3]. Each value must arrive bit for bit.
struct Digest : lk::Component {
  lk::In<rm::JointTrajectory> trajectory{this, "trajectory"};
  lk::Out<rm::JointState> digest{this, "digest"};
  void forward() override {
    const rm::JointTrajectory* plan = trajectory.take();
    if (plan == nullptr) return;
    static constexpr const char* kNames[] = {"t0", "tN", "p1", "pL", "stamp", "N", "J", "v3"};
    rm::JointState& out = digest.write();
    const std::size_t samples = plan->t.size();
    const std::size_t joints = plan->positions.size() / samples;
    const double values[] = {plan->t[0], plan->t[samples - 1], plan->positions[1], plan->positions[samples * joints - 1],
                             plan->header.stamp, static_cast<double>(samples), static_cast<double>(joints),
                             plan->velocities ? (*plan->velocities)[3] : -1.0};
    out.size = 8;
    for (std::size_t index = 0; index < 8; ++index) {
      out.name[index] = kNames[index];
      out.position[index] = values[index];
    }
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = plan->header.stamp;
    out.header.frame_id = plan->header.frame_id;
  }
};

struct EchoParams {
  int unused = 0;  // LK_PARAMS reflects at least one field
};
LK_PARAMS(EchoParams, unused)

lk::System echo(const EchoParams&) {
  lk::System s{"echo"};
  auto& joint_state = s.add<Echo<rm::JointState>>("joint_state");
  auto& servo_state = s.add<Echo<rm::ServoState>>("servo_state");
  auto& servo_command = s.add<Echo<rm::ServoCommand>>("servo_command");
  auto& digest = s.add<Digest>("digest");
  s.expose("joint_state_in", joint_state.input);
  s.expose("joint_state_out", joint_state.output);
  s.expose("servo_state_in", servo_state.input);
  s.expose("servo_state_out", servo_state.output);
  s.expose("servo_command_in", servo_command.input);
  s.expose("servo_command_out", servo_command.output);
  s.expose(digest.trajectory, digest.digest);
  return s;
}
LK_EXPORT(echo)
