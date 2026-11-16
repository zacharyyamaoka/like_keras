// CONTRACT §15 A16 (consumer-001): the C++ JointTrajectory view carries the plan's joint names, so a C++ controller
// maps a plan by name, as its Python peer does. This box republishes the names of each plan it takes, with the last
// sample's position of each joint, as a JointState. Driven by kpi/polyglot/test_polyglot_friction.py, in and across processes.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <like_keras/msgs/trajectory.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <string_view>

struct PlanNames : lk::Component {
  lk::In<rm::JointTrajectory> trajectory{this, "trajectory"};
  lk::Out<rm::JointState> names{this, "names"};

  void forward() override {
    const rm::JointTrajectory* plan = trajectory.take();
    if (plan == nullptr || plan->t.empty()) return;
    const std::size_t samples = plan->t.size();
    const std::size_t joints = std::min<std::size_t>(plan->positions.size() / samples, lk::kMaxJoints);
    rm::JointState& out = names.write();
    out.size = joints;
    for (std::size_t joint = 0; joint < joints; ++joint) {
      // WHY a copy into storage this box owns: the view's names point into the received buffer, valid for the step
      // only (A16), and the JointState goes out after it.
      const std::string_view name = plan->joint_names[joint];
      const std::size_t length = std::min(name.size(), storage_[joint].size());
      std::memcpy(storage_[joint].data(), name.data(), length);
      out.name[joint] = std::string_view(storage_[joint].data(), length);
      out.position[joint] = plan->positions[(samples - 1) * (plan->positions.size() / samples) + joint];
    }
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = plan->header.stamp;
  }

  std::array<std::array<char, 64>, lk::kMaxJoints> storage_{};
};

struct PlanNamesParams {
  int unused = 0;  // LK_PARAMS reflects at least one field
};
LK_PARAMS(PlanNamesParams, unused)

lk::System plan_names(const PlanNamesParams&) {
  lk::System s{"plan_names"};
  auto& box = s.add<PlanNames>("plan_names");
  s.expose(box.trajectory, box.names);
  return s;
}
LK_EXPORT(plan_names)
