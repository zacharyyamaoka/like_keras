// JtcCore: the vendored robot_controllers JTC law behind the reference Jtc component (CONTRACT §6.2).
//
// - replace(): the new plan is COPIED into preallocated storage at once (the message may be retired after this
//   step), and becomes active at the first tick whose now() >= its header.stamp (the plan's absolute start, §5.6).
// - sample(): the active plan through the vendored law (its clock samples t = 0 on the first tick, then adds the
//   measured dt = now - previous now); past the end it holds the last sample with velocity 0.
// - hold_here(): no plan, or the plan was retracted (a lapsed write lease): hold where the arm is, latched ONCE.
//   WHY once: re-latching at the measured (sagging) pose each tick walks the arm down (robot_controllers, H3).
// kpi/tasks/arm_cell/jtc_law.py is the same law in Python; T1 holds them equal to 1e-9 rad.
#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <like_keras/lk.hpp>
#include <like_keras/msgs/control.hpp>
#include <like_keras/msgs/trajectory.hpp>

#include "joint_trajectory_controller.hpp"

class JtcCore {
 public:
  // All allocation happens here (activate), never in sample().
  void reserve(int max_joints, int max_samples) {
    max_joints_ = static_cast<std::size_t>(max_joints);
    max_samples_ = static_cast<std::size_t>(max_samples);
    for (Plan& plan : plans_) {
      plan.times.assign(max_samples_, 0.0);
      plan.positions.assign(max_samples_ * max_joints_, 0.0);
      plan.velocities.assign(max_samples_ * max_joints_, 0.0);
      plan.effort.assign(max_samples_ * max_joints_, 0.0);
    }
  }

  // Returns false (the plan refused, nothing changed) when it is empty or larger than the reserved storage.
  bool replace(const rm::JointTrajectory& trajectory, const rm::ServoState& /*state*/) {
    const std::size_t samples = trajectory.t.size();
    if (samples == 0 || samples > max_samples_) return false;
    const std::size_t joints = trajectory.positions.size() / samples;
    if (joints == 0 || joints > max_joints_ || joints * samples != trajectory.positions.size()) return false;
    Plan& plan = plans_[1 - active_slot_];
    plan.samples = samples;
    plan.joints = joints;
    plan.stamp = trajectory.header.stamp;
    std::copy(trajectory.t.begin(), trajectory.t.end(), plan.times.begin());
    std::copy(trajectory.positions.begin(), trajectory.positions.end(), plan.positions.begin());
    plan.has_velocities = trajectory.velocities.has_value();
    if (plan.has_velocities) std::copy(trajectory.velocities->begin(), trajectory.velocities->end(), plan.velocities.begin());
    plan.has_effort = trajectory.effort.has_value();
    if (plan.has_effort) std::copy(trajectory.effort->begin(), trajectory.effort->end(), plan.effort.begin());
    pending_ = true;
    return true;
  }

  void hold_here(const rm::ServoState& state) {
    pending_ = false;  // a plan taken and then retracted before its start never runs
    if (holding_) return;
    for (std::size_t joint = 0; joint < state.size; ++joint) hold_[joint] = state.position[joint];
    holding_ = true;
    active_ = false;
  }

  void sample(double now, const rm::ServoState& state, rm::ServoCommand& command) {
    double dt = now - last_now_;
    if (pending_ && now >= plans_[1 - active_slot_].stamp) {
      active_slot_ = 1 - active_slot_;
      const Plan& plan = plans_[active_slot_];
      robot_controllers::TrajectoryView view;
      view.times = plan.times.data();
      view.positions = plan.positions.data();
      view.velocities = plan.has_velocities ? plan.velocities.data() : nullptr;
      view.effort = plan.has_effort ? plan.effort.data() : nullptr;
      view.samples = plan.samples;
      view.joints = plan.joints;
      law_.replace_input(view, nullptr, nullptr);
      pending_ = false;
      active_ = true;
      holding_ = false;
      dt = 0.0;  // the law's first tick after an input samples t = 0 whatever dt is
    }
    last_now_ = now;
    if (!active_ && !holding_) {  // a plan waits for its start and nothing ran yet: hold where the arm is
      for (std::size_t joint = 0; joint < state.size; ++joint) hold_[joint] = state.position[joint];
      holding_ = true;
    }
    command.size = state.size;
    command.velocity.emplace();
    command.feedforward_effort.emplace();
    command.stiffness.reset();
    command.damping.reset();
    command.max_effort.reset();
    if (active_ && law_.tick(nullptr, dt)) {
      for (std::size_t joint = 0; joint < state.size; ++joint) {
        command.position[joint] = law_.position()[joint];
        (*command.velocity)[joint] = law_.velocity()[joint];
        (*command.feedforward_effort)[joint] = law_.feedforward_effort()[joint];
      }
    } else {
      for (std::size_t joint = 0; joint < state.size; ++joint) {
        command.position[joint] = hold_[joint];
        (*command.velocity)[joint] = 0.0;
        (*command.feedforward_effort)[joint] = 0.0;
      }
    }
    for (std::size_t joint = 0; joint < state.size; ++joint) command.name[joint] = state.name[joint];
    command.state_stamp = state.header.stamp;
    command.state_seq = state.seq;
    command.header.stamp = now;
    command.header.frame_id = std::string_view{};
  }

 private:
  struct Plan {
    std::vector<double> times, positions, velocities, effort;
    std::size_t samples = 0, joints = 0;
    double stamp = 0.0;
    bool has_velocities = false, has_effort = false;
  };
  std::array<Plan, 2> plans_{};
  int active_slot_ = 0;
  bool pending_ = false, active_ = false, holding_ = false;
  double last_now_ = 0.0;
  std::size_t max_joints_ = 0, max_samples_ = 0;
  std::array<double, lk::kMaxJoints> hold_{};
  robot_controllers::JointTrajectoryController law_{};
};
