// Guard: the safety stage between the JTC and the bus (CONTRACT §6.2; consumer R6).
//
// - A board fault (32-50) or a runaway (|proposed - measured| > runaway_rad on any joint) latches a hold at the
//   measured position, velocity 0, feed-forward 0, for the rest of the activation.
// - Overspeed (|measured velocity| > max_velocity_rad_s) is logged once per episode, never acted on
//   (Zach, 2026-11-13: "recover, don't stop"; overspeed guards are log-only).
// - Commands stay PD + feed-forward only, capped at max_effort_nm (the datasheet cap; no integrator, R6).
// kpi/tasks/arm_cell/guard_law.py is the same rule in Python.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <like_keras/lk.hpp>
#include <like_keras/msgs/control.hpp>

struct Guard : lk::Component {
  struct Params {
    double runaway_rad = 0.5;
    double max_velocity_rad_s = 10.0;
    double max_effort_nm = 2.0;
  };
  lk::In<rm::ServoState> state{this, "state"};
  lk::In<rm::ServoCommand> proposed{this, "proposed"};
  lk::Out<rm::ServoCommand> safe{this, "safe"};

  explicit Guard(const Params& params) : params_(params) {}

  void forward() override {
    const rm::ServoState* measured = state.get();
    const rm::ServoCommand* command = proposed.get();
    if (measured == nullptr || command == nullptr) return;  // nothing to guard yet: nothing sent this step
    const std::size_t joints = measured->size;
    if (!latched_) {
      bool fault = false, runaway = false;
      for (std::size_t joint = 0; joint < joints; ++joint) {
        if (measured->fault && (*measured->fault)[joint] >= 32 && (*measured->fault)[joint] <= 50) fault = true;
        const double target = command->position[joint];
        if (!std::isnan(target) && std::fabs(target - measured->position[joint]) > params_.runaway_rad) runaway = true;
      }
      if (fault || runaway) {
        latched_ = true;
        for (std::size_t joint = 0; joint < joints; ++joint) hold_[joint] = measured->position[joint];
        if (fault) log.error("board fault: holding at the measured position");
        else log.error("runaway: holding at the measured position");
      }
    }
    bool overspeed = false;
    if (measured->velocity)
      for (std::size_t joint = 0; joint < joints; ++joint)
        overspeed |= std::fabs((*measured->velocity)[joint]) > params_.max_velocity_rad_s;
    if (overspeed && !overspeed_logged_) log.warning("overspeed above %g rad/s (logged only)", params_.max_velocity_rad_s);
    overspeed_logged_ = overspeed;
    rm::ServoCommand& out = safe.write();
    out = *command;
    if (latched_) {
      for (std::size_t joint = 0; joint < joints; ++joint) {
        out.position[joint] = hold_[joint];
        (*out.velocity)[joint] = 0.0;
        (*out.feedforward_effort)[joint] = 0.0;
      }
    }
    out.max_effort.emplace();
    for (std::size_t joint = 0; joint < joints; ++joint) (*out.max_effort)[joint] = params_.max_effort_nm;
  }

  Params params_;
  bool latched_ = false;
  bool overspeed_logged_ = false;
  std::array<double, lk::kMaxJoints> hold_{};
};
