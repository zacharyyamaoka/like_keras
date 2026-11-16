// Guard: the safety stage between the JTC and the bus (CONTRACT §6.2; consumer R6).
//
// - A board fault (32-50) latches a hold at the measured position, velocity 0, feed-forward 0, for the rest of the
//   activation.
// - A runaway (|proposed - measured| > runaway_rad on any joint) holds the same way until the next plan that starts
//   within start_tolerance_rad of the measured position on every joint; then it accepts that plan and logs the
//   release. WHY recover, not latch (Zach, 2026-11-13: "robots recover, they don't stop"; CONTRACT §15 A14): a
//   latched runaway hold ended every later plan of the run (M2 verifier round 2, ruler-006 §2). The guard sees the
//   JTC's proposal, not the plan: the JTC samples a new plan's t = 0 on its first tick, so a plan that starts where
//   the arm is shows as a proposal within the tolerance, and such a proposal never steps the arm, whichever plan
//   it comes from. A NaN target is no start (the hold stays).
// - Overspeed (|measured velocity| > max_velocity_rad_s) is logged once per episode, never acted on
//   (Zach, 2026-11-13: "recover, don't stop"; overspeed guards are log-only).
// - Commands stay PD + feed-forward only, capped at max_effort_nm (the datasheet cap; no integrator, R6).
// kpi/tasks/arm_cell/guard_law.py is the same rule in Python; T1 holds them equal to 1e-9 rad.
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
    // WHY 3°: arm-hold's READY tolerance, the start tolerance moteus_controller_node refuses a far first sample with.
    double start_tolerance_rad = 0.0524;
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
    if (holding_ == Hold::kRunaway && starts_here(*measured, *command)) {
      holding_ = Hold::kNone;
      log.info("runaway hold released: a plan starts within %g rad of the measured position",
               params_.start_tolerance_rad);
    }
    if (holding_ == Hold::kNone) {
      bool fault = false, runaway = false;
      for (std::size_t joint = 0; joint < joints; ++joint) {
        if (measured->fault && (*measured->fault)[joint] >= 32 && (*measured->fault)[joint] <= 50) fault = true;
        const double target = command->position[joint];
        if (!std::isnan(target) && std::fabs(target - measured->position[joint]) > params_.runaway_rad) runaway = true;
      }
      if (fault || runaway) {
        holding_ = fault ? Hold::kFault : Hold::kRunaway;
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
    if (holding_ != Hold::kNone) {
      for (std::size_t joint = 0; joint < joints; ++joint) {
        out.position[joint] = hold_[joint];
        (*out.velocity)[joint] = 0.0;
        (*out.feedforward_effort)[joint] = 0.0;
      }
    }
    out.max_effort.emplace();
    for (std::size_t joint = 0; joint < joints; ++joint) (*out.max_effort)[joint] = params_.max_effort_nm;
  }

  // A proposal within start_tolerance_rad of the measured position on every joint. WHY `!(... <= ...)`: a NaN
  // target compares false, so it is no start, exactly as numpy's `<=` in guard_law.py.
  bool starts_here(const rm::ServoState& measured, const rm::ServoCommand& command) const {
    for (std::size_t joint = 0; joint < measured.size; ++joint)
      if (!(std::fabs(command.position[joint] - measured.position[joint]) <= params_.start_tolerance_rad)) return false;
    return true;
  }

  enum class Hold { kNone, kFault, kRunaway };
  Params params_;
  Hold holding_ = Hold::kNone;
  bool overspeed_logged_ = false;
  std::array<double, lk::kMaxJoints> hold_{};
};
