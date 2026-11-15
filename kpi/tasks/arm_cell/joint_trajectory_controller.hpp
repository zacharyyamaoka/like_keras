// VENDORED, unmodified below the next blank line, from robot_controllers (branch claude/cpp-impl, commit 30307c3,
// src/robot_controllers/controllers/cpp/include/robot_controllers/joint_trajectory_controller.hpp), the JTC law
// pinned bit-exact to robot_controllers' Python JointTrajectoryController by its tests/test_cpp_parity.py.
// Credit: the robot_controllers C++ lane. Vendored so kpi/ is self-contained (the ruler never imports a peer repo).

// robot_controllers: the joint_trajectory_controller law in C++17, header-only (CONTRACT §2, amendment 2).
//
// The same black box as controllers/joint_trajectory.py:JointTrajectoryController, a second implementation:
// a time-driven clock (each tick adds the measured dt; the first tick after an input samples t = 0), the
// trajectory sampled there (linear in position, velocity and effort between samples, the ends held), the end
// held at rest with status done, and the optional path tolerance that aborts and latches a hold at the
// measured position. tests/test_cpp_parity.py pins it to the Python law to 1e-12 (it is bit-exact).
//
// WHY header-only and allocation-free: the same header runs inside a real-time control thread (the moteus
// controller node in bam_ws) and behind the ctypes shim (jtc_capi.cpp). Storage is fixed capacity
// (kMaxJoints), the trajectory is a non-owning view, and tick() never allocates, locks or does I/O.
//
// WHY `(1 - w) * a + w * b` with -ffp-contract=off: it is the Python sampler's form, and both endpoints come
// back bit-exact; a fused multiply-add would round differently and break parity with the reference law.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace robot_controllers {

// A clock this close to the trajectory's end counts as at the end (summed dts carry float round-off).
inline constexpr double kEndToleranceS = 1e-9;

enum class Status : int { kRunning = 0, kDone = 1, kAborted = 2 };

inline const char* status_name(Status status) {
  switch (status) {
    case Status::kRunning: return "running";
    case Status::kDone: return "done";
    case Status::kAborted: return "aborted";
  }
  return "unknown";
}

// One robot_msgs JointTrajectory, not owned: `times` has `samples` entries; each per-sample block is
// `samples * joints` doubles, sample-major (robot_msgs' (N, J) C-order array, `positions.ravel()`).
// `velocities` and `effort` may be null (absent); the caller keeps every array alive while it is loaded.
struct TrajectoryView {
  const double* times = nullptr;
  const double* positions = nullptr;
  const double* velocities = nullptr;
  const double* effort = nullptr;
  std::size_t samples = 0;
  std::size_t joints = 0;
};

enum class LoadError : int {
  kNone = 0,
  kEmpty = 1,          // no samples: nothing to follow
  kNoJoints = 2,
  kTooManyJoints = 3,  // more than kMaxJoints
  kMissingArray = 4,   // times or positions null
};

// Time since the active input began (s). Each tick adds its measured dt, except the first tick after an
// input arrives, which samples t = 0 (CONTRACT amendment 1.6: that dt elapsed before the input began).
class TrajectoryClock {
 public:
  void restart() {
    time_ = 0.0;
    fresh_ = true;
  }

  double advance(double dt) {
    if (fresh_) {
      fresh_ = false;
    } else {
      time_ += dt;
    }
    return time_;
  }

  double time() const { return time_; }

 private:
  double time_ = 0.0;
  bool fresh_ = true;
};

// The measured period must be finite and not negative (NaN fails `>=`).
inline bool valid_dt(double dt) { return dt >= 0.0 && dt != std::numeric_limits<double>::infinity(); }

class JointTrajectoryController {
 public:
  static constexpr std::size_t kMaxJoints = 64;

  // Build on a trajectory (Python: `JointTrajectoryController(trajectory, config)`): active_input stays 0.
  // `path_tolerance_rad` and `max_effort_nm` are one value per joint in the trajectory's order, or null
  // (no guard / the board's own cap). A refused input changes nothing.
  LoadError load(const TrajectoryView& trajectory, const double* path_tolerance_rad, const double* max_effort_nm) {
    const LoadError error = check(trajectory);
    if (error != LoadError::kNone) return error;
    trajectory_ = trajectory;
    joints_ = trajectory.joints;
    end_time_ = trajectory.times[trajectory.samples - 1];
    start_time_ = trajectory.times[0];
    has_tolerance_ = path_tolerance_rad != nullptr;
    has_max_effort_ = max_effort_nm != nullptr;
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      tolerance_[joint] = has_tolerance_ ? path_tolerance_rad[joint] : 0.0;
      max_effort_[joint] = has_max_effort_ ? max_effort_nm[joint] : 0.0;
      position_[joint] = 0.0;
      velocity_[joint] = 0.0;
      effort_[joint] = 0.0;
    }
    hold_latched_ = false;
    worst_joint_ = -1;
    worst_error_ = 0.0;
    status_ = Status::kRunning;
    loaded_ = true;
    return LoadError::kNone;
  }

  // Run `trajectory` from now: its t = 0 is the next tick. Accepted in any status (an abort included).
  LoadError replace_input(const TrajectoryView& trajectory, const double* path_tolerance_rad,
                          const double* max_effort_nm) {
    const LoadError error = load(trajectory, path_tolerance_rad, max_effort_nm);
    if (error != LoadError::kNone) return error;
    clock_.restart();
    ++active_input_;
    return LoadError::kNone;
  }

  // One control tick. `measured_positions` (one per joint, the trajectory's order) is read only when a path
  // tolerance is set; it may be null otherwise. Returns false, changing nothing, on an invalid dt.
  bool tick(const double* measured_positions, double dt) {
    if (!valid_dt(dt) || !loaded_) return false;
    const double time = clock_.advance(dt);
    if (hold_latched_) return true;  // latched at the abort: keep commanding the hold
    sample(time);
    if (time + kEndToleranceS >= end_time_) {
      // WHY at rest: the hold after the end must not carry the last sample's velocity into the board's PD.
      for (std::size_t joint = 0; joint < joints_; ++joint) velocity_[joint] = 0.0;
      if (status_ == Status::kRunning) status_ = Status::kDone;
    }
    if (has_tolerance_) guard(measured_positions);
    return true;
  }

  // The command of the last tick, one value per joint in the trajectory's order.
  const double* position() const { return position_.data(); }
  const double* velocity() const { return velocity_.data(); }
  const double* feedforward_effort() const { return effort_.data(); }
  const double* max_effort() const { return has_max_effort_ ? max_effort_.data() : nullptr; }

  Status status() const { return status_; }
  bool loaded() const { return loaded_; }
  bool hold_latched() const { return hold_latched_; }
  std::size_t joints() const { return joints_; }
  double clock_s() const { return clock_.time(); }
  double end_time() const { return end_time_; }
  int active_input() const { return active_input_; }
  // Set when the path tolerance aborted: the joint furthest off the path and its error (rad).
  int worst_joint() const { return worst_joint_; }
  double worst_error() const { return worst_error_; }

 private:
  static LoadError check(const TrajectoryView& trajectory) {
    if (trajectory.samples == 0) return LoadError::kEmpty;
    if (trajectory.joints == 0) return LoadError::kNoJoints;
    if (trajectory.joints > kMaxJoints) return LoadError::kTooManyJoints;
    if (trajectory.times == nullptr || trajectory.positions == nullptr) return LoadError::kMissingArray;
    return LoadError::kNone;
  }

  void copy_row(const double* block, std::size_t row, std::array<double, kMaxJoints>& out) const {
    const double* source = block + row * joints_;
    for (std::size_t joint = 0; joint < joints_; ++joint) out[joint] = source[joint];
  }

  void zero(std::array<double, kMaxJoints>& out) const {
    for (std::size_t joint = 0; joint < joints_; ++joint) out[joint] = 0.0;
  }

  // TrajectorySampler.sample: the first and last samples held outside the trajectory, linear inside it.
  void sample(double time) {
    const std::size_t samples = trajectory_.samples;
    if (time >= end_time_ || samples == 1) {
      const std::size_t last = samples - 1;
      copy_row(trajectory_.positions, last, position_);
      if (trajectory_.velocities != nullptr) copy_row(trajectory_.velocities, last, velocity_); else zero(velocity_);
      if (trajectory_.effort != nullptr) copy_row(trajectory_.effort, last, effort_); else zero(effort_);
      return;
    }
    if (time < start_time_) {
      copy_row(trajectory_.positions, 0, position_);
      if (trajectory_.velocities != nullptr) copy_row(trajectory_.velocities, 0, velocity_); else zero(velocity_);
      if (trajectory_.effort != nullptr) copy_row(trajectory_.effort, 0, effort_); else zero(effort_);
      return;
    }
    // start <= time < end, so lower is in [0, N - 2]: Python's bisect_right(times, time) - 1.
    const double* times = trajectory_.times;
    const std::size_t lower = static_cast<std::size_t>(std::upper_bound(times, times + samples, time) - times) - 1;
    const std::size_t upper = lower + 1;
    const double span = times[upper] - times[lower];
    const double weight = (time - times[lower]) / span;
    const double keep = 1.0 - weight;
    const double* positions_lower = trajectory_.positions + lower * joints_;
    const double* positions_upper = trajectory_.positions + upper * joints_;
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      position_[joint] = keep * positions_lower[joint] + weight * positions_upper[joint];
    }
    if (trajectory_.velocities == nullptr) {
      // WHY the segment slope, not zero: a zero velocity target makes the board's damping brake the motion.
      for (std::size_t joint = 0; joint < joints_; ++joint) {
        velocity_[joint] = (positions_upper[joint] - positions_lower[joint]) / span;
      }
    } else {
      const double* velocities_lower = trajectory_.velocities + lower * joints_;
      const double* velocities_upper = trajectory_.velocities + upper * joints_;
      for (std::size_t joint = 0; joint < joints_; ++joint) {
        velocity_[joint] = keep * velocities_lower[joint] + weight * velocities_upper[joint];
      }
    }
    if (trajectory_.effort == nullptr) {
      zero(effort_);
    } else {
      const double* effort_lower = trajectory_.effort + lower * joints_;
      const double* effort_upper = trajectory_.effort + upper * joints_;
      for (std::size_t joint = 0; joint < joints_; ++joint) {
        effort_[joint] = keep * effort_lower[joint] + weight * effort_upper[joint];
      }
    }
  }

  // Path tolerance: any joint off the path (or a NaN measurement) aborts and latches the hold.
  void guard(const double* measured) {
    std::array<double, kMaxJoints> error{};
    std::array<bool, kMaxJoints> off_path{};
    bool any_off = false;
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      error[joint] = std::fabs(position_[joint] - measured[joint]);
      off_path[joint] = !(error[joint] <= tolerance_[joint]);  // a NaN measurement counts as off the path
      any_off = any_off || off_path[joint];
    }
    if (!any_off) return;
    // numpy argmax(where(off_path, nan_to_num(error, nan=inf), -1)): the first of the largest.
    double worst_value = -std::numeric_limits<double>::infinity();
    int worst = -1;
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      const double value = off_path[joint] ? (std::isnan(error[joint]) ? std::numeric_limits<double>::infinity()
                                                                       : error[joint])
                                           : -1.0;
      if (worst < 0 || value > worst_value) {
        worst_value = value;
        worst = static_cast<int>(joint);
      }
    }
    worst_joint_ = worst;
    worst_error_ = error[static_cast<std::size_t>(worst)];
    status_ = Status::kAborted;
    // WHY latched once, not re-read each tick: re-latching at the measured (sagging) pose walks the arm down
    // one sag per tick (bam_homing H3). Feed-forward 0 as bam_controllers v1's fault hold.
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      position_[joint] = measured[joint];
      velocity_[joint] = 0.0;
      effort_[joint] = 0.0;
    }
    hold_latched_ = true;
  }

  TrajectoryView trajectory_{};
  std::size_t joints_ = 0;
  double start_time_ = 0.0;
  double end_time_ = 0.0;
  TrajectoryClock clock_{};
  Status status_ = Status::kRunning;
  bool loaded_ = false;
  bool has_tolerance_ = false;
  bool has_max_effort_ = false;
  bool hold_latched_ = false;
  int active_input_ = 0;
  int worst_joint_ = -1;
  double worst_error_ = 0.0;
  std::array<double, kMaxJoints> tolerance_{};
  std::array<double, kMaxJoints> max_effort_{};
  std::array<double, kMaxJoints> position_{};
  std::array<double, kMaxJoints> velocity_{};
  std::array<double, kMaxJoints> effort_{};
};

}  // namespace robot_controllers
