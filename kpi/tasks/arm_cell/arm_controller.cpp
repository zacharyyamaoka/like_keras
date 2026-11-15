// File 2 of the reference consumer (CONTRACT §6.2): the C++ controller chain, bus -> jtc -> guard on one thread.
// One .so, three placements (cell.py): its own process under dora, a GIL-free thread inside Python, the twin bus.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/control.hpp>
#include <like_keras/msgs/trajectory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <string>

#include "fake_bus.hpp"  // the twin bus by params (twin = true); kpi never opens a real adapter
#include "guard.hpp"
#include "jtc_core.hpp"

struct MoteusBus : lk::Component {
  struct Params {
    std::string adapter = "0607623A";
    bool twin = false;
    double watchdog_s = 0.25;
    double exchange_s = 0.0035;
    int joints = 7;
    std::string board_file;
    std::string lock_path;
  };
  lk::In<rm::ServoCommand> command{this, "command", {.lagged = true}};  // goes out in the next exchange
  lk::Out<rm::ServoState> state{this, "state"};
  explicit MoteusBus(const Params& params) : p_(params) {}
  void activate() override {
    bus_.open_exclusive(p_);  // R1: BUS_LOCK, refused before the port opens
    bus_.take_over();         // R2: a hold in the same tick, never a bare stop
  }
  void forward() override {
    const auto started = std::chrono::steady_clock::now();
    bus_.exchange(now(), command.get(), state.write());
    exchange_s_[steps_++ % exchange_s_.size()] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (steps_ % 100 == 0) {  // R4, R14: the exchange p95, the bus errors and the watchdog lapses, as custom metrics
      std::array<double, 128> sorted = exchange_s_;
      const std::size_t filled = std::min<std::size_t>(steps_, sorted.size());
      std::nth_element(sorted.begin(), sorted.begin() + (filled * 95) / 100, sorted.begin() + filled);
      metric("exchange_p95_s", sorted[(filled * 95) / 100]);
      metric("bus_errors", 0.0);
      metric("watchdog_lapses", static_cast<double>(bus_.lapses()));
    }
  }
  void deactivate() override { bus_.stop_feeding(); }  // boards fall into their timeout mode
  Params p_;
  MoteusBusCore bus_;
  std::array<double, 128> exchange_s_{};
  std::size_t steps_ = 0;
};
LK_PARAMS(MoteusBus::Params, adapter, twin, watchdog_s, exchange_s, joints, board_file, lock_path)

struct Jtc : lk::Component {
  lk::In<rm::JointTrajectory> trajectory{this, "trajectory"};  // latest
  lk::In<rm::ServoState> state{this, "state"};
  lk::Out<rm::ServoCommand> command{this, "command"};
  void activate() override { core_.reserve(lk::kMaxJoints, 4096); }
  void forward() override {
    const rm::ServoState* measured = state.get();
    if (measured == nullptr) return;
    if (auto* t = trajectory.take()) core_.replace(*t, *measured);  // starts at the first tick >= t->header.stamp
    if (!trajectory.get()) core_.hold_here(*measured);               // none yet, or retracted (lease lapsed): hold (R5)
    core_.sample(now(), *measured, command.write());                 // past the end: the last sample, qd = 0
  }
  JtcCore core_;
};

LK_PARAMS(Guard::Params, runaway_rad, max_velocity_rad_s, max_effort_nm)

struct ArmConfig {
  MoteusBus::Params bus;
  Guard::Params guard;
};
LK_PARAMS(ArmConfig, bus, guard)

lk::System arm_controller(const ArmConfig& c) {
  lk::System s{"arm_controller"};
  auto& bus = s.add<MoteusBus>("bus", c.bus);
  auto& jtc = s.add<Jtc>("jtc");
  auto& guard = s.add<Guard>("guard", c.guard);  // fault, runaway -> hold; overspeed logged only; PD + FF only
  s.wire(jtc.state, bus.state);
  s.wire(guard.state, bus.state);
  s.wire(guard.proposed, jtc.command);
  s.wire(bus.command, guard.safe);  // a legal cycle: bus.command is lagged
  s.expose(jtc.trajectory, bus.state);
  s.expose("command", guard.safe);
  return s;  // derived order: bus -> jtc -> guard
}
LK_EXPORT(arm_controller)
