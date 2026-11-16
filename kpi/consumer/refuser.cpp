// CONTRACT §15 A34 (consumer-017): a refused activate() is final under dora too. This box refuses in activate,
// every time, as the controller does when another process holds its bus (T7): before it throws it appends one line
// to starts_path, so a test counts how often its node was started (dora's restart_policy must not bring it back).
// Driven by kpi/consumer/test_consumer_friction.py, real time and lockstep.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

#include <cstdio>
#include <stdexcept>
#include <string>

struct Refuser : lk::Component {
  struct Params {
    std::string starts_path;
    std::string reason = "the bus lock is held by kpi_holder";
  };
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit Refuser(const Params& params) : params_(params) {}

  void activate() override {
    if (std::FILE* file = std::fopen(params_.starts_path.c_str(), "a")) {
      std::fputs("activate\n", file);
      std::fclose(file);
    }
    throw std::runtime_error("REFUSED: " + params_.reason);
  }

  void forward() override {  // never runs: activate always refuses
    rm::JointState& out = echo.write();
    out.size = 0;
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }

  Params params_;
};
LK_PARAMS(Refuser::Params, starts_path, reason)

lk::System refuser(const Refuser::Params& params) {
  lk::System s{"refuser"};
  auto& box = s.add<Refuser>("refuser", params);
  s.expose(box.echo);
  return s;
}
LK_EXPORT(refuser)
