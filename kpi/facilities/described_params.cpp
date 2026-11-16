// CONTRACT §15 A41 (consumer-020): a native box's describe() lists every LK_PARAMS field with its default, overlaid by
// the config given, as a Python component's does (lk.native already reads the defaults from lk_describe()). This box
// has the field kinds a controller's config has: scalars of every reflected type (one a NaN, the consumer's natural
// "off"), a vector of numbers and a table of rows, each with a non-empty default. Its Python twin, with the same
// Params, is kpi/facilities/friction_parts.py DescribedParamsPy. It sends its scalars once, as a JointState, so it is a
// component that runs. Driven by kpi/facilities/test_facilities_friction.py.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

#include <limits>
#include <string>
#include <vector>

struct JointRow {
  std::string name;
  double offset = 0.0;
};
LK_PARAMS(JointRow, name, offset)

struct DescribedParams {
  double gain = 0.5;
  int retries = 3;
  bool armed = true;
  std::string adapter = "0607623A";
  double timeout_s = std::numeric_limits<double>::quiet_NaN();  // NaN = off (A33)
  std::vector<double> limits{-1.5, 1.5};
  std::vector<JointRow> joints{{"shoulder", 0.25}};
};
LK_PARAMS(DescribedParams, gain, retries, armed, adapter, timeout_s, limits, joints)

struct Described : lk::Component {
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit Described(const DescribedParams& params) : params_(params) {}

  void forward() override {
    if (sent_) return;
    sent_ = true;
    rm::JointState& out = echo.write();
    out.size = 3;
    out.name[0] = "gain";
    out.name[1] = "retries";
    out.name[2] = "timeout_s";
    out.position[0] = params_.gain;
    out.position[1] = params_.retries;
    out.position[2] = params_.timeout_s;
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }

  DescribedParams params_;
  bool sent_ = false;
};

lk::System described_params(const DescribedParams& params) {
  lk::System s{"described_params"};
  auto& described = s.add<Described>("described", params);
  s.expose(described.echo);
  return s;
}
LK_EXPORT(described_params)
