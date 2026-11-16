// Lane cpp's box for CONTRACT §15 A44 where the frozen ruler (kpi/facilities/partial_rows.cpp, one flat table) does not
// reach: a table inside a nested aggregate ("arm.joints"), a table inside a table's row ("arm.joints[].limits", with
// a default row of its own) and a table at the top ("stops"). Every row type's member defaults differ from every
// default row's, and from a zeroed row, so a key a given row leaves out shows what filled it. The box sends what it
// runs once, as a JointState: one leaf per joint slot, the path as the name (a string leaf: "path=value") and a
// number leaf's value as the position. Driven by tests/cpp/test_friction.cpp (the C ABI) and
// tests/test_native_params.py (lk.native and describe()).
#include <algorithm>
#include <cstddef>
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <string>
#include <utility>
#include <vector>

struct Limit {
  double low = -1.0;
  double high = 1.0;
};
LK_PARAMS(Limit, low, high)

struct Joint {
  std::string name = "spare";
  double offset = 0.125;
  std::vector<Limit> limits{{-2.0, 2.0}};  // a table inside a row: a given row that leaves it out runs this one
};
LK_PARAMS(Joint, name, offset, limits)

struct Arm {
  std::vector<Joint> joints{{"shoulder", 0.25, {{-3.0, 3.0}, {-4.0, 4.0}}}, {"elbow", -0.5, {}}};
};
LK_PARAMS(Arm, joints)

struct RowTablesParams {
  Arm arm;
  std::vector<Limit> stops{{0.25, 0.75}};
};
LK_PARAMS(RowTablesParams, arm, stops)

struct RowTables : lk::Component {
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit RowTables(const RowTablesParams& params) {  // the leaves, built here: forward() only points at them
    const auto limits = [&](const std::string& table, const std::vector<Limit>& rows) {
      for (std::size_t row = 0; row < rows.size(); ++row) {
        const std::string at = table + "[" + std::to_string(row) + "].";
        leaves.emplace_back(at + "low", rows[row].low), leaves.emplace_back(at + "high", rows[row].high);
      }
    };
    for (std::size_t row = 0; row < params.arm.joints.size(); ++row) {
      const Joint& joint = params.arm.joints[row];
      const std::string at = "arm.joints[" + std::to_string(row) + "].";
      leaves.emplace_back(at + "name=" + joint.name, 0.0), leaves.emplace_back(at + "offset", joint.offset);
      limits(at + "limits", joint.limits);
    }
    limits("stops", params.stops);
  }

  void forward() override {
    if (sent_) return;
    sent_ = true;
    rm::JointState& out = echo.write();
    out.size = std::min<std::size_t>(leaves.size(), lk::kMaxJoints);
    for (std::size_t leaf = 0; leaf < out.size; ++leaf)
      out.name[leaf] = leaves[leaf].first, out.position[leaf] = leaves[leaf].second;
    out.velocity.reset(), out.effort.reset();
    out.header.stamp = now();
  }

  std::vector<std::pair<std::string, double>> leaves;  // (path, value): what the box runs
  bool sent_ = false;
};

lk::System row_tables(const RowTablesParams& params) {
  lk::System s{"row_tables"};
  auto& box = s.add<RowTables>("box", params);
  s.expose(box.echo);
  return s;
}
LK_EXPORT(row_tables)
