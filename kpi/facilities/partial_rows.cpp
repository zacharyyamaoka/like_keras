// CONTRACT §15 A44 (library verifier, the round after A31-A42), frozen by ruler M0.8 (A45): a given row of a params
// table is filled from the row type's member defaults (a value-initialized row), never from the default table's row at
// the same index, and describe() lists each given row filled the same way, so the datasheet is exactly what the box
// runs. This box's table has two default rows, and its row type has member defaults unlike either row (and unlike a
// zeroed row), so a key the config leaves out shows which of them filled it. It sends what it runs once, as a
// JointState: per row, the name, the offset as position and the scale as velocity. Driven by
// kpi/facilities/test_facilities_friction.py.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

struct PartRow {
  std::string name = "spare";
  double offset = 0.125;
  double scale = 1.5;
};
LK_PARAMS(PartRow, name, offset, scale)

struct PartialRowsParams {
  std::vector<PartRow> joints{{"shoulder", 0.25, 2.0}, {"elbow", -0.5, 3.0}};
};
LK_PARAMS(PartialRowsParams, joints)

struct PartialRows : lk::Component {
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit PartialRows(const PartialRowsParams& params) : params_(params) {}

  void forward() override {
    if (sent_) return;
    sent_ = true;
    rm::JointState& out = echo.write();
    const std::size_t joints = std::min<std::size_t>(params_.joints.size(), lk::kMaxJoints);
    out.size = joints;
    out.velocity.emplace();
    for (std::size_t joint = 0; joint < joints; ++joint) {
      out.name[joint] = std::string_view(params_.joints[joint].name);  // params_ lives as long as the box
      out.position[joint] = params_.joints[joint].offset;
      (*out.velocity)[joint] = params_.joints[joint].scale;
    }
    out.effort.reset();
    out.header.stamp = now();
  }

  PartialRowsParams params_;
  bool sent_ = false;
};

lk::System partial_rows(const PartialRowsParams& params) {
  lk::System s{"partial_rows"};
  auto& rows = s.add<PartialRows>("rows", params);
  s.expose(rows.echo);
  return s;
}
LK_EXPORT(partial_rows)
