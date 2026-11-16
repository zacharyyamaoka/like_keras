// CONTRACT §15 A24 (consumer-009): LK_PARAMS reflects std::vector<double> and std::vector<Nested> of an LK_PARAMS
// aggregate, filled at lk_create (never on the RT thread); describe() shows every row; an unknown key in a row is
// refused naming rows[2].<key>. This box sends its table once, as a JointState: a row's name and value per joint, and
// the gains as velocities. Driven by kpi/facilities/test_facilities_friction.py.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

struct Row {
  std::string name;
  double value = 0.0;
};
LK_PARAMS(Row, name, value)

struct TableParams {
  std::vector<Row> rows;
  std::vector<double> gains;
};
LK_PARAMS(TableParams, rows, gains)

struct Table : lk::Component {
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit Table(const TableParams& params) : params_(params) {}

  void forward() override {
    if (sent_) return;
    sent_ = true;
    rm::JointState& out = echo.write();
    const std::size_t joints = std::min<std::size_t>(params_.rows.size(), lk::kMaxJoints);
    out.size = joints;
    out.velocity.emplace();
    for (std::size_t joint = 0; joint < joints; ++joint) {
      out.name[joint] = std::string_view(params_.rows[joint].name);  // params_ lives as long as the box
      out.position[joint] = params_.rows[joint].value;
      (*out.velocity)[joint] = joint < params_.gains.size() ? params_.gains[joint] : -1.0;
    }
    out.effort.reset();
    out.header.stamp = now();
  }

  TableParams params_;
  bool sent_ = false;
};

lk::System table_params(const TableParams& params) {
  lk::System s{"table_params"};
  auto& table = s.add<Table>("table", params);
  s.expose(table.echo);
  return s;
}
LK_EXPORT(table_params)
