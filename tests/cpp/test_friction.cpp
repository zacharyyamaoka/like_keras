// Lane cpp's tests of the first consumer's decisions on the C++ side (CONTRACT §15 A17, A19, A20, A22, A23, A24,
// A28), where the frozen M0.3 ruler test does not reach the failure mode: a queue of converted inputs and a refused
// conversion (A17), the scheduling policy only while an RT thread runs (A19), the period of a rate-less box (A20),
// held health against the facility's own and a %s argument (A22), an optional input that is neither wired nor
// exposed (A23), the JSON paths of a table's errors (A24), and deactivate's last words on the C ABI (A28).
#include <gtest/gtest.h>
#include <like_keras/abi.h>
#include <sched.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <like_keras/msgs/std.hpp>
#include <string>
#include <thread>
#include <vector>

#include "box.hpp"

namespace {

using lk::detail::Box;

constexpr lk_place kStepped{50.0, nullptr, 0, 0, 0, 1};

std::uint32_t facility_port(const Box& box, std::uint32_t which) {
  return static_cast<std::uint32_t>(box.outs.size()) + which;
}

ArrowArray encoded(double position) {
  rm::JointState state{};
  state.size = 1, state.name[0] = "j1", state.position[0] = position;
  ArrowArray message{};
  lk::detail::find_wire(rm::JointState::kHash)->encode(&state, &message);
  return message;
}

struct HealthRow {
  std::string path, level, code, mode;
};

std::vector<HealthRow> health_round(Box& box) {
  std::vector<HealthRow> round;
  ArrowArray array{};
  while (box.take(facility_port(box, 1), &array) == 1) {
    rm::DiagnosticRecord view{};
    EXPECT_TRUE(lk::detail::find_wire(rm::DiagnosticRecord::kHash)->decode(array, &view));
    round.push_back({std::string(view.path), std::string(view.level), std::string(view.code), std::string(view.mode)});
    lk::detail::release(array);
  }
  return round;
}

std::vector<std::string> log_round(Box& box) {
  std::vector<std::string> lines;
  ArrowArray array{};
  while (box.take(facility_port(box, 0), &array) == 1) {
    rm::LogRecord view{};
    EXPECT_TRUE(lk::detail::find_wire(rm::LogRecord::kHash)->decode(array, &view));
    lines.emplace_back(view.message);
    lk::detail::release(array);
  }
  return lines;
}

std::vector<std::string> metric_names_of_the_box(Box& box) {
  std::vector<std::string> names;
  ArrowArray array{};
  bool first = true;
  while (box.take(facility_port(box, 2), &array) == 1) {
    rm::Metrics view{};
    EXPECT_TRUE(lk::detail::find_wire(rm::Metrics::kHash)->decode(array, &view));
    for (std::size_t at = 0; first && at < view.size; ++at) names.emplace_back(view.names[at]);
    first = false;
    lk::detail::release(array);
  }
  return names;
}

}  // namespace

// ---------------------------------------------------------------- A24: vectors and tables of LK_PARAMS (the C ABI)

struct Limit {
  double low = 0.0, high = 0.0;
};
LK_PARAMS(Limit, low, high)

struct Joint {
  std::string name;
  std::vector<Limit> limits;  // a table inside a table's row
};
LK_PARAMS(Joint, name, limits)

struct TableConfig {
  std::vector<Joint> joints;
  std::vector<double> gains{1.5, -2.0};
};
LK_PARAMS(TableConfig, joints, gains)

struct TableBox : lk::Component {
  explicit TableBox(const TableConfig& config) : config(config) {}
  void forward() override {}
  TableConfig config;
};

lk::System table_box(const TableConfig& config) {
  lk::System s{"table"};
  s.add<TableBox>("table", config);
  return s;
}
LK_EXPORT(table_box)

namespace {

std::string created_error(const char* params) {
  char error[512] = {};
  lk_box* box = lk_create(params, error, sizeof error);
  if (box != nullptr) lk_destroy(box);
  return box != nullptr ? "" : error;
}

TEST(TableParams, DefaultsAreShownAndEveryErrorNamesItsRow) {
  const std::string sheet = lk_describe();
  EXPECT_NE(sheet.find("\"params\": {\"joints\": [], \"gains\": [1.5, -2]}"), std::string::npos) << sheet;
  EXPECT_EQ(created_error(R"({"joints": [{"name": "a"}, {"name": "b"}], "gains": [1, 2, 3]})"), "");
  EXPECT_EQ(created_error(R"({"joints": [{"name": "a"}, {"name": "b"}, {"nam": "c"}]})"),
            "params.joints[2].nam: unknown key (known: name, limits)");
  EXPECT_EQ(created_error(R"({"joints": [{"name": "a"}, 3]})"), "params.joints[1]: expected an object, got a number");
  EXPECT_EQ(created_error(R"({"joints": [{"name": "a", "limits": [{"low": 1}, {"low": 1, "hgh": 2}]}]})"),
            "params.joints[0].limits[1].hgh: unknown key (known: low, high)");
  EXPECT_EQ(created_error(R"({"gains": [1, "two"]})"), "params.gains[1]: expected a number, got a string");
  EXPECT_EQ(created_error(R"({"joints": {"name": "a"}})"), "params.joints: expected an array, got an object");
}

}  // namespace
