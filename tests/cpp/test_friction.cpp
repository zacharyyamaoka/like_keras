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

// ---------------------------------------------------------------- A17: In<T, Converted>

std::atomic<int> made{0}, freed{0}, freed_on_rt{0};
std::atomic<std::thread::id> rt_thread{};

struct Plan {  // what the RT thread reads: data made on the pushing thread, freed off the RT thread
  std::vector<double> positions;
  Plan() = default;
  Plan(Plan&& other) noexcept : positions(std::move(other.positions)) { other.positions.clear(); }
  ~Plan() {
    if (positions.empty()) return;
    freed.fetch_add(1);
    if (std::this_thread::get_id() == rt_thread.load()) freed_on_rt.fetch_add(1);
  }
  static Plan from(const rm::JointState& state) {
    if (state.position[0] < 0.0) throw std::invalid_argument("a negative plan is refused");  // validation
    Plan plan;
    plan.positions.assign(1000, state.position[0]);
    made.fetch_add(1);
    return plan;
  }
};

struct Follower : lk::Component {
  lk::In<rm::JointState, Plan> plans{this, "plans", {.queue = 4}};
  std::atomic<int> read{0}, intact{0};
  void forward() override {
    rt_thread.store(std::this_thread::get_id());
    const Plan* plan = plans.get();
    read.fetch_add(1);
    if (plan != nullptr && plan->positions.size() == 1000 && plan->positions.front() == plan->positions.back())
      intact.fetch_add(1);
  }
};

TEST(ConvertedInput, AQueueOfConvertedValuesIsFreedOffTheRtThreadAndNoneLeaks) {
  made = 0, freed = 0, freed_on_rt = 0;
  {
    lk::System s{"follow"};
    Follower& follower = s.add<Follower>("follower");
    s.expose(follower.plans);
    Box box{std::move(s)};
    std::string error;
    const lk_place realtime{500.0, nullptr, 0, 0, 0, 0};
    ASSERT_TRUE(box.start(realtime, error)) << error;
    for (int plan = 1; plan <= 200; ++plan) {
      ArrowArray message = encoded(plan);
      box.push(0, &message);  // a full ring refuses the newest (0); a refused one is never made
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (follower.read.load() < 50 && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
    box.stop();
    EXPECT_GE(follower.read.load(), 50);
    EXPECT_EQ(follower.intact.load(), follower.read.load()) << "a converted value read on the RT thread was not intact";
  }
  EXPECT_GT(made.load(), 50);
  EXPECT_EQ(freed.load(), made.load()) << "converted values leaked";
  EXPECT_EQ(freed_on_rt.load(), 0) << "a converted value was freed on the RT thread";
}

struct Latest : lk::Component {
  lk::In<rm::JointState, Plan> plan{this, "plan"};
  double seen = -1.0;
  int taken = 0;
  void forward() override {
    if (const Plan* fresh = plan.take()) seen = fresh->positions[0], ++taken;
  }
};

TEST(ConvertedInput, AConversionThatThrowsIsARefusedPushAndTheLastValueStays) {
  made = 0, freed = 0;
  {
    lk::System s{"latest"};
    Latest& latest = s.add<Latest>("latest");
    s.expose(latest.plan);
    Box box{std::move(s)};
    std::string error;
    ASSERT_TRUE(box.start(kStepped, error)) << error;
    ArrowArray good = encoded(2.0), refused = encoded(-1.0);
    ASSERT_EQ(box.push(0, &good), 0);
    box.step(0.0);
    EXPECT_EQ(box.push(0, &refused), -1) << "Converted::from threw: lk_push refuses it, it never throws into C";
    EXPECT_EQ(refused.release, nullptr) << "a refused message is released (moved in) all the same";
    box.step(0.02);
    EXPECT_EQ(latest.taken, 1) << "a refused push is not an arrival";
    EXPECT_EQ(latest.seen, 2.0);
    box.stop();
  }
  EXPECT_EQ(made.load(), 1);
  EXPECT_EQ(freed.load(), 1);
}

// ---------------------------------------------------------------- A19: sched_policy while an RT thread runs

struct Idle : lk::Component {
  void forward() override {}
};

lk::System idle_box() {
  lk::System s{"idle_box"};
  s.add<Idle>("idle");
  return s;
}

TEST(RtLadder, TheBoxRecordReportsThePolicyOnlyWhileAnRtThreadRuns) {
  Box stepped{idle_box()};
  std::string error;
  ASSERT_TRUE(stepped.start(kStepped, error)) << error;
  stepped.step(0.0);
  const auto names = metric_names_of_the_box(stepped);
  EXPECT_EQ(std::count(names.begin(), names.end(), "sched_policy"), 0) << "lockstep has no RT thread to report";
  stepped.stop();

  Box running{idle_box()};
  const lk_place realtime{200.0, nullptr, 0, 0, 0, 0};  // fifo 0: no ladder asked, SCHED_OTHER honestly
  ASSERT_TRUE(running.start(realtime, error)) << error;
  ArrowArray array{};
  bool reported = false;
  while (running.take(facility_port(running, 2), &array) == 1) {
    rm::Metrics view{};
    lk::detail::find_wire(rm::Metrics::kHash)->decode(array, &view);
    for (std::size_t at = 0; view.path.empty() && at < view.size; ++at)
      if (view.names[at] == "sched_policy") reported = view.values[at] == SCHED_OTHER;
    lk::detail::release(array);
  }
  running.stop();
  EXPECT_TRUE(reported) << "the box record did not report the RT thread's SCHED_OTHER";
  const auto after = metric_names_of_the_box(running);
  EXPECT_EQ(std::count(after.begin(), after.end(), "sched_policy"), 0) << "no RT thread after stop: nothing to report";
}

// ---------------------------------------------------------------- A20: period()

std::atomic<double> period_in_activate{-1.0};

struct PeriodReader : lk::Component {
  lk::In<rm::JointState> trigger{this, "trigger", {.queue = 1}};
  void activate() override { period_in_activate = period(); }
  void forward() override {}
};

TEST(Period, IsTheGroupsPeriodFromActivateAndZeroRateLessOrEager) {
  PeriodReader eager;
  EXPECT_EQ(eager.period(), 0.0) << "no box, no group";
  for (const double hz : {50.0, 0.0}) {
    lk::System s{"period"};
    PeriodReader& reader = s.add<PeriodReader>("reader");
    s.expose(reader.trigger);
    Box box{std::move(s)};
    std::string error;
    const lk_place place{hz, nullptr, 0, 0, 0, 1};
    ASSERT_TRUE(box.start(place, error)) << error;
    EXPECT_DOUBLE_EQ(period_in_activate.load(), hz > 0.0 ? 0.02 : 0.0) << hz << " Hz";
    box.stop();
  }
}

// ---------------------------------------------------------------- A22: health(level, code), %s

struct SaysHealth : lk::Component {
  int step = 0;
  void forward() override {
    switch (step++) {
      case 0:
        health("warn", "Degraded");
        break;  // round 1: warn / Degraded
      case 2:
        log.warning("a warning, same level");
        break;  // round 3: the component's code on a tie
      case 3:
        log.error("refused: %s at %g", "joint", 3);
        break;  // round 4: error / ErrorLogged, worse
      case 5:
        throw std::runtime_error("broken");  // round 6: Exception wins over the held error
      default:
        break;
    }
    if (step == 5) health("error", "Braked");  // round 5: error / Braked, the component's
  }
};

TEST(Health, HeldTheWorseOfTheComponentsAndTheFacilitysAndAnExceptionWins) {
  lk::System s{"saying"};
  s.add<SaysHealth>("says");
  Box box{std::move(s)};
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  std::vector<std::pair<std::string, std::string>> rounds;
  for (int round = 0; round < 6; ++round) {
    box.step(round * 0.02);
    const auto health = health_round(box);
    ASSERT_EQ(health.size(), 1u);
    rounds.emplace_back(health[0].level, health[0].code);
  }
  const std::vector<std::pair<std::string, std::string>> expected{{"warn", "Degraded"}, {"warn", "Degraded"},
                                                                  {"warn", "Degraded"}, {"error", "ErrorLogged"},
                                                                  {"error", "Braked"},  {"error", "Exception"}};
  EXPECT_EQ(rounds, expected);
  const auto lines = log_round(box);
  EXPECT_NE(std::find(lines.begin(), lines.end(), "refused: joint at 3"), lines.end()) << "a %s argument";
  box.stop();
}

TEST(Health, AnErrorLoggedLastsUntilARoundWithNone) {
  struct Once : lk::Component {
    int step = 0;
    void forward() override {
      if (step++ == 0) log.error("once");
    }
  };
  lk::System s{"once"};
  s.add<Once>("once");
  Box box{std::move(s)};
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  box.step(0.0);
  EXPECT_EQ(health_round(box)[0].code, "ErrorLogged");
  box.step(0.02);
  EXPECT_EQ(health_round(box)[0].level, "ok") << "no error logged in this round";
  box.stop();
}

// ---------------------------------------------------------------- A23: an optional input

struct Guarded : lk::Component {
  lk::In<rm::DiagnosticRecord> verdict{this, "verdict", {.optional = true}};
  lk::In<rm::JointState> state{this, "state"};
  bool unwired_reads_null = false;
  void forward() override { unwired_reads_null = verdict.get() == nullptr; }
};

TEST(OptionalInput, MayStayUnwiredAndUnexposedAndReadsNull) {
  lk::System s{"guarded"};
  Guarded& guarded = s.add<Guarded>("guarded");
  s.expose(guarded.state);
  Box box{std::move(s)};
  EXPECT_EQ(box.sheet().find("\"optional\": true"), std::string::npos) << "an unexposed input is not on the sheet";
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  box.step(0.0);
  EXPECT_TRUE(guarded.unwired_reads_null);
  box.stop();

  lk::System exposed{"exposed"};
  Guarded& other = exposed.add<Guarded>("guarded");
  exposed.expose(other.state, other.verdict);
  EXPECT_NE(Box{std::move(exposed)}.sheet().find("\"verdict\": {\"dir\": \"in\""), std::string::npos);

  lk::System unbound{"unbound"};
  unbound.add<Guarded>("guarded");  // state is neither wired nor exposed, nor optional
  EXPECT_THROW(Box{std::move(unbound)}, std::invalid_argument);
}

TEST(OptionalInput, IsListedOptionalOnTheSheet) {
  lk::System s{"listed"};
  Guarded& guarded = s.add<Guarded>("guarded");
  s.expose(guarded.state, guarded.verdict);
  const std::string sheet = Box{std::move(s)}.sheet();
  const auto verdict = sheet.find("\"verdict\"");
  ASSERT_NE(verdict, std::string::npos);
  EXPECT_NE(sheet.find("\"optional\": true", verdict), std::string::npos) << sheet;
  EXPECT_EQ(sheet.find("\"optional\""), sheet.find("\"optional\": true", verdict)) << "only the optional one";
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
