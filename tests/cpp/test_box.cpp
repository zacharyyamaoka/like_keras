// Lane cpp's own tests of one box (cpp/src/box.hpp): its step clock and the counters its metrics publish
// (CONTRACT §5.4, §5.5, §8). Build and run: tests/cpp/CMakeLists.txt, or tests/test_cpp_box.py under pytest.
//
// WHY (fix round 1, M2): the M2 verifier found health's uptime mixing a wall-clock activation with the logical step
// time (no health at all under lockstep), drops.<port> bumped by two threads through a non-atomic load and store
// (lost counts), and three counters the box kept but never published.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <like_keras/msgs/std.hpp>
#include <map>
#include <string>
#include <vector>

#include "box.hpp"

namespace {

using lk::detail::Box;

constexpr lk_place kStepped{50.0, nullptr, 0, 0, 0, 1};  // lockstep at 50 Hz: the host steps the box

// The facility ports follow the exposed outs: log, health, metrics (abi.h).
std::uint32_t facility_port(const Box& box, std::uint32_t which) {
  return static_cast<std::uint32_t>(box.outs.size()) + which;
}

struct Metrics {
  std::string path;
  std::map<std::string, double> values;
};

std::vector<Metrics> metrics_round(Box& box) {  // one round: a record per component (the box's own first)
  std::vector<Metrics> round;
  ArrowArray array{};
  while (box.take(facility_port(box, 2), &array) == 1) {
    rm::Metrics view{};
    EXPECT_TRUE(lk::detail::find_wire(rm::Metrics::kHash)->decode(array, &view));
    Metrics& record = round.emplace_back();
    record.path = view.path;
    for (std::size_t at = 0; at < view.size; ++at) record.values[std::string(view.names[at])] = view.values[at];
    lk::detail::release(array);
  }
  return round;
}

std::map<std::string, double> values_of(const std::vector<Metrics>& round, const std::string& path) {
  for (const Metrics& record : round)
    if (record.path == path) return record.values;
  ADD_FAILURE() << "no metrics record for '" << path << "'";
  return {};
}

struct Health {
  std::string path, mode;
  double stamp, uptime_s;
};

std::vector<Health> health_round(Box& box) {
  std::vector<Health> round;
  ArrowArray array{};
  while (box.take(facility_port(box, 1), &array) == 1) {
    rm::DiagnosticRecord view{};
    EXPECT_TRUE(lk::detail::find_wire(rm::DiagnosticRecord::kHash)->decode(array, &view));
    round.push_back({std::string(view.path), std::string(view.mode), view.header.stamp, view.uptime_s});
    lk::detail::release(array);
  }
  return round;
}

std::size_t log_round(Box& box) {  // how many LogRecords one round of the log port delivers
  std::size_t delivered = 0;
  ArrowArray array{};
  for (; box.take(facility_port(box, 0), &array) == 1; ++delivered) lk::detail::release(array);
  return delivered;
}

// ---------------------------------------------------------------- the step clock (item 1)

struct Idle : lk::Component {
  void forward() override {}
};

lk::System idle_box() {
  lk::System s{"idle_box"};
  s.add<Idle>("idle");
  return s;
}

TEST(StepClock, LockstepUptimeCountsFromLogicalZero) {
  Box box{idle_box()};
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  const auto before = health_round(box);  // no step yet: logical 0.0, where the box activated (CONTRACT §5.5)
  ASSERT_EQ(before.size(), 1u);
  EXPECT_EQ(before[0].mode, "operational");
  EXPECT_EQ(before[0].stamp, 0.0);
  EXPECT_EQ(before[0].uptime_s, 0.0);
  for (const double at : {0.0, 0.5, 1.0}) box.step(at);
  const auto after = health_round(box);
  ASSERT_EQ(after.size(), 1u);
  EXPECT_DOUBLE_EQ(after[0].stamp, 1.0);
  EXPECT_DOUBLE_EQ(after[0].uptime_s, 1.0) << "uptime mixes a wall-clock activation with the logical step time";
  box.stop();
}

TEST(StepClock, RealtimeUptimeCountsFromActivation) {
  Box box{idle_box()};
  std::string error;
  const lk_place realtime{200.0, nullptr, 0, 0, 0, 0};
  ASSERT_TRUE(box.start(realtime, error)) << error;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (box.children[0].steps.load() < 10 && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
  const auto round = health_round(box);
  box.stop();
  ASSERT_EQ(round.size(), 1u);
  EXPECT_GE(round[0].uptime_s, 0.0);
  EXPECT_LT(round[0].uptime_s, 5.0);
  EXPECT_GT(round[0].stamp, 1.7e9) << "real time stamps epoch seconds (CONTRACT §5.6)";
}

// ---------------------------------------------------------------- drops.<queue in>: two writers (item 3)

struct Consumer : lk::Component {
  lk::In<rm::JointState> queued{this, "queued", {.queue = 16}};
  std::atomic<std::uint64_t> consumed{0};  // forward runs only with a message (gather): one per pop
  void forward() override { consumed.fetch_add(queued.get() != nullptr, std::memory_order_relaxed); }
};

// The host pushes as fast as it can into an exposed queue(16) (a full ring refuses the newest, counted by the host)
// while the RT thread trims it to 16 at every step (drop-oldest, counted by the RT thread). Every message is then
// consumed, dropped, or still in the ring: a lost count breaks the sum. Not a timing test: the sum is exact.
TEST(QueueDrops, TwoWritersLoseNoCount) {
  lk::System s{"stress_box"};
  Consumer& consumer = s.add<Consumer>("consumer");
  s.expose(consumer.queued);
  Box box{std::move(s)};
  std::string error;
  const lk_place fast{20000.0, nullptr, 0, 0, 0, 0};
  ASSERT_TRUE(box.start(fast, error)) << error;
  rm::JointState state{};
  state.size = 1, state.name[0] = "joint", state.position[0] = 1.0;
  const lk::detail::Wire& wire = *lk::detail::find_wire(rm::JointState::kHash);
  std::uint64_t pushed = 0;
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
  while (std::chrono::steady_clock::now() < until) {
    for (int burst = 0; burst < 256; ++burst) {
      ArrowArray message{};
      wire.encode(&state, &message);
      ASSERT_EQ(box.push(0, &message), 0);
      ++pushed;
    }
  }
  box.stop();
  const lk::detail::Ring& ring = *box.ins[0]->ring;
  const std::uint64_t left = ring.head.load() - ring.tail.load();
  const double dropped = values_of(metrics_round(box), "consumer").at("drops.queued");
  EXPECT_GT(consumer.consumed.load(), 0u);
  EXPECT_GT(ring.refused.load(), 0u) << "the host never found the ring full: one writer only";
  EXPECT_GT(ring.trimmed.load(), 0u) << "the RT thread never trimmed: one writer only";
  EXPECT_EQ(static_cast<double>(pushed), static_cast<double>(consumer.consumed.load() + left) + dropped)
      << pushed << " pushed, " << consumer.consumed.load() << " consumed, " << left << " left, " << dropped
      << " counted as drops";
}

// ---------------------------------------------------------------- counters published (item 4)

struct Wide : lk::Component {  // its 16 names total 4800 B, over the 4 KiB string arena of an exposed Out
  lk::Out<rm::JointState> wide{this, "wide"};
  std::vector<std::string> names;
  Wide() {
    for (int joint = 0; joint < lk::kMaxJoints; ++joint) names.emplace_back(300, static_cast<char>('a' + joint));
  }
  void forward() override {
    rm::JointState& out = wide.write();
    out.size = lk::kMaxJoints;
    for (int joint = 0; joint < lk::kMaxJoints; ++joint) out.name[joint] = names[joint];
  }
};

TEST(PublishedCounters, AnOutOverItsStringArenaCountsDropsOnItsPort) {
  lk::System s{"wide_box"};
  Wide& wide = s.add<Wide>("talker");
  s.expose(wide.wide);
  Box box{std::move(s)};
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  for (int tick = 0; tick < 5; ++tick) box.step(tick * 0.02);
  ArrowArray none{};
  EXPECT_EQ(box.take(0, &none), 0) << "a message over the arena was published";
  EXPECT_EQ(values_of(metrics_round(box), "talker")["drops.wide"], 5.0);
  box.stop();
}

struct Chatty : lk::Component {
  int lines_per_step = 1100;  // over the 1024-entry log ring
  void forward() override {
    for (int line = 0; line < lines_per_step; ++line) log.info("line %g", line);
  }
};

TEST(PublishedCounters, LostLogLinesCountOnTheBoxRecord) {
  lk::System s{"chatty_box"};
  s.add<Chatty>("chatty");
  Box box{std::move(s)};
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  // Five steps of 1100 lines, nobody taking the log port: the ring refuses 76 each step (the logging thread's
  // count), and the backlog keeps the newest 4096 (the facilities thread's count).
  double dropped = 0.0;
  for (int tick = 0; tick < 5; ++tick) {
    box.step(tick * 0.02);
    dropped = values_of(metrics_round(box), "")["drops.log"];
  }
  const std::size_t delivered = log_round(box);
  EXPECT_EQ(delivered, 4096u);
  EXPECT_EQ(dropped, 5 * 76.0 + (5 * 1024 - 4096));
  EXPECT_EQ(static_cast<double>(delivered) + dropped, 5 * 1100.0) << "a logged line is delivered or counted";
  box.stop();
}

std::atomic<int> released{0};
void count_release(ArrowArray* array) {
  released.fetch_add(1);
  array->release = nullptr;
}

TEST(PublishedCounters, AFullGraveyardCountsUnreleasedOnTheBoxRecord) {
  Box box{idle_box()};
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  released = 0;
  for (int message = 0; message < 4097; ++message) {  // the graveyard holds 4096 until a host thread buries them
    ArrowArray retired{};
    retired.release = &count_release;
    box.retire(retired);
  }
  const auto round = metrics_round(box);  // lk_take buries first: 4096 released here, on this thread
  EXPECT_EQ(released.load(), 4096);
  EXPECT_EQ(values_of(round, "")["unreleased"], 1.0);
  box.stop();
}

TEST(PublishedCounters, TheBoxRecordComesFirstWithTheStandardNames) {
  Box box{idle_box()};
  std::string error;
  ASSERT_TRUE(box.start(kStepped, error)) << error;
  for (int tick = 0; tick < 7; ++tick) box.step(tick * 0.02);
  const auto round = metrics_round(box);
  ASSERT_EQ(round.size(), 2u);
  EXPECT_EQ(round[0].path, "") << "the host names a record with path \"\" the box's own path";
  for (const char* name :
       {"steps", "step_p50_s", "step_p99_s", "step_max_s", "period_error_p99_s", "overruns", "drops.log", "unreleased"})
    EXPECT_EQ(round[0].values.count(name), 1u) << name;
  EXPECT_EQ(round[0].values.at("steps"), 7.0);
  EXPECT_EQ(round[1].path, "idle");
  EXPECT_EQ(round[1].values.at("steps"), 7.0);
  box.stop();
}

}  // namespace
