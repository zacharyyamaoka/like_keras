// CONTRACT §15 A17 (consumer-002): In<T, Converted>. like_keras converts each pushed message with Converted::from on
// the pushing thread, inside lk_push (allocation allowed there), hands the RT thread a pointer, and retires it
// through the graveyard, off the RT thread. This box is the raw node's PlanMailbox written against that seam: it
// adopts the first plan by moving the converted data out of the value it is handed, keeps it for the whole run while
// every later plan is taken and refused, and frees it in deactivate. Each plan holds id * 1000 + index in every cell
// and is poisoned with NaN when freed, so a plan read after it was freed (or overwritten) reads wrong.
// kpi/polyglot/test_polyglot_friction.py reads the counters from the file counts_path once the deployment has stopped.
#include <fcntl.h>
#include <like_keras/lk.hpp>
#include <like_keras/msgs/trajectory.hpp>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

enum Count : int {
  kMade,           // plans converted (Converted::from)
  kMadeOnRt,       // ... of them on the RT thread: must stay 0
  kFreed,          // plans freed
  kFreedOnRt,      // ... of them on the RT thread: must stay 0
  kTaken,          // plans take() handed to forward
  kTakenBroken,    // ... that did not read intact
  kKeptChecks,     // steps that read the kept plan
  kKeptBroken,     // ... and found it not intact
  kKeptId,         // the id of the kept plan
  kCounts,
};
// WHY a mapped file: the test reads the counters after lk_destroy, and a component library exports only the lk_* C
// ABI (CONTRACT §15 A28), so nothing of this box can be looked up from outside. Mapped in activate (off the RT
// thread) and never unmapped, so the plans freed when the box is destroyed still count.
std::int64_t* counts = nullptr;
std::atomic<long> rt_thread{0};

void bump(Count which) {
  if (counts != nullptr) std::atomic_ref<std::int64_t>(counts[which]).fetch_add(1);
}
void set(Count which, std::int64_t value) {
  if (counts != nullptr) std::atomic_ref<std::int64_t>(counts[which]).store(value);
}

long this_thread() { return static_cast<long>(::syscall(SYS_gettid)); }
bool on_rt_thread() {
  const long rt = rt_thread.load(std::memory_order_relaxed);
  return rt != 0 && rt == this_thread();
}

struct PlanData {
  std::vector<double> positions;
  long id = -1;

  PlanData() = default;
  PlanData(const PlanData&) = delete;
  PlanData& operator=(const PlanData&) = delete;
  ~PlanData() {
    bump(kFreed);
    if (on_rt_thread()) bump(kFreedOnRt);
    std::fill(positions.begin(), positions.end(), std::numeric_limits<double>::quiet_NaN());  // poison
  }

  bool intact() const {
    if (positions.empty()) return false;
    for (std::size_t index = 0; index < positions.size(); ++index) {
      if (positions[index] != static_cast<double>(id) * 1000.0 + static_cast<double>(index)) return false;
    }
    return true;
  }
};

}  // namespace

struct KeptPlan {
  // WHY mutable: adopting a plan moves its data out of the converted value the input hands over, as the raw node's
  // control thread took a Plan* out of its mailbox. What stays behind is empty, so retiring it frees nothing.
  mutable std::unique_ptr<PlanData> data;

  static KeptPlan from(const rm::JointTrajectory& trajectory) {
    bump(kMade);
    if (on_rt_thread()) bump(kMadeOnRt);
    auto data = std::make_unique<PlanData>();
    data->positions.assign(trajectory.positions.begin(), trajectory.positions.end());
    data->id = data->positions.empty() ? -1 : static_cast<long>(data->positions[0] / 1000.0);
    return KeptPlan{std::move(data)};
  }
};

struct Keeper : lk::Component {
  struct Params {
    std::string counts_path;  // kCounts int64 counters, in Count order, zeroed at activate
  };
  lk::In<rm::JointTrajectory, KeptPlan> trajectory{this, "trajectory"};

  explicit Keeper(const Params& params) : params_(params) {}

  void activate() override {
    rt_thread = 0;
    const int file = ::open(params_.counts_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (file < 0 || ::ftruncate(file, kCounts * sizeof(std::int64_t)) != 0) {
      throw std::runtime_error("kept_plan: cannot open counts_path");
    }
    void* memory = ::mmap(nullptr, kCounts * sizeof(std::int64_t), PROT_READ | PROT_WRITE, MAP_SHARED, file, 0);
    ::close(file);
    if (memory == MAP_FAILED) throw std::runtime_error("kept_plan: cannot map counts_path");
    counts = static_cast<std::int64_t*>(memory);
  }

  void forward() override {
    rt_thread.store(this_thread(), std::memory_order_relaxed);
    if (const KeptPlan* plan = trajectory.take()) {
      bump(kTaken);
      if (!plan->data || !plan->data->intact()) {
        bump(kTakenBroken);
      } else if (!kept_) {
        kept_.swap(plan->data);  // adopt the first plan; every later one is refused and retired by like_keras
        set(kKeptId, kept_->id);
      }
    }
    if (kept_) {
      bump(kKeptChecks);
      if (!kept_->intact()) bump(kKeptBroken);
    }
  }

  void deactivate() override { kept_.reset(); }  // freed here: deactivate runs off the RT thread

  Params params_;
  std::unique_ptr<PlanData> kept_;
};
LK_PARAMS(Keeper::Params, counts_path)

lk::System kept_plan(const Keeper::Params& params) {
  lk::System s{"kept_plan"};
  auto& keeper = s.add<Keeper>("keeper", params);
  s.expose(keeper.trajectory);
  return s;
}
LK_EXPORT(kept_plan)
