// The real-time hot-path gate (CONTRACT §11): 0 heap allocations per tick over 100k ticks of the C++ chain, and no
// lock taken on the stepping thread. gtest 1.14; built by kpi/CMakeLists.txt as build/kpi/alloc_probe.
//
// The probe drives libarm_controller.so (kpi/tasks/arm_cell, on the fake twin bus) through the C ABI alone
// (CONTRACT §6.3, declared here from the contract, so the probe does not trust like_keras's own header): lk_create,
// lk_start(stepped = 1), then lk_step on this thread, a 7 x 100 plan pushed every 50 ticks, every output taken.
// While a tick runs, this binary counts every operator new / malloc family call and every pthread mutex, rwlock or
// semaphore wait made on this thread (the replacements below interpose on the whole process; -rdynamic exports them).
// Two runs: the guard as configured (it passes the plans through), and a guard whose runaway_rad makes every plan
// run away and its hold be released again (CONTRACT §15 A14), so the hold, the release and their log lines step too.
#include <dlfcn.h>
#include <gtest/gtest.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <new>
#include <regex>
#include <string>
#include <string_view>
#include <vector>

#include "golden/lkf_frame.hpp"

extern "C" {
void* __libc_malloc(size_t);
void* __libc_calloc(size_t, size_t);
void* __libc_realloc(void*, size_t);
void* __libc_memalign(size_t, size_t);
void __libc_free(void*);
}

namespace {
thread_local bool counting = false;
std::atomic<long> allocations{0};
std::atomic<long> locks{0};
inline void count_allocation() {
  if (counting) allocations.fetch_add(1, std::memory_order_relaxed);
}
inline void count_lock() {
  if (counting) locks.fetch_add(1, std::memory_order_relaxed);
}
template <class F>
F next(const char* name) {
  return reinterpret_cast<F>(dlsym(RTLD_NEXT, name));
}
}  // namespace

extern "C" {
void* malloc(size_t size) { count_allocation(); return __libc_malloc(size); }
void* calloc(size_t count, size_t size) { count_allocation(); return __libc_calloc(count, size); }
void* realloc(void* pointer, size_t size) { count_allocation(); return __libc_realloc(pointer, size); }
void* memalign(size_t alignment, size_t size) { count_allocation(); return __libc_memalign(alignment, size); }
void* aligned_alloc(size_t alignment, size_t size) { count_allocation(); return __libc_memalign(alignment, size); }
int posix_memalign(void** out, size_t alignment, size_t size) {
  count_allocation();
  *out = __libc_memalign(alignment, size);
  return *out ? 0 : ENOMEM;
}
void free(void* pointer) { __libc_free(pointer); }

int pthread_mutex_lock(pthread_mutex_t* mutex) {
  static auto real = next<int (*)(pthread_mutex_t*)>("pthread_mutex_lock");
  count_lock();
  return real(mutex);
}
int pthread_mutex_trylock(pthread_mutex_t* mutex) {
  static auto real = next<int (*)(pthread_mutex_t*)>("pthread_mutex_trylock");
  count_lock();
  return real(mutex);
}
int pthread_rwlock_rdlock(pthread_rwlock_t* lock) {
  static auto real = next<int (*)(pthread_rwlock_t*)>("pthread_rwlock_rdlock");
  count_lock();
  return real(lock);
}
int pthread_rwlock_wrlock(pthread_rwlock_t* lock) {
  static auto real = next<int (*)(pthread_rwlock_t*)>("pthread_rwlock_wrlock");
  count_lock();
  return real(lock);
}
int sem_wait(sem_t* semaphore) {
  static auto real = next<int (*)(sem_t*)>("sem_wait");
  count_lock();
  return real(semaphore);
}
int pthread_cond_wait(pthread_cond_t* condition, pthread_mutex_t* mutex) {
  static auto real = next<int (*)(pthread_cond_t*, pthread_mutex_t*)>("pthread_cond_wait");
  count_lock();
  return real(condition, mutex);
}
}

void* operator new(std::size_t size) {
  count_allocation();
  if (void* pointer = __libc_malloc(size ? size : 1)) return pointer;
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return operator new(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  count_allocation();
  return __libc_malloc(size ? size : 1);
}
void* operator new[](std::size_t size, const std::nothrow_t& tag) noexcept { return operator new(size, tag); }
void* operator new(std::size_t size, std::align_val_t alignment) {
  count_allocation();
  if (void* pointer = __libc_memalign(static_cast<size_t>(alignment), size ? size : 1)) return pointer;
  throw std::bad_alloc();
}
void* operator new[](std::size_t size, std::align_val_t alignment) { return operator new(size, alignment); }
void operator delete(void* pointer) noexcept { __libc_free(pointer); }
void operator delete[](void* pointer) noexcept { __libc_free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { __libc_free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { __libc_free(pointer); }
void operator delete(void* pointer, std::align_val_t) noexcept { __libc_free(pointer); }
void operator delete[](void* pointer, std::align_val_t) noexcept { __libc_free(pointer); }
void operator delete(void* pointer, std::size_t, std::align_val_t) noexcept { __libc_free(pointer); }
void operator delete[](void* pointer, std::size_t, std::align_val_t) noexcept { __libc_free(pointer); }

// CONTRACT §6.3, as written there.
extern "C" {
typedef struct lk_box lk_box;
typedef struct {
  double hz;
  const int* cpus;
  int ncpus;
  int fifo;
  int lock_memory;
  int stepped;
} lk_place;
}

namespace {

struct Abi {
  uint32_t (*abi_version)();
  const char* (*describe)();
  lk_box* (*create)(const char*, char*, size_t);
  int (*start)(lk_box*, const lk_place*, char*, size_t);
  int (*step)(lk_box*, double);
  int (*push)(lk_box*, uint32_t, ArrowArray*);
  int (*take)(lk_box*, uint32_t, ArrowArray*);
  void (*stop)(lk_box*);
  void (*destroy)(lk_box*);
};

template <class F>
void bind(void* library, const char* name, F& out) {
  out = reinterpret_cast<F>(dlsym(library, name));
  if (out == nullptr) throw std::runtime_error(std::string("missing ") + name);
}

// Port indices: the order of the exposed ports of each direction in lk_describe()'s "ports" (the ruler's reading,
// docs/friction/contract-002.md).
std::vector<std::string> ports(const std::string& description, const std::string& direction) {
  std::vector<std::string> found;
  const std::regex port("\"([A-Za-z0-9_]+)\"\\s*:\\s*\\{\\s*\"dir\"\\s*:\\s*\"(in|out)\"");
  for (auto it = std::sregex_iterator(description.begin(), description.end(), port); it != std::sregex_iterator(); ++it)
    if ((*it)[2] == direction) found.push_back((*it)[1]);
  return found;
}

// What one run of the chain saw: the allocations and locks lk_step made after the warm-up, the guard's log lines
// (taken from the box's log port), and how often the consumer released the pushed plan.
struct ChainRun {
  long allocations = 0, locks = 0, runaway_logs = 0, released_logs = 0;
  int frames_released = 0;
};

// The message of a LogRecord taken from the box (std/LogRecord: header, path, tick, level, message; utf8 = validity,
// int32 offsets, data), or empty when the layout is not that.
std::string_view log_message(const ArrowArray& record) {
  if (record.n_children < 5 || record.children[4] == nullptr) return {};
  const ArrowArray& message = *record.children[4];
  if (message.n_buffers != 3 || message.buffers[1] == nullptr || message.buffers[2] == nullptr) return {};
  const auto* offsets = static_cast<const int32_t*>(message.buffers[1]);
  return {static_cast<const char*>(message.buffers[2]) + offsets[message.offset],
          static_cast<std::size_t>(offsets[message.offset + 1] - offsets[message.offset])};
}

// libarm_controller.so through the C ABI: lk_create with `guard` as the guard's params, lk_start(stepped = 1), then
// kWarmup + kTicks lk_step on this thread, a 7 x 100 plan pushed every 50 ticks, every output taken.
void run_chain(const std::string& guard, ChainRun& run) {
  const std::string library_path = std::string(KPI_BUILD_DIR) + "/libarm_controller.so";
  void* library = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  ASSERT_NE(library, nullptr) << dlerror();
  Abi abi{};
  bind(library, "lk_abi_version", abi.abi_version);
  bind(library, "lk_describe", abi.describe);
  bind(library, "lk_create", abi.create);
  bind(library, "lk_start", abi.start);
  bind(library, "lk_step", abi.step);
  bind(library, "lk_push", abi.push);
  bind(library, "lk_take", abi.take);
  bind(library, "lk_stop", abi.stop);
  bind(library, "lk_destroy", abi.destroy);
  ASSERT_EQ(abi.abi_version(), 1u);
  const std::string description = abi.describe();
  const auto inputs = ports(description, "in");
  const auto outputs = ports(description, "out");
  uint32_t trajectory_port = 0;
  for (; trajectory_port < inputs.size() && inputs[trajectory_port] != "trajectory"; ++trajectory_port) {}
  ASSERT_LT(trajectory_port, inputs.size()) << "no exposed input `trajectory` in " << description;
  const auto log_port = static_cast<uint32_t>(outputs.size());  // abi.h: facilities follow the exposed outs, log 1st

  const auto scratch = std::filesystem::path(KPI_BUILD_DIR) / ("alloc_probe-" + std::to_string(getpid()));
  std::filesystem::create_directories(scratch);
  const std::string params = R"({"bus": {"twin": true, "exchange_s": 0.0, "joints": 7, "board_file": "", "lock_path": ")" +
                             (scratch / "bus.lock").string() + R"("}, "guard": )" + guard + "}";
  char error[1024] = {0};
  lk_box* box = abi.create(params.c_str(), error, sizeof(error));
  ASSERT_NE(box, nullptr) << error;
  const lk_place place{100.0, nullptr, 0, 0, 0, 1};
  ASSERT_EQ(abi.start(box, &place, error, sizeof(error)), 0) << error;

  auto frame = kpi::load_frame(std::string(KPI_DIR) + "/golden/bench/JointTrajectory_7x100.lkf");
  const auto root_release = frame->arrays[0].release;
  constexpr int kWarmup = 1000;
  constexpr int kTicks = 100000;
  long allocations_seen = 0, locks_seen = 0;
  double now = 0.0;
  for (int tick = 0; tick < kWarmup + kTicks; ++tick) {
    if (tick % 50 == 0) {
      frame->arrays[0].release = root_release;  // the consumer moved the last one out; hand it over again
      ASSERT_EQ(abi.push(box, trajectory_port, &frame->arrays[0]), 0);
    }
    if (tick == kWarmup) {
      allocations_seen = allocations.load();
      locks_seen = locks.load();
    }
    counting = tick >= kWarmup;
    abi.step(box, now);
    counting = false;
    for (uint32_t port = 0; port < outputs.size(); ++port) {
      ArrowArray out{};
      while (abi.take(box, port, &out) == 1) {
        if (out.release) out.release(&out);
      }
    }
    ArrowArray record{};
    while (abi.take(box, log_port, &record) == 1) {
      const std::string_view message = log_message(record);
      run.runaway_logs += message.find("runaway: holding") != std::string_view::npos;
      run.released_logs += message.find("runaway hold released") != std::string_view::npos;
      if (record.release) record.release(&record);
    }
    now += 0.01;
  }
  run.allocations = allocations.load() - allocations_seen;
  run.locks = locks.load() - locks_seen;
  abi.stop(box);
  abi.destroy(box);
  run.frames_released = frame->released;
  std::filesystem::remove_all(scratch);
}

}  // namespace

TEST(RealTimeHotPath, ZeroAllocationsAndNoLocksPerTick) {
  ChainRun run;
  run_chain("{}", run);
  EXPECT_EQ(run.allocations, 0) << "heap allocations in lk_step over 100000 ticks";
  EXPECT_EQ(run.locks, 0) << "locks or waits taken in lk_step over 100000 ticks";
  EXPECT_GT(run.frames_released, 0) << "no pushed plan was ever released";
}

TEST(RealTimeHotPath, ZeroAllocationsThroughTheGuardsRunawayHoldAndRelease) {
  // WHY runaway_rad 1 mrad: the 7 x 100 plan leaves 0 rad and comes back to it, so every plan runs away from the
  // two-tick-late fake arm and the guard holds; the plan's end, back within start_tolerance_rad of the held arm,
  // releases it (CONTRACT §15 A14). That is a hold, a release and two log lines in every 50 ticks.
  ChainRun run;
  run_chain(R"({"runaway_rad": 0.001})", run);
  std::printf("guard: %ld runaway holds and %ld releases logged over %d plans\n", run.runaway_logs, run.released_logs,
              run.frames_released);
  EXPECT_EQ(run.allocations, 0) << "heap allocations in lk_step over 100000 ticks of runaway holds and releases";
  EXPECT_EQ(run.locks, 0) << "locks or waits taken in lk_step over 100000 ticks of runaway holds and releases";
  EXPECT_GT(run.runaway_logs, 100) << "the guard never held a runaway (the scenario broke)";
  EXPECT_GT(run.released_logs, 100) << "the guard never released a runaway hold";
}
