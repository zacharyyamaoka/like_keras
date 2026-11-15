// build/kpi/bench_chain (Google Benchmark 1.8.3): the 4-component chain stepped through the C ABI (lk_step on
// libchain4.so) against the same four bodies called by hand. kpi/bench_overhead.py reads the JSON:
// per edge = (BM_LkChain - BM_HandChain) / 3. The C ABI is declared here from CONTRACT §6.3.
#include <benchmark/benchmark.h>
#include <dlfcn.h>

#include <cstdint>
#include <cstdio>
#include <string>

#include <like_keras/msgs/sensor.hpp>

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

// The hand-written chain: the same bodies as chain4.cpp's components, the edges as pointers.
struct HandChain {
  rm::JointState head{}, first{}, second{};
  double sum = 0.0;
  void tick(double now) {
    head.size = 7;
    head.position[0] = now;
    first.size = head.size;
    first.position[0] = head.position[0] + 1.0;
    second.size = first.size;
    second.position[0] = first.position[0] + 1.0;
    sum += second.position[0];
  }
};

void BM_HandChain(benchmark::State& state) {
  HandChain chain;
  double now = 0.0;
  for (auto _ : state) {
    chain.tick(now);
    now += 0.001;
    benchmark::DoNotOptimize(chain.sum);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_HandChain);

void BM_LkChain(benchmark::State& state) {
  const std::string path = std::string(KPI_BUILD_DIR) + "/libchain4.so";
  void* library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (library == nullptr) {
    state.SkipWithError(dlerror());
    return;
  }
  auto create = reinterpret_cast<lk_box* (*)(const char*, char*, size_t)>(dlsym(library, "lk_create"));
  auto start = reinterpret_cast<int (*)(lk_box*, const lk_place*, char*, size_t)>(dlsym(library, "lk_start"));
  auto step = reinterpret_cast<int (*)(lk_box*, double)>(dlsym(library, "lk_step"));
  auto stop = reinterpret_cast<void (*)(lk_box*)>(dlsym(library, "lk_stop"));
  auto destroy = reinterpret_cast<void (*)(lk_box*)>(dlsym(library, "lk_destroy"));
  char error[512] = {0};
  lk_box* box = create("{\"unused\": 0}", error, sizeof(error));
  const lk_place place{1000.0, nullptr, 0, 0, 0, 1};
  if (box == nullptr || start(box, &place, error, sizeof(error)) != 0) {
    state.SkipWithError(error);
    return;
  }
  double now = 0.0;
  for (auto _ : state) {
    step(box, now);
    now += 0.001;
  }
  stop(box);
  destroy(box);
}
BENCHMARK(BM_LkChain);

}  // namespace

BENCHMARK_MAIN();
