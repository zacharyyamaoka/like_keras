// One created .so instance (the C ABI's lk_box): the System, its derived order, the slots between the RT thread and
// the host's threads, and the facilities' counters (CONTRACT §5, §6, §8). Internal to cpp/src: rt.cpp steps it,
// abi.cpp carries it across the C ABI. A component never includes this.
#pragma once

#include <like_keras/abi.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <like_keras/lk.hpp>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace lk::detail {

// ---------------------------------------------------------------- the wire (abi.cpp)

struct Arena {  // where an exposed Out's strings are copied on the RT thread, so the view outlives its sources
  char* data;
  std::size_t size, used = 0;
};

struct Wire {  // one v0 message type across the C ABI (CONTRACT §7.1)
  std::uint64_t hash;
  bool (*decode)(const ArrowArray& array, void* view);  // views into the array's buffers; false: refused
  void (*encode)(const void* view, ArrowArray* out);    // a fresh array the caller releases; nullptr: In only
  bool (*pin)(void* view, Arena& arena);                // strings into the arena; false: it did not fit
};
const Wire* find_wire(std::uint64_t hash);
void release(ArrowArray& array);  // its release callback, on this (never the RT) thread; then marked released
std::int64_t mono_ns();
double epoch_s();

// ---------------------------------------------------------------- lock-free slots (CONTRACT §5.1)

// WHY lifted from the controller lane's handoff.hpp (moteus_controller_node, credit): the RT thread never waits on
// a host thread, and nothing here allocates after construction. Single producer, single consumer, fixed capacity.
template <class T, std::size_t Capacity>
class Spsc {
  static_assert((Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");

 public:
  bool push(const T& value) {
    const std::uint64_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= Capacity) return false;
    slots_[head & (Capacity - 1)] = value;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }
  bool pop(T& value) {
    const std::uint64_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    value = slots_[tail & (Capacity - 1)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

 private:
  std::array<T, Capacity> slots_{};
  alignas(64) std::atomic<std::uint64_t> head_{0};
  alignas(64) std::atomic<std::uint64_t> tail_{0};
};

// The index half of handoff.hpp's triple buffer: the writer owns `back`, the reader `front`, and `middle` is
// swapped atomically with a fresh bit. One exchange per side, no lock, no wait (latest, CONTRACT §5.4).
struct Triple {
  std::atomic<std::uint8_t> middle{2};
  std::uint8_t back = 0, front = 1;
  void publish() { back = middle.exchange(static_cast<std::uint8_t>(back | 4), std::memory_order_acq_rel) & 3; }
  bool fresh() {
    if (!(middle.load(std::memory_order_relaxed) & 4)) return false;
    front = middle.exchange(front, std::memory_order_acq_rel) & 3;
    return true;
  }
};

struct Slot {  // one message: its view's bytes and the array it views into (released on a host thread)
  std::vector<std::byte> value;
  ArrowArray owner{};               // for a converted input (In<T, Converted>): its Held, which release() deletes
  const void* converted = nullptr;  // that Held's value: what the reader sees instead of `value`
  double stamp = 0.0;
  std::uint64_t seq = 0;  // an exposed Out's: which of its publishes this is (CONTRACT §15 A35)
  bool empty = true;
  std::vector<char> arena;  // an exposed Out's strings
  const void* data() const { return converted != nullptr ? converted : value.data(); }
};

struct Box;

// A queue input (CONTRACT §5.4): bounded SPSC; the reader trims to n (drop-oldest, counted) and empties it on a
// retraction; a full ring refuses the newest (counted too) so the writer never waits.
struct Ring {
  Ring(std::size_t n, std::size_t bytes);
  Slot* claim();  // writer: the next free slot, or nullptr
  void commit();  // writer
  bool ready(Box& box);
  void pop(Box& box, Port& port);
  std::uint64_t drops() const {
    return refused.load(std::memory_order_relaxed) + trimmed.load(std::memory_order_relaxed);
  }
  std::size_t n, mask;
  std::vector<Slot> slots;
  Slot current;  // the message this step's forward reads
  // WHY a counter per side: bump() is a load and a store, so one counter bumped by the host (a full exposed ring)
  // and by the RT thread (its trims) lost counts (M2 verifier, round 1). drops.<port> is their sum.
  alignas(64) std::atomic<std::uint64_t> head{0};
  std::atomic<std::uint64_t> retract_to{0}, refused{0};  // the writer's: a full ring refuses the newest
  // WHY (CONTRACT §15 A23): an exposed optional input's host may leave it unwired, and an input nothing writes must
  // not hold the gather of §5.2 forever; it gates once its host pushes to it, a retraction included (abi.h)
  std::atomic<bool> gates{true};
  alignas(64) std::atomic<std::uint64_t> tail{0};
  std::atomic<std::uint64_t> trimmed{0};  // the reader's: drop-oldest beyond n
  bool due = false;                       // the reader's: it held a message when this step's gather looked
};

struct Inlet {  // an exposed In: the host pushes (any thread, one at a time), the RT thread reads
  Port* port;
  const Wire* wire;
  Ring* ring = nullptr;  // queue inputs; latest ones use the triple buffer
  std::array<Slot, 3> slots;
  Triple triple;
};

struct Outlet {  // an exposed Out: the RT thread publishes a copy, a host thread takes it as a fresh Arrow array
  Port* port;
  const Wire* wire;
  std::string name;  // its exposed name
  std::array<Slot, 3> slots;
  Triple triple;
  std::atomic<std::uint64_t> drops{0};      // the RT thread's: a step whose strings overflowed the arena
  std::atomic<std::uint64_t> published{0};  // the RT thread's: publishes, the seq of the newest
  // CONTRACT §15 A35, the host's side: what never went out (superseded in the slot before a take, refused by the
  // transport) and the worst gap between two sends (lk_sent)
  std::atomic<std::uint64_t> taken{0}, superseded{0}, refused{0};
  std::atomic<std::int64_t> sent_ns{0}, gap_max_ns{0};
};

struct Plumbing {  // where a published Out also goes beyond its in-box readers
  std::vector<Ring*> rings;
  std::vector<Outlet*> outlets;
};

struct Lag {  // a lagged input whose writer steps BEFORE it: it reads a copy made at the end of the step
  Port* src;
  Port* dst;
  std::vector<std::byte> value;
  std::uint64_t seq = 0;
};

// Fixed bins, 8 per octave of nanoseconds; one writer (relaxed stores), the facilities thread reads.
struct Histogram {
  static constexpr std::size_t kBins = 512;
  void add(std::int64_t ns);
  std::array<std::atomic<std::uint32_t>, kBins> bins{};
  std::atomic<std::int64_t> max_ns{0};
};

enum Mode : int { kInitialization = 0, kOperational = 1, kOffline = 2 };

struct Child {  // one leaf of the box's System
  Component* component;
  std::vector<Port*> ins, outs, queued;  // queued: its queue inputs with a writer or exposed, in declaration order
  std::vector<std::size_t> upstream;     // in-box writers over non-lagged wires (a rate-less box's trigger)
  Histogram step;
  std::atomic<std::uint64_t> steps{0};
  std::array<std::atomic<const char*>, 32> metric_names{};
  std::array<std::atomic<double>, 32> metric_values{};
  std::atomic<int> metric_count{0};
  std::atomic<int> mode{kInitialization};
  std::atomic<double> activated{0.0};
  // Component::health(level, code): string literals behind a sequence count (odd while written), so the facilities
  // thread never reads one call's level with another's code; one writer, the thread the component runs on
  std::atomic<std::uint32_t> own_seq{0};
  std::atomic<const char*> own_level{nullptr}, own_code{nullptr};
  bool stepped = false;  // this step (rate-less trigger)
  // the facilities thread's side
  std::array<std::uint32_t, Histogram::kBins> step_seen{};
  std::string warning, logged_error, error;  // since the last health round; logged_error: log.error's text
};

struct LogEntry {
  std::uint32_t child;
  std::int32_t level;
  const char* fmt;
  Arg a, b;
  double wall;
  std::int64_t tick;
};

struct Box {
  explicit Box(System built);
  ~Box();
  // stepping (rt.cpp)
  void step(double at);  // one step on the calling thread: the RT thread, or lk_step's caller
  void run();            // the RT thread
  bool start(const lk_place& placed, std::string& error);
  void climb();  // host: the RT thread's scheduling ladder before its first step (CONTRACT §15 A19)
  int raise(long process, int thread, int below);  // host: lk_raise, a host or transport thread (§15 A35)
  int lower(long process, bool final);             // host: lk_lower
  void sent(std::uint32_t port, bool went);        // host: lk_sent
  void stop();
  bool send(Port& out, double at);
  void retire(ArrowArray& owner);  // RT: hand a consumed input to a host thread
  void bury();                     // host: release what the RT thread retired
  void fail(Child& child, const char* what);
  void deactivate_all();
  void write_log(const Component& component, int level, const char* fmt, Arg a = 0.0, Arg b = 0.0);
  double clock() const;  // the box's own step clock: every stamp the box takes but a log line's wall time
  // the host side (abi.cpp, rt.cpp)
  int push(std::uint32_t port, ArrowArray* msg);
  bool admit(Inlet& inlet, Slot& slot, ArrowArray& msg);
  int take(std::uint32_t port, ArrowArray* out);
  void facilities(int which);  // one round of log (0), health (1) or metrics (2) records into `pending`

  System system;
  std::deque<Child> children;  // the derived order (CONTRACT §5.3)
  std::vector<std::unique_ptr<Inlet>> ins;
  std::vector<std::unique_ptr<Outlet>> outs;
  std::vector<std::unique_ptr<Ring>> rings;
  std::deque<Plumbing> plumbing;
  std::vector<Lag> lags;
  std::atomic<double> now{0.0};
  Histogram period_error, step_time;  // step_time: the whole step, every child (the box's own metrics record)
  std::array<std::uint32_t, Histogram::kBins> period_seen{}, step_seen{};
  // One writer each: steps, overruns, unreleased by the stepping thread; log_refused by the thread that logs (the
  // host before the RT thread starts and after it is joined); log_trimmed by the facilities thread.
  std::atomic<std::uint64_t> steps{0}, overruns{0}, unreleased{0}, log_refused{0};
  std::uint64_t overruns_seen = 0, log_trimmed = 0;
  Spsc<ArrowArray, 4096> graveyard;
  std::atomic_flag burying;
  Spsc<LogEntry, 1024> logs;
  int wake_fd = -1;
  std::thread rt;
  std::atomic<int> rt_tid{0};        // the RT thread's kernel id while it runs (metrics read its policy live)
  std::atomic<int> rt_stage{0};      // 1: the RT thread tried SCHED_FIFO itself; 2: the host's ladder is done, go
  int fifo_refused = 0;              // the RT thread's pthread_setschedparam answer (0: granted)
  std::atomic<int> ladder_slot{-1};  // its place among the threads a SIGXCPU demotes (rt.cpp's Ladder)
  std::uint32_t demotions_seen = 0;
  // CONTRACT §15 A35: the host threads lk_raise put on the ladder (io_sched_policy reads them), the other processes
  // it raised threads in (lk_stop lowers them for good), and whether this box runs the process's sentinel
  std::array<std::atomic<int>, 16> io_threads{};
  std::vector<long> raised;  // under the ladder's mutex
  bool sentinel = false, realtime_over = false;
  std::atomic<bool> sends_reported{false};  // its host calls lk_sent: the box record counts every exposed Out's sends
  std::atomic<bool> running{false};
  std::atomic<std::uint32_t> arrivals{0};
  lk_place place{};
  std::vector<int> cpus;
  double epoch0 = 0.0;
  std::int64_t mono0 = 0;
  std::atomic<bool> failed{false};
  std::atomic<int> converting{0};  // In<T, Converted> conversions running on pushing threads (§15 A31)
  bool started = false;
  std::array<std::deque<ArrowArray>, 3> pending;  // facility records not taken yet, per facility
  std::array<bool, 3> round_open{};
  std::deque<LogEntry> backlog;  // drained from `logs`, not yet taken as LogRecords
  std::string sheet() const;     // lk_describe()'s JSON (abi.cpp)
};

}  // namespace lk::detail

struct lk_box : lk::detail::Box {
  using Box::Box;
};
