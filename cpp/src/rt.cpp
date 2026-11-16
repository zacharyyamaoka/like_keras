// The C++ runtime of one box (CONTRACT §5, §6.1, §8): the Component and System API, the derived order, the RT
// thread on absolute deadlines, the lock-free slots between it and the host, and the facilities (the RT thread only
// bumps counters and histogram bins; records are built on the host thread that takes them).
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <bit>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <like_keras/msgs/std.hpp>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>

#include "box.hpp"

namespace lk {
namespace {
template <class T>
void bump(std::atomic<T>& counter, T by = 1) {  // one writer: a load and a store, never a locked instruction
  counter.store(counter.load(std::memory_order_relaxed) + by, std::memory_order_relaxed);
}
constexpr std::uint32_t kBoxLog = std::numeric_limits<std::uint32_t>::max();  // a log line about the box itself
}  // namespace

Component::Component() { log.owner_ = this; }

void Component::metric(const char* name, double value) {
  if (box_ == nullptr) return;  // an eager call: no box, no facilities
  detail::Child& child = box_->children[index_];
  const int count = child.metric_count.load(std::memory_order_acquire);
  for (int slot = 0; slot < count; ++slot) {
    const char* known = child.metric_names[slot].load(std::memory_order_relaxed);
    if (known == name || std::strcmp(known, name) == 0)
      return child.metric_values[slot].store(value, std::memory_order_relaxed);
  }
  if (count == static_cast<int>(child.metric_names.size())) return;  // over 32 names (CONTRACT §6.1): not kept
  child.metric_names[count].store(name, std::memory_order_relaxed);
  child.metric_values[count].store(value, std::memory_order_relaxed);
  child.metric_count.store(count + 1, std::memory_order_release);
}

void Component::Log::info(const char* fmt, detail::Arg a, detail::Arg b) {
  if (owner_->box_) owner_->box_->write_log(*owner_, 20, fmt, a, b);
}
void Component::Log::warning(const char* fmt, detail::Arg a, detail::Arg b) {
  if (owner_->box_) owner_->box_->write_log(*owner_, 30, fmt, a, b);
}
void Component::Log::error(const char* fmt, detail::Arg a, detail::Arg b) {
  if (owner_->box_) owner_->box_->write_log(*owner_, 40, fmt, a, b);
}

double Component::period() const {  // place is written before the first activate (Box::start)
  return box_ != nullptr && box_->place.hz > 0.0 ? 1.0 / box_->place.hz : 0.0;
}

void Component::health(const char* level, const char* code) {
  if (box_ == nullptr) return;  // an eager call: no box, no facilities
  detail::Child& child = box_->children[index_];
  const std::uint32_t seq = child.own_seq.load(std::memory_order_relaxed);
  child.own_seq.store(seq + 1, std::memory_order_relaxed);  // odd: being written
  std::atomic_thread_fence(std::memory_order_release);
  child.own_level.store(level, std::memory_order_relaxed);
  child.own_code.store(code, std::memory_order_relaxed);
  child.own_seq.store(seq + 2, std::memory_order_release);
}

System::System(std::string_view name) { name_ = name; }

System::System(System&& other) noexcept
    : children_(std::move(other.children_)), wires_(std::move(other.wires_)), exposed_(std::move(other.exposed_)) {
  name_ = std::move(other.name_);
}

void System::adopt(std::unique_ptr<Component> child, std::string_view name) {
  for (const auto& known : children_)
    if (known->name_ == name) throw std::invalid_argument(name_ + ": two children named '" + std::string(name) + "'");
  child->name_ = name;
  children_.push_back(std::move(child));
}

void System::link(detail::Port& dst, detail::Port& src) { wires_.emplace_back(&dst, &src); }

void System::publish(std::string_view as, detail::Port& port) { exposed_.emplace_back(std::string(as), &port); }

void System::forward() {  // a box sorts children_ into the derived order when it is built
  for (auto& child : children_) child->forward();
}

namespace detail {

void enroll(Component* owner, Port* port) { owner->ports_.push_back(port); }

std::int64_t mono_ns() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return now.tv_sec * 1'000'000'000LL + now.tv_nsec;
}

double epoch_s() {
  timespec now{};
  clock_gettime(CLOCK_REALTIME, &now);
  return static_cast<double>(now.tv_sec) + static_cast<double>(now.tv_nsec) * 1e-9;
}

void Histogram::add(std::int64_t ns) {
  const std::uint64_t value = ns > 0 ? static_cast<std::uint64_t>(ns) : 0;
  std::size_t bin = value;
  if (value >= 8) {
    const int octave = std::bit_width(value) - 1;
    bin = static_cast<std::size_t>(octave) * 8 + ((value >> (octave - 3)) & 7);
  }
  bump(bins[bin], 1u);
  if (static_cast<std::int64_t>(value) > max_ns.load(std::memory_order_relaxed))
    max_ns.store(static_cast<std::int64_t>(value), std::memory_order_relaxed);
}

Ring::Ring(std::size_t n, std::size_t bytes)
    : n(n), mask(std::bit_ceil(std::max<std::size_t>(4, 2 * n)) - 1), slots(mask + 1) {
  for (Slot& slot : slots) slot.value.resize(bytes);
  current.value.resize(bytes);
}

Slot* Ring::claim() {
  const std::uint64_t at = head.load(std::memory_order_relaxed);
  return at - tail.load(std::memory_order_acquire) > mask ? nullptr : &slots[at & mask];
}

void Ring::commit() { head.store(head.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

bool Ring::ready(Box& box) {  // the reader trims to n (drop-oldest) and empties what a retraction covers
  std::uint64_t at = tail.load(std::memory_order_relaxed);
  const std::uint64_t end = head.load(std::memory_order_acquire),
                      retracted = retract_to.load(std::memory_order_acquire);
  for (; at < end && (at < retracted || end - at > n); ++at) {
    if (at >= retracted) bump(trimmed);
    box.retire(slots[at & mask].owner);
  }
  tail.store(at, std::memory_order_release);
  return at < end;
}

void Ring::pop(Box& box, Port& port) {  // this step's message moves into `current`; the last one is retired
  const std::uint64_t at = tail.load(std::memory_order_relaxed);
  box.retire(current.owner);
  std::swap(current, slots[at & mask]);
  tail.store(at + 1, std::memory_order_release);
  port.own.value.store(current.data(), std::memory_order_relaxed);
  port.own.stamp.store(current.stamp, std::memory_order_relaxed);
  bump(port.own.seq);
}

// ---------------------------------------------------------------- building a box

Box::Box(System built) : system(std::move(built)) {
  auto& kids = system.children_;
  const std::size_t count = kids.size();
  const auto index_of = [&](const Component* component) {
    for (std::size_t at = 0; at < count; ++at)
      if (kids[at].get() == component) return at;
    return count;
  };
  const auto where = [&](const Port* port) {
    return system.name_ + "." + port->owner->name_ + "." + std::string(port->name);
  };
  std::map<const Port*, Port*> writer;  // In -> the in-box Out wired to it
  std::vector<std::vector<std::size_t>> readers(count);
  std::vector<int> waiting(count, 0);
  for (auto [dst, src] : system.wires_) {
    if (index_of(dst->owner) == count || index_of(src->owner) == count)
      throw std::invalid_argument("wire " + where(dst) + " <- " + where(src) + ": both ends must be children of " +
                                  system.name_ + " (s.add them first)");
    if (!writer.emplace(dst, src).second)
      throw std::invalid_argument(where(dst) + " is wired twice: an input has one writer (add a merge component)");
    if (!dst->opts.lagged)
      readers[index_of(src->owner)].push_back(index_of(dst->owner)), ++waiting[index_of(dst->owner)];
  }
  std::set<std::string> names;
  std::set<const Port*> exposed_in;
  for (const auto& [name, port] : system.exposed_) {
    if (index_of(port->owner) == count)
      throw std::invalid_argument(system.name_ + ": exposed " + name + " is not a child's port");
    if (!names.insert(name).second)
      throw std::invalid_argument(system.name_ + ": two exposed ports named '" + name + "'");
    if (!port->output && writer.count(port))
      throw std::invalid_argument(where(port) + " is both wired and exposed: an input has one writer");
    if (!port->output) exposed_in.insert(port);
  }
  // CONTRACT §5.3: a topological sort over the in-box edges, lagged ones ignored; ties keep the order of add().
  std::vector<std::size_t> order;
  std::vector<bool> placed(count, false);
  while (order.size() < count) {
    std::size_t next = 0;
    while (next < count && (placed[next] || waiting[next] > 0)) ++next;
    if (next == count) {
      std::string cycle;
      for (std::size_t at = 0; at < count; ++at)
        if (!placed[at]) cycle += " " + kids[at]->name_;
      throw std::invalid_argument(system.name_ + ": a cycle with no lagged input among" + cycle +
                                  ": declare the input the device delays with {.lagged = true}");
    }
    placed[next] = true;
    order.push_back(next);
    for (std::size_t reader : readers[next]) --waiting[reader];
  }
  std::vector<std::unique_ptr<Component>> sorted;
  for (std::size_t position = 0; position < count; ++position) {
    auto& component = kids[order[position]];
    Child& child = children.emplace_back();
    child.component = component.get();
    component->box_ = this, component->clock_ = &now, component->index_ = position;
    for (Port* port : component->ports_) {
      (port->output ? child.outs : child.ins).push_back(port);
      // an optional queue input neither wired nor exposed has no ring: it reads nullptr and gates nothing (§15 A23)
      if (!port->output && port->opts.queue > 0 && (writer.count(port) || exposed_in.count(port)))
        child.queued.push_back(port);
      if (!port->output && !port->opts.optional && !writer.count(port) && !exposed_in.count(port))
        throw std::invalid_argument(where(port) +
                                    " is unbound: wire it (s.wire), expose it (s.expose) or declare "
                                    "it {.optional = true}");
      if (!port->output && port->opts.queue > 0 && port->opts.lagged)
        throw std::invalid_argument(where(port) +
                                    ": a lagged input reads the previous step's latest value, not a queue");
    }
    sorted.push_back(std::move(component));
  }
  kids = std::move(sorted);  // System::forward() runs the derived order too
  const auto position_of = [&](const Component* component) { return component->index_; };
  const auto plumb = [&](Port* out) -> Plumbing& {
    if (out->plumbing == nullptr) out->plumbing = &plumbing.emplace_back();
    return *static_cast<Plumbing*>(out->plumbing);
  };
  for (auto [dst, src] : system.wires_) {
    if (!dst->opts.lagged) children[position_of(dst->owner)].upstream.push_back(position_of(src->owner));
    if (dst->opts.queue > 0) {  // an in-box queue: the writer's publish copies into the reader's ring
      Ring& ring = *rings.emplace_back(std::make_unique<Ring>(dst->opts.queue, dst->bytes));
      plumb(src).rings.push_back(&ring);
      dst->plumbing = &ring;
    } else if (dst->opts.lagged && position_of(src->owner) < position_of(dst->owner)) {
      lags.push_back({src, dst, std::vector<std::byte>(src->bytes)});  // the writer steps first: read last step's copy
    } else {
      dst->feed = &src->own;  // CONTRACT §5.1: an edge inside a group is a pointer, no copy, no message
    }
  }
  for (const auto& [name, port] : system.exposed_) {
    const Wire* wire = find_wire(port->hash);
    if (wire == nullptr)
      throw std::invalid_argument(where(port) + ": '" + std::string(port->type) +
                                  "' has no wire in like_keras v0 (CONTRACT §7.2): an exposed port carries a v0 type");
    if (port->output) {
      if (wire->encode == nullptr)
        throw std::invalid_argument(where(port) + ": " + std::string(port->type) + " is In-only in v0 (CONTRACT §7.4)");
      Outlet& outlet = *outs.emplace_back(std::make_unique<Outlet>());
      outlet.port = port, outlet.wire = wire;
      for (Slot& slot : outlet.slots) slot.value.resize(port->bytes), slot.arena.resize(4096);
      plumb(port).outlets.push_back(&outlet);
      continue;
    }
    if (port->opts.lagged)
      throw std::invalid_argument(where(port) +
                                  ": an exposed input crosses a group, which delays it already; drop lagged");
    Inlet& inlet = *ins.emplace_back(std::make_unique<Inlet>());
    inlet.port = port, inlet.wire = wire;
    if (port->opts.queue > 0) {
      inlet.ring = rings.emplace_back(std::make_unique<Ring>(port->opts.queue, port->bytes)).get();
      inlet.ring->gates.store(!port->opts.optional, std::memory_order_relaxed);  // until its host pushes (§15 A23)
      port->plumbing = inlet.ring;
    }
    for (Slot& slot : inlet.slots) slot.value.resize(port->bytes);
  }
  wake_fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
}

Box::~Box() {
  stop();
  for (auto& inlet : ins)
    for (Slot& slot : inlet->slots) release(slot.owner);
  for (auto& ring : rings) {
    for (Slot& slot : ring->slots) release(slot.owner);
    release(ring->current.owner);
  }
  bury();
  for (auto& queue : pending)
    for (ArrowArray& array : queue) release(array);
  if (wake_fd >= 0) ::close(wake_fd);
}

// ---------------------------------------------------------------- stepping (the RT thread, or lk_step's caller)

void Box::step(double at) {
  now.store(at, std::memory_order_relaxed);
  for (auto& inlet : ins) {  // latest inputs: the newest pushed value, swapped in once, at the step's start
    if (inlet->ring != nullptr || !inlet->triple.fresh()) continue;
    const Slot& slot = inlet->slots[inlet->triple.front];
    Feed& feed = inlet->port->own;
    feed.value.store(slot.empty ? nullptr : slot.data(), std::memory_order_relaxed);
    feed.stamp.store(slot.stamp, std::memory_order_relaxed);
    bump(feed.seq);
  }
  const bool rateless = place.hz <= 0.0;
  bool published = false;
  const std::int64_t began = mono_ns();
  std::int64_t mark = began;
  for (Child& child : children) {
    child.stepped = false;
    if (child.mode.load(std::memory_order_relaxed) != kOperational) continue;
    // CONTRACT §5.2: a component with queue inputs steps when each holds a message (gather); an exposed optional
    // one its host never pushed to gates nothing and reads nullptr (§15 A23)
    bool ready = true, gated = false;
    for (Port* port : child.queued) {
      Ring& ring = *static_cast<Ring*>(port->plumbing);
      ring.due = ring.ready(*this);
      const bool gates = ring.gates.load(std::memory_order_acquire);
      gated |= gates, ready = (ring.due || !gates) && ready;
    }
    if (!ready) continue;
    if (rateless && !gated &&
        std::none_of(child.upstream.begin(), child.upstream.end(),
                     [&](std::size_t at) { return children[at].stepped; }))
      continue;
    for (Port* port : child.queued)
      if (Ring& ring = *static_cast<Ring*>(port->plumbing); ring.due) ring.pop(*this, *port);
    try {
      child.component->forward();
    } catch (const std::exception& error) {
      return fail(child, error.what());
    } catch (...) {
      return fail(child, "an exception that is not a std::exception");
    }
    const std::int64_t ended = mono_ns();
    child.step.add(ended - mark);
    mark = ended;
    bump(child.steps, std::uint64_t{1});
    child.stepped = true;
    for (Port* out : child.outs)
      if (out->written) published |= send(*out, at);
  }
  step_time.add(mark - began);  // to the end of the last child that stepped
  bump(steps, std::uint64_t{1});
  for (Lag& lag : lags) {
    const std::uint64_t seq = lag.src->own.seq.load(std::memory_order_relaxed);
    if (seq == lag.seq) continue;
    lag.seq = seq;
    std::memcpy(lag.value.data(), lag.src->value, lag.src->bytes);
    lag.dst->own.value.store(lag.value.data(), std::memory_order_relaxed);
    lag.dst->own.stamp.store(lag.src->own.stamp.load(std::memory_order_relaxed), std::memory_order_relaxed);
    bump(lag.dst->own.seq);
  }
  if (published) {  // CONTRACT §6.3: one non-blocking eventfd write per step that published, counted in overhead
    const std::uint64_t one = 1;
    (void)!::write(wake_fd, &one, sizeof one);
  }
}

bool Box::send(Port& out, double at) {
  out.written = false;
  out.own.value.store(out.value, std::memory_order_relaxed);
  out.own.stamp.store(at, std::memory_order_relaxed);
  bump(out.own.seq);
  if (out.plumbing == nullptr) return false;
  const Plumbing& plumbed = *static_cast<Plumbing*>(out.plumbing);
  for (Ring* ring : plumbed.rings) {
    Slot* slot = ring->claim();
    if (slot == nullptr) {
      bump(ring->refused);
      continue;
    }
    std::memcpy(slot->value.data(), out.value, out.bytes);
    slot->stamp = at;
    ring->commit();
  }
  for (Outlet* outlet : plumbed.outlets) {  // a copy whose strings it owns: the views may point into retired inputs
    Slot& slot = outlet->slots[outlet->triple.back];
    std::memcpy(slot.value.data(), out.value, out.bytes);
    Arena arena{slot.arena.data(), slot.arena.size()};
    if (outlet->wire->pin(slot.value.data(), arena))
      outlet->triple.publish();
    else
      bump(outlet->drops);
  }
  return !plumbed.outlets.empty();
}

void Box::retire(ArrowArray& owner) {
  if (owner.release == nullptr) return;
  if (!graveyard.push(owner)) bump(unreleased);  // full: never released rather than released on the RT thread
  owner.release = nullptr;
}

void Box::bury() {
  if (burying.test_and_set(std::memory_order_acquire)) return;  // another host thread is at it: never wait
  ArrowArray array{};
  while (graveyard.pop(array)) release(array);
  burying.clear(std::memory_order_release);
}

void Box::fail(Child& child, const char* what) {  // CONTRACT §5.7: the group stops; health says why
  child.error = what;
  failed.store(true, std::memory_order_release);
}

void Box::deactivate_all() {
  for (auto child = children.rbegin(); child != children.rend(); ++child) {
    int operational = kOperational;
    if (!child->mode.compare_exchange_strong(operational, kOffline)) continue;
    while (converting.load() != 0) std::this_thread::yield();  // a conversion that saw it operational ends first (A31)
    try {
      child->component->deactivate();
    } catch (...) {  // deactivate must not throw; if it does, keep shutting down the rest
    }
  }
}

void Box::write_log(const Component& component, int level, const char* fmt, Arg a, Arg b) {
  const bool own = &component != &system;
  const std::uint64_t tick = own ? children[component.index_].steps.load(std::memory_order_relaxed) : 0;
  // WHY wall time, not clock(): CONTRACT §2.3 stamps a LogRecord with the wall time of the record, as Python's
  // logging.LogRecord.created does under lockstep too. It is never subtracted from a step time.
  const LogEntry entry{own ? static_cast<std::uint32_t>(component.index_) : kBoxLog,
                       level,
                       fmt,
                       a,
                       b,
                       epoch_s(),
                       static_cast<std::int64_t>(tick)};
  if (!logs.push(entry)) bump(log_refused);
}

// WHY one clock for every stamp (activation, arrival, health and metrics): health's uptime subtracted a wall-clock
// activation (epoch s) from the logical step time, -1.79e9 s under lockstep, which robot_msgs refused (M2 verifier,
// round 1). Stepped, it is the last lk_step's time, 0.0 before the first, where CONTRACT §5.5 starts logical time
// (Python's inproc activates there too); in real time, epoch seconds on the monotonic clock mapped at start.
double Box::clock() const {
  if (place.stepped) return now.load(std::memory_order_relaxed);
  return started ? epoch0 + static_cast<double>(mono_ns() - mono0) * 1e-9 : epoch_s();
}

// ---------------------------------------------------------------- the RT thread's scheduling ladder (CONTRACT §15 A19)
//
// SCHED_FIFO at place.fifo, else rtkit's SCHED_RR, else SCHED_OTHER, reported in the box's metrics (sched_policy as
// Linux numbers it, sched_priority). Lifted, with credit, from the controller lane's first node
// (moteus_controller_node cpp/src/rt.cpp and rt_sentinel.cpp, fix rounds 3-6), where it took six verify-and-fix
// rounds to make safe; what carried over, and why:
// - rtkit is asked from a host thread, with the RT thread's id, before the RT thread's first step: a busctl call
//   takes 10-18 ms here, never on the RT thread. Through busctl, not a D-Bus library (§2.1: the standard library
//   only), and only from the host's PID namespace: rtkit names threads by their host ids, so from a private one
//   (kpi/run.py) it would name another process's thread.
// - rtkit grants only under RLIMIT_RTTIME with hard <= its RTTimeUSecMax, so it is set soft < hard (100 / 200 ms
//   here): at the soft limit the kernel sends SIGXCPU, which demotes instead of killing (on_sigxcpu); soft == hard
//   gave SIGKILL with no SIGXCPU first. The kernel raises the soft limit by 1 s on each SIGXCPU, past the hard one,
//   so the handler puts it back. The hard limit cannot be raised again unprivileged: a process that once got an
//   rtkit grant keeps it, which only bites a thread that is realtime and never blocks.
// - Every grant ends with the run: the RT thread demotes itself before it exits, and no request is in flight after
//   lk_start (each one is waited for, 1 s at most, then SIGKILLed).
// WHY a handler, not the raw node's signalfd sentinel: a component runs in someone's process (a Python one under
// inproc), whose threads cannot all block SIGXCPU. The kernel hands it to the thread that ran over, which runs the
// handler on its way back to user space and demotes itself; any other thread that takes it demotes it by id.
namespace {

constexpr ino_t kHostPidNamespace = 0xEFFFFFFC;  // PROC_PID_INIT_INO

struct Ladder {  // process-wide: every RT thread a SIGXCPU demotes, whichever thread takes the signal
  std::array<std::atomic<int>, 16> threads{};
  std::array<std::atomic<std::uint32_t>, 16> demotions{};
  rlimit budget{RLIM_INFINITY, RLIM_INFINITY};  // what each SIGXCPU puts back
  struct sigaction previous {};
  int users = 0;
  std::mutex mutex;  // host threads only (Box::climb, Box::stop)
} ladder;

bool realtime(int thread) {  // 0: the calling thread
  const int policy = ::sched_getscheduler(thread) & ~SCHED_RESET_ON_FORK;
  return policy == SCHED_FIFO || policy == SCHED_RR;
}

bool demote(int thread) {
  const sched_param normal{};
  // WHY keep SCHED_RESET_ON_FORK: rtkit grants with it set, and an unprivileged thread may not clear it (EPERM)
  return ::sched_setscheduler(thread, SCHED_OTHER | SCHED_RESET_ON_FORK, &normal) == 0 ||
         ::sched_setscheduler(thread, SCHED_OTHER, &normal) == 0;
}

void on_sigxcpu(int number, siginfo_t* info, void* context) {  // async-signal-safe: syscalls and atomics only
  const int saved = errno;
  for (std::size_t slot = 0; slot < ladder.threads.size(); ++slot) {
    const int thread = ladder.threads[slot].load(std::memory_order_acquire);
    if (thread != 0 && realtime(thread) && demote(thread))
      ladder.demotions[slot].fetch_add(1, std::memory_order_release);
  }
  ::setrlimit(RLIMIT_RTTIME, &ladder.budget);
  const struct sigaction& previous = ladder.previous;
  if ((previous.sa_flags & SA_SIGINFO) && previous.sa_sigaction != nullptr)
    previous.sa_sigaction(number, info, context);
  else if (!(previous.sa_flags & SA_SIGINFO) && previous.sa_handler != SIG_DFL && previous.sa_handler != SIG_IGN)
    previous.sa_handler(number);
  errno = saved;
}

// `busctl --system <arguments>`: true when it answered within 1 s and exited 0, its stdout in `out`. WHY the bound:
// a stuck request must never hold lk_start, and one SIGKILLed is reaped here, so none is in flight after it.
bool busctl(std::vector<std::string> arguments, std::string& out) {
  arguments.insert(arguments.begin(), {"busctl", "--system"});
  std::vector<char*> argv;
  for (std::string& argument : arguments) argv.push_back(argument.data());
  argv.push_back(nullptr);
  int ends[2];
  if (::pipe2(ends, O_CLOEXEC) != 0) return false;
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, ends[1], 1);
  posix_spawn_file_actions_addopen(&actions, 2, "/dev/null", O_WRONLY, 0);
  pid_t child = 0;
  const int spawned = ::posix_spawnp(&child, "busctl", &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  ::close(ends[1]);
  bool ended = false;
  for (const std::int64_t deadline = mono_ns() + 1'000'000'000; spawned == 0 && !ended;) {
    pollfd ready{ends[0], POLLIN, 0};
    const int waited = ::poll(&ready, 1, static_cast<int>(std::max<std::int64_t>(0, deadline - mono_ns()) / 1'000'000));
    if (waited < 0 && errno == EINTR) continue;
    if (waited <= 0) break;
    char chunk[256];
    const ssize_t got = ::read(ends[0], chunk, sizeof chunk);
    if (got > 0) out.append(chunk, static_cast<std::size_t>(got));
    ended = got == 0 || (got < 0 && errno != EINTR);
  }
  ::close(ends[0]);
  if (spawned != 0) return false;
  if (!ended) ::kill(child, SIGKILL);
  int status = 0;
  while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {
  }
  return ended && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

// rtkit's SCHED_RR for `thread` at min(wanted, rtkit's highest): nullptr when granted, else why not
const char* ask_rtkit(int thread, int wanted) {
  struct stat namespace_of {};
  if (::stat("/proc/self/ns/pid", &namespace_of) != 0 || namespace_of.st_ino != kHostPidNamespace)
    return "rtkit cannot name the RT thread from a private PID namespace";
  const std::vector<std::string> rtkit{"org.freedesktop.RealtimeKit1", "/org/freedesktop/RealtimeKit1",
                                       "org.freedesktop.RealtimeKit1"};
  std::string answer;
  int highest = 0;
  long long rttime_max = 0;
  auto ask = rtkit;
  ask.insert(ask.begin(), "get-property"), ask.insert(ask.end(), {"MaxRealtimePriority", "RTTimeUSecMax"});
  if (!busctl(ask, answer) || std::sscanf(answer.c_str(), "i %d x %lld", &highest, &rttime_max) != 2 || highest < 1 ||
      rttime_max < 2)
    return "no rtkit answered on the system bus";
  rlimit limit{};
  ::getrlimit(RLIMIT_RTTIME, &limit);
  const rlim_t hard = std::min<rlim_t>(limit.rlim_max, static_cast<rlim_t>(rttime_max));
  const rlimit budget{limit.rlim_cur < hard ? limit.rlim_cur : hard / 2, hard};
  if (::setrlimit(RLIMIT_RTTIME, &budget) != 0) return "RLIMIT_RTTIME could not be set";
  ask = rtkit;
  ask.insert(ask.begin(), "call");
  ask.insert(ask.end(), {"MakeThreadRealtimeWithPID", "ttu", std::to_string(::getpid()), std::to_string(thread),
                         std::to_string(std::min(wanted, highest))});
  busctl(ask, answer);
  return realtime(thread) ? nullptr : "rtkit refused the RT thread";
}

}  // namespace

void Box::climb() {
  while (rt_stage.load(std::memory_order_acquire) == 0) rt_stage.wait(0, std::memory_order_acquire);
  const int thread = rt_tid.load(std::memory_order_relaxed);
  try {
    const char* refused = place.fifo > 0 && fifo_refused != 0 ? ask_rtkit(thread, place.fifo) : nullptr;
    if (refused != nullptr)
      write_log(system, 30, "SCHED_FIFO %g refused, and %s: the RT thread runs SCHED_OTHER", place.fifo, refused);
    else if (place.fifo > 0 && fifo_refused != 0)
      write_log(system, 20, "SCHED_FIFO %g refused: rtkit granted SCHED_RR under RLIMIT_RTTIME", place.fifo);
    if (realtime(thread)) {  // enlisted for on_sigxcpu, whatever granted it
      std::lock_guard guard(ladder.mutex);
      ::getrlimit(RLIMIT_RTTIME, &ladder.budget);
      for (std::size_t slot = 0; slot < ladder.threads.size() && ladder_slot.load() < 0; ++slot) {
        if (ladder.threads[slot].load() != 0) continue;
        demotions_seen = ladder.demotions[slot].load();
        ladder.threads[slot].store(thread), ladder_slot.store(static_cast<int>(slot));
      }
      if (ladder_slot.load() >= 0 && ladder.users++ == 0) {
        struct sigaction action {};
        action.sa_sigaction = on_sigxcpu, action.sa_flags = SA_SIGINFO | SA_RESTART;
        sigemptyset(&action.sa_mask);
        ::sigaction(SIGXCPU, &action, &ladder.previous);
      }
    }
  } catch (const std::exception&) {  // never fatal: the thread runs on whatever it has
  }
  mono0 = mono_ns(), epoch0 = epoch_s();  // after the ladder: tick 0 is not already late
  rt_stage.store(2, std::memory_order_release);
  rt_stage.notify_all();
}

bool Box::start(const lk_place& placed, std::string& error) {
  place = placed;
  cpus.assign(placed.cpus, placed.cpus + (placed.cpus ? placed.ncpus : 0));
  place.cpus = cpus.data();
  for (Child& child : children) {  // CONTRACT §5.7: activate in the derived order, before the first step
    try {
      child.component->activate();
    } catch (const std::exception& refused) {
      error = system.name_ + "." + child.component->name_ + ".activate(): " + refused.what();
      deactivate_all();
      return false;
    }
    child.activated.store(clock(), std::memory_order_relaxed);
    child.mode.store(kOperational, std::memory_order_release);
  }
  mono0 = mono_ns(), epoch0 = epoch_s(), started = true;  // after activation: tick 0 is not already late
  if (placed.stepped) return true;
  // WHY only on request: mlockall locks the whole process, which inside a Python process (inproc) would pin the
  // interpreter's heap too; there, run() pre-faults the RT thread's stack and the slots are touched when built.
  if (placed.lock_memory && ::mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    write_log(system, 30, "mlockall refused (errno %g): pages may fault on the RT thread", errno);
  running.store(true, std::memory_order_release);
  rt = std::thread([this] { run(); });
  climb();
  return true;
}

void Box::run() {
  if (!cpus.empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int cpu : cpus) CPU_SET(cpu, &set);
    if (const int refused = pthread_setaffinity_np(pthread_self(), sizeof set, &set))
      write_log(system, 30, "the RT thread's cpus were refused (errno %g): it runs on the process's", refused);
  }
  // WHY a ladder, not a refusal: SCHED_FIFO needs CAP_SYS_NICE or an rtprio limit, which a lane or a user session
  // often lacks (`ulimit -r` is 0 here). Then climb() asks rtkit, and else the thread runs SCHED_OTHER on the same
  // absolute deadlines, logged at warning, so health shows it rather than the jitter alone (§4.3: never silently).
  rt_tid.store(static_cast<int>(::gettid()), std::memory_order_relaxed);
  if (place.fifo > 0) {
    sched_param priority{};
    priority.sched_priority = place.fifo;
    fifo_refused = pthread_setschedparam(pthread_self(), SCHED_FIFO, &priority);
  }
  rt_stage.store(1, std::memory_order_release);
  rt_stage.notify_all();
  while (rt_stage.load(std::memory_order_acquire) != 2) rt_stage.wait(1, std::memory_order_acquire);
  volatile std::byte stack[256 * 1024];  // pre-fault the stack the steps will use
  for (std::size_t at = 0; at < sizeof stack; at += 4096) stack[at] = std::byte{0};
  std::int64_t tick = 0;
  std::uint32_t handled = 0;
  while (running.load(std::memory_order_acquire)) {
    if (place.hz <= 0.0) {  // rate-less (CONTRACT §5.2): a step per queued message
      arrivals.wait(handled, std::memory_order_acquire);
      handled = arrivals.load(std::memory_order_acquire);
      if (!running.load(std::memory_order_acquire)) break;
      step(clock());
      if (failed.load(std::memory_order_acquire)) break;
      continue;
    }
    // CONTRACT §5.2: absolute deadlines from the start; now() is the scheduled time, epoch0 + tick / hz.
    const std::int64_t deadline = mono0 + std::llround(static_cast<double>(tick) * 1e9 / place.hz);
    const timespec until{static_cast<time_t>(deadline / 1'000'000'000), static_cast<long>(deadline % 1'000'000'000)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &until, nullptr) == EINTR) {
    }
    if (!running.load(std::memory_order_acquire)) break;
    period_error.add(mono_ns() - deadline);
    step(epoch0 + static_cast<double>(tick) / place.hz);
    if (failed.load(std::memory_order_acquire)) break;
    // An overrun skips the ticks whose deadline has passed, counted, and never catches up with a burst.
    const auto late = static_cast<std::int64_t>(std::floor(static_cast<double>(mono_ns() - mono0) * place.hz / 1e9));
    const std::int64_t next = std::max(tick + 1, late + 1);
    bump(overruns, static_cast<std::uint64_t>(next - tick - 1));
    tick = next;
  }
  if (realtime(0)) demote(0);  // CONTRACT §15 A19: no grant outlives the run, a failed one included
  if (const int slot = ladder_slot.load(); slot >= 0) ladder.threads[static_cast<std::size_t>(slot)].store(0);
  rt_tid.store(0, std::memory_order_release);
  if (failed.load(std::memory_order_acquire)) deactivate_all();  // §5.7: the group deactivates
}

void Box::stop() {
  if (running.exchange(false)) {
    arrivals.fetch_add(1, std::memory_order_release);
    arrivals.notify_all();
  }
  if (rt.joinable()) rt.join();
  if (ladder_slot.load() >= 0) {  // the last box of this runtime puts SIGXCPU back as it found it
    std::lock_guard guard(ladder.mutex);
    struct sigaction current {};
    ::sigaction(SIGXCPU, nullptr, &current);
    if (--ladder.users == 0 && (current.sa_flags & SA_SIGINFO) && current.sa_sigaction == on_sigxcpu)
      ::sigaction(SIGXCPU, &ladder.previous, nullptr);
    ladder_slot = -1;
  }
  deactivate_all();  // CONTRACT §5.7: reverse order; idempotent
}

namespace {
void drop_held(ArrowArray* array) {  // a converted value's release, run on a host thread like any other
  delete static_cast<Held*>(array->private_data);
  array->release = nullptr;
}
}  // namespace

// On the pushing thread: msg decoded into the slot's view, and for an In<T, Converted> converted right here
// (CONTRACT §15 A17: allocation and validation off the RT thread), its arrays then released at once. The slot's
// owner is what a host thread releases once the reader has moved on. False: refused (msg released).
bool Box::admit(Inlet& inlet, Slot& slot, ArrowArray& msg) {
  slot.converted = nullptr;
  if (!inlet.wire->decode(msg, slot.value.data())) return release(msg), false;
  if (inlet.port->convert == nullptr) {
    slot.owner = msg, msg.release = nullptr;
    return true;
  }
  // WHY a window (CONTRACT §15 A31, consumer-014): a conversion may read its owner's params and what activate()
  // built, so it runs only while the owner is operational. The count and the mode are seq_cst on both sides (here
  // and deactivate_all), so either this push sees the owner offline or deactivate() waits for this conversion.
  Held* held = nullptr;
  converting.fetch_add(1);
  try {
    if (children[inlet.port->owner->index_].mode.load() == kOperational)
      held = inlet.port->convert(slot.value.data(), inlet.port->owner);
  } catch (...) {  // Converted::from refused it (validation): counted as a refused push, never thrown into C
  }
  converting.fetch_sub(1);
  release(msg);
  if (held == nullptr) return false;
  slot.owner = ArrowArray{0, 0, 0, 0, 0, nullptr, nullptr, nullptr, drop_held, held};
  slot.converted = held->value;
  return true;
}

int Box::push(std::uint32_t port, ArrowArray* msg) {
  bury();
  if (port >= ins.size()) {
    if (msg != nullptr) release(*msg);
    return -1;
  }
  Inlet& inlet = *ins[port];
  const double stamp = clock();
  if (inlet.ring != nullptr) {
    Ring& ring = *inlet.ring;
    ring.gates.store(true, std::memory_order_release);  // its host writes it: it is wired (§15 A23), before any commit
    if (msg == nullptr) {  // retracted: the reader empties what was queued before now
      ring.retract_to.store(ring.head.load(std::memory_order_relaxed), std::memory_order_release);
      return 0;
    }
    Slot* slot = ring.claim();
    if (slot == nullptr) {
      bump(ring.refused);
      release(*msg);
      return 0;
    }
    if (!admit(inlet, *slot, *msg)) return -1;
    slot->stamp = stamp;
    ring.commit();
    if (place.hz <= 0.0) {  // only a queued message triggers a rate-less box; latest never does
      arrivals.fetch_add(1, std::memory_order_release);
      arrivals.notify_one();
    }
    return 0;
  }
  Slot& slot = inlet.slots[inlet.triple.back];
  // the RT thread reads it no more: it moved off this slot at an earlier step's start, or never read it (superseded)
  release(slot.owner);
  slot.empty = true;
  if (msg != nullptr) {
    if (!admit(inlet, slot, *msg)) return -1;
    slot.empty = false;
  }
  slot.stamp = stamp;
  inlet.triple.publish();
  return 0;
}

int Box::take(std::uint32_t port, ArrowArray* out) {
  bury();
  if (port < outs.size()) {
    Outlet& outlet = *outs[port];
    if (!outlet.triple.fresh()) return 0;
    outlet.wire->encode(outlet.slots[outlet.triple.front].value.data(), out);
    return 1;
  }
  const std::size_t which = port - outs.size();
  if (which >= pending.size()) return -1;
  auto& queue = pending[which];
  if (queue.empty()) {  // a round: every record, then 0 once, then the next round
    if (std::exchange(round_open[which], false)) return 0;
    facilities(static_cast<int>(which));
    if (queue.empty()) return 0;
    round_open[which] = true;
  }
  *out = queue.front();
  queue.pop_front();
  return 1;
}

// ---------------------------------------------------------------- facilities (CONTRACT §8), on a host thread

namespace {

double bin_upper_s(std::size_t bin) {
  if (bin < 8) return static_cast<double>(bin + 1) * 1e-9;
  const std::size_t octave = bin / 8, sub = bin % 8;
  return static_cast<double>((9 + sub) << (octave - 3)) * 1e-9;
}

// The percentiles of the bins counted since `seen` (which then catches up), as Python's facilities compute them:
// the value at index int(n * p / 100) of the sorted period. NaN when nothing was counted.
std::array<double, 2> percentiles(const Histogram& histogram, std::array<std::uint32_t, Histogram::kBins>& seen) {
  std::array<std::uint32_t, Histogram::kBins> counted{};
  std::uint64_t total = 0;
  for (std::size_t bin = 0; bin < Histogram::kBins; ++bin) {
    const std::uint32_t now = histogram.bins[bin].load(std::memory_order_relaxed);
    counted[bin] = now - seen[bin], seen[bin] = now, total += counted[bin];
  }
  std::array<double, 2> out{std::nan(""), std::nan("")};
  const std::array<double, 2> wanted{50.0, 99.0};
  for (std::size_t which = 0; which < 2 && total > 0; ++which) {
    const auto index = std::min<std::uint64_t>(total - 1, static_cast<std::uint64_t>(total * wanted[which] / 100));
    std::uint64_t below = 0;
    for (std::size_t bin = 0; bin < Histogram::kBins; ++bin)
      if ((below += counted[bin]) > index) {
        out[which] = bin_upper_s(bin);
        break;
      }
  }
  return out;
}

std::string formatted(const LogEntry& entry) {
  char text[512];
  const Arg &a = entry.a, &b = entry.b;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
  // CONTRACT §6.1, §15 A22: a literal and two arguments, each a number (%g) or a string literal (%s)
  if (a.text != nullptr && b.text != nullptr)
    std::snprintf(text, sizeof text, entry.fmt, a.text, b.text);
  else if (a.text != nullptr)
    std::snprintf(text, sizeof text, entry.fmt, a.text, b.number);
  else if (b.text != nullptr)
    std::snprintf(text, sizeof text, entry.fmt, a.number, b.text);
  else
    std::snprintf(text, sizeof text, entry.fmt, a.number, b.number);
#pragma GCC diagnostic pop
  return text;
}

const char* const kModes[] = {"initialization", "operational", "offline"};

int severity(std::string_view level) {  // what "the worse" of two health levels means (CONTRACT §15 A22)
  return level == "error" ? 3 : level == "warn" ? 2 : level == "ok" ? 0 : 1;
}

// The component's own health(level, code), read whole (Child::own_seq); nullptr level: never called
std::pair<const char*, const char*> own_health(const Child& child) {
  for (;;) {
    const std::uint32_t before = child.own_seq.load(std::memory_order_acquire);
    const char* level = child.own_level.load(std::memory_order_relaxed);
    const char* code = child.own_code.load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (before % 2 == 0 && child.own_seq.load(std::memory_order_relaxed) == before) return {level, code};
  }
}

}  // namespace

void Box::facilities(int which) {
  LogEntry entry{};
  const auto take_in = [&] {
    for (std::uint32_t index = 0; entry.level >= 30 && index < children.size(); ++index)  // a box line warns them all
      if (entry.child == index || entry.child == kBoxLog)
        (entry.level >= 40 ? children[index].logged_error : children[index].warning) = formatted(entry);
    backlog.push_back(entry);
    if (backlog.size() > 4096) backlog.pop_front(), ++log_trimmed;  // nobody takes the log port: keep the newest
  };
  while (logs.pop(entry)) take_in();
  if (const int slot = ladder_slot.load(); slot >= 0) {  // CONTRACT §15 A19: what on_sigxcpu did, said here
    const std::uint32_t demoted = ladder.demotions[static_cast<std::size_t>(slot)].load(std::memory_order_acquire);
    if (demoted != std::exchange(demotions_seen, demoted)) {
      entry = {kBoxLog, 30, "the RT thread ran past RLIMIT_RTTIME's soft limit (SIGXCPU): demoted to SCHED_OTHER",
               {},      {}, epoch_s(),
               0};
      take_in();
    }
  }
  const double at = clock();
  const auto encode = [&](const auto& record, std::uint64_t hash) {
    ArrowArray array{};
    find_wire(hash)->encode(&record, &array);
    pending[which].push_back(array);
  };
  const auto name_of = [&](std::uint32_t child) {
    return child < children.size() ? std::string_view(children[child].component->name_) : std::string_view();
  };
  if (which == 0) {
    for (const LogEntry& line : backlog) {
      const std::string message = formatted(line);
      rm::LogRecord record{};
      record.header.stamp = line.wall, record.path = name_of(line.child), record.tick = line.tick;
      record.level = line.level, record.message = message;
      encode(record, rm::LogRecord::kHash);
    }
    backlog.clear();
    return;
  }
  const bool broken = failed.load(std::memory_order_acquire);
  const std::uint64_t overran = overruns.load(std::memory_order_relaxed);
  const auto period = which == 2 ? percentiles(period_error, period_seen) : std::array<double, 2>{};
  using Values = std::vector<std::pair<std::string, double>>;
  const auto standard = [&](Histogram& took, std::array<std::uint32_t, Histogram::kBins>& seen, std::uint64_t count) {
    const auto step = percentiles(took, seen);
    const std::int64_t longest = took.max_ns.exchange(0, std::memory_order_relaxed);
    return Values{{"steps", static_cast<double>(count)},
                  {"step_p50_s", step[0]},
                  {"step_p99_s", step[1]},
                  {"step_max_s", std::isnan(step[0]) ? step[0] : static_cast<double>(longest) * 1e-9},
                  {"period_error_p99_s", period[1]},
                  {"overruns", static_cast<double>(overran)}};
  };
  const auto publish = [&](std::string_view path, const Values& values) {
    rm::Metrics record{};
    record.header.stamp = at, record.path = path;
    record.size = std::min(values.size(), record.names.size());
    for (std::size_t slot = 0; slot < record.size; ++slot)
      record.names[slot] = values[slot].first, record.values[slot] = values[slot].second;
    encode(record, rm::Metrics::kHash);
  };
  // WHY the box's own record (path "", which the host names the box's path): the log ring and the graveyard belong
  // to no child, and a dropped log line or an unreleased message must be counted where someone can see it (M2
  // verifier, round 1). WHY first: its steps count at a step's end, so it never exceeds a child's read after it.
  if (which == 2) {
    Values box = standard(step_time, step_seen, steps.load(std::memory_order_relaxed));
    box.emplace_back("drops.log", static_cast<double>(log_refused.load(std::memory_order_relaxed) + log_trimmed));
    box.emplace_back("unreleased", static_cast<double>(unreleased.load(std::memory_order_relaxed)));
    sched_param param{};  // CONTRACT §15 A19: what the RT thread runs under now, read from the thread itself
    if (const int thread = rt_tid.load(std::memory_order_acquire);
        thread != 0 && ::sched_getparam(thread, &param) == 0) {
      box.emplace_back("sched_policy", ::sched_getscheduler(thread) & ~SCHED_RESET_ON_FORK);
      box.emplace_back("sched_priority", param.sched_priority);
    }
    publish("", box);
  }
  for (std::uint32_t index = 0; index < children.size(); ++index) {
    Child& child = children[index];
    const int mode = child.mode.load(std::memory_order_acquire);
    if (which == 1) {
      std::vector<std::string> stale;
      for (Port* in : child.ins)
        if (in->hid_stale.exchange(false, std::memory_order_relaxed)) stale.emplace_back(in->name);
      rm::DiagnosticRecord record{};
      std::string message;
      record.level = "ok";
      if (broken && !child.error.empty()) {
        record.level = "error", record.code = "Exception", message = child.error;
      } else if (!child.logged_error.empty()) {  // CONTRACT §15 A22: until a round in which no error was logged
        record.level = "error", record.code = "ErrorLogged", message = child.logged_error;
      } else if (!stale.empty()) {
        record.level = "warn", record.code = "InputStale", message = "stale:";
        for (const auto& name : stale) message += (message.size() > 6 ? ", " : " ") + name;
      } else if (overran > overruns_seen) {
        record.level = "warn", record.code = "Overrun",
        message = std::to_string(overran - overruns_seen) + " ticks skipped";
      } else if (!child.warning.empty()) {
        record.level = "warn", record.code = "WarningLogged", message = child.warning;
      } else if (mode == kInitialization) {
        record.level = "unknown";
      }
      // CONTRACT §15 A22: the worse of the component's own health(level, code) and the facility's; on a tie the
      // component's code says more, but an Exception always wins (the component no longer runs to say otherwise).
      // WHY a tie keeps the facility's message (health() carries none): as in Python, where the attach proxy's
      // LeaseHeld names the holder only through its log.error line (§4.5; fix round 1, item 4)
      const auto [own_level, own_code] = own_health(child);
      if (own_level != nullptr && record.code != "Exception" &&
          severity(own_level) >= std::max(severity(record.level), 1)) {
        if (severity(own_level) > severity(record.level)) message.clear();
        record.level = own_level, record.code = own_code != nullptr ? own_code : "";
      }
      child.warning.clear(), child.logged_error.clear();
      record.header.stamp = at, record.path = name_of(index), record.message = message, record.mode = kModes[mode];
      record.expected_period_s = 1.0;
      record.uptime_s = mode == kOperational ? at - child.activated.load(std::memory_order_relaxed) : 0.0;
      record.size = std::min(stale.size(), record.stale_inputs.size());
      for (std::size_t at_name = 0; at_name < record.size; ++at_name) record.stale_inputs[at_name] = stale[at_name];
      encode(record, rm::DiagnosticRecord::kHash);
      continue;
    }
    Values values = standard(child.step, child.step_seen, child.steps.load(std::memory_order_relaxed));
    for (Port* in : child.ins) {
      if (in->opts.queue > 0) {  // an unwired optional queue has no ring and drops nothing (§15 A23)
        values.emplace_back("drops." + std::string(in->name),
                            in->plumbing == nullptr ? 0.0 : static_cast<Ring*>(in->plumbing)->drops());
      } else {
        const bool has = in->feed->value.load(std::memory_order_relaxed) != nullptr;
        values.emplace_back("input_age_s." + std::string(in->name),
                            has ? at - in->feed->stamp.load(std::memory_order_relaxed) : std::nan(""));
      }
    }
    for (Port* out : child.outs) {  // an exposed Out whose strings overflowed the 4 KiB arena was not published
      if (out->plumbing == nullptr || static_cast<Plumbing*>(out->plumbing)->outlets.empty()) continue;
      std::uint64_t dropped = 0;
      for (const Outlet* outlet : static_cast<Plumbing*>(out->plumbing)->outlets)
        dropped += outlet->drops.load(std::memory_order_relaxed);
      values.emplace_back("drops." + std::string(out->name), static_cast<double>(dropped));
    }
    const int custom = child.metric_count.load(std::memory_order_acquire);
    for (int slot = 0; slot < custom; ++slot) {
      const std::string name = child.metric_names[slot].load(std::memory_order_relaxed);
      const bool standard_name =
          std::any_of(values.begin(), values.begin() + 6, [&](const auto& kept) { return kept.first == name; });
      if (!standard_name && name.rfind("input_age_s.", 0) && name.rfind("drops.", 0) &&
          name.rfind("refused_writes.", 0))
        values.emplace_back(name,
                            child.metric_values[slot].load(std::memory_order_relaxed));  // a standard name is refused
    }
    publish(name_of(index), values);
  }
  if (which == 1) overruns_seen = overran;
}

}  // namespace detail
}  // namespace lk
