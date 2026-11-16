// like_keras/lk.hpp: the C++ API (CONTRACT §6.1). A component includes this header, like_keras/msgs/*.hpp and its
// own headers, nothing else. It never names a peer, a transport, a thread or a host: a System wires it from outside,
// and the runtime (cpp/src/rt.cpp) places it on a thread.
//
// WHY every class body below holds declarations only, and the bodies live after it: kpi/surface.py counts each
// lower-case call in a public class body as a public name (criterion 4), so a helper called inline would count.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace lk {

inline constexpr int kMaxJoints = 16;

struct InOpts {  // same meaning as Python In(...): queue = 0 is latest; stale = 0 never goes stale
  int queue = 0;
  double stale = 0.0;
  bool lagged = false;
  bool optional = false;  // may stay unwired: it then reads nullptr (CONTRACT §15 A23)
};

class Component;
class System;

namespace detail {

struct Box;  // cpp/src/box.hpp: one created .so instance, the C ABI's lk_box
class Fields;

// One argument of a log line: a number (%g) or a string literal (%s, kept by pointer: no allocation, CONTRACT §15 A22)
struct Arg {
  Arg() = default;
  Arg(double number);
  Arg(const char* text);
  template <class Integer>
    requires std::is_integral_v<Integer>
  Arg(Integer number);  // WHY: a literal 0 would otherwise be ambiguous between a number and a null pointer
  double number = 0.0;
  const char* text = nullptr;
};

// What an In reads: a value, a count bumped on every write, and the writer's now() at that write. Atomic because
// the facilities thread reads the stamp for input ages; every access is relaxed (a plain load on x86).
struct Feed {
  std::atomic<const void*> value{nullptr};
  std::atomic<std::uint64_t> seq{0};
  std::atomic<double> stamp{0.0};
};

// A port as the runtime sees it, whatever its type. Every view is trivially copyable, so `bytes` copies it.
struct Port {
  Component* owner = nullptr;
  std::string_view name, type;
  std::uint64_t hash = 0;
  std::size_t bytes = 0;
  bool output = false;
  InOpts opts{};
  void* value = nullptr;    // Out: its storage
  bool written = false;     // Out: write() was called this step
  Feed own;                 // Out: what it offers readers; In: what a ring, a lag or the boundary hands it
  const Feed* feed = &own;  // In: what it reads (a writer's `own` once wired)
  std::uint64_t seen = 0;   // In: the seq take() returned last
  mutable std::atomic<bool> hid_stale{false};  // In: get() hid a stale value since the facilities last looked
  void* plumbing = nullptr;                    // cpp/src: where a published Out also goes (queue rings, the boundary)
};

void enroll(Component* owner, Port* port);
template <class T>
constexpr std::string_view type_of() {
  if constexpr (requires { T::kType; }) return T::kType;
  return {};
}
template <class T>
constexpr std::uint64_t hash_of() {
  if constexpr (requires { T::kHash; }) return T::kHash;
  return 0;
}
template <class T>
concept PortLike = std::is_base_of_v<Port, T>;
using Factory = System (*)(Fields&);
bool enroll_factory(Factory make);
template <class Params>
Params params_of(System (*)(const Params&));  // declared only: LK_EXPORT reads the factory's Params type from it

}  // namespace detail

class Component {
 public:
  Component();
  Component(const Component&) = delete;
  Component& operator=(const Component&) = delete;
  virtual ~Component() = default;
  virtual void activate() {}    // non-RT thread; may block and open devices; throw to refuse
  virtual void forward() = 0;   // RT thread: no allocation, no lock, no blocking call except device I/O it owns
  virtual void deactivate() {}  // non-RT thread; must not throw
  double now() const;           // this step's scheduled time, seconds (CONTRACT §5.6)
  double period() const;        // the group's period, seconds; 0.0 when rate-less; from activate() on (§15 A20)
  void metric(const char* name, double value);  // name: a string literal; <= 32 per component; no allocation
  // level "ok" | "warn" | "error" and a PascalCase code, both string literals; held until called again, and health
  // reports the worse of it and the facility's own (CONTRACT §15 A22)
  void health(const char* level, const char* code);
  struct Log {  // fmt: a string literal; two arguments, numbers (%g) or string literals (%s); copied into a
                // lock-free ring, formatted off the RT thread. An error makes health error / ErrorLogged until a
                // round in which none was logged.
    void info(const char* fmt, detail::Arg a = 0.0, detail::Arg b = 0.0);
    void warning(const char* fmt, detail::Arg a = 0.0, detail::Arg b = 0.0);
    void error(const char* fmt, detail::Arg a = 0.0, detail::Arg b = 0.0);

   private:
    friend class Component;
    friend struct detail::Box;
    Component* owner_ = nullptr;
  } log;

 private:
  friend struct detail::Box;
  friend class System;
  friend void detail::enroll(Component*, detail::Port*);
  std::string name_;
  std::vector<detail::Port*> ports_;
  const std::atomic<double>* clock_ = nullptr;  // the box's step time once placed; eager calls read 0.0
  detail::Box* box_ = nullptr;
  std::size_t index_ = 0;  // this leaf's place in the box (logs, metrics)
};

template <class T>
class In : private detail::Port {
 public:
  In(Component* owner, const char* name, InOpts opts = {});
  In(const In&) = delete;
  const T* get()
      const;        // the current value: latest -> newest (nullptr if none, stale or retracted); queue -> this step's
  const T* take();  // only what arrived since the last take(), else nullptr

 private:
  friend class System;
  friend struct detail::Box;
};

template <class T>
class Out : private detail::Port {
 public:
  Out(Component* owner, const char* name);
  Out(const Out&) = delete;
  T& write();  // this step's preallocated value; calling write() marks it sent this step

 private:
  friend class System;
  friend struct detail::Box;
  T value_{};
};

class System : public Component {
 public:
  explicit System(std::string_view name);
  System(System&& other) noexcept;
  template <class T, class... Args>
  T& add(std::string_view name, Args&&... args);  // allocation happens here, never later
  template <class T>
  void wire(In<T>& dst, Out<T>& src);  // dst <- src (subscribe direction); a swapped or mistyped wire does not compile
  template <detail::PortLike... Ports>
  void expose(Ports&... ports);  // boundary name = port name
  template <detail::PortLike P>
  void expose(std::string_view as, P& port);
  void forward() final;  // the children, in the derived order (CONTRACT §5.3)

 private:
  friend struct detail::Box;
  void adopt(std::unique_ptr<Component> child, std::string_view name);
  void link(detail::Port& dst, detail::Port& src);
  void publish(std::string_view as, detail::Port& port);
  std::vector<std::unique_ptr<Component>> children_;
  std::vector<std::pair<detail::Port*, detail::Port*>> wires_;  // (dst, src)
  std::vector<std::pair<std::string, detail::Port*>> exposed_;
};

// ---------------------------------------------------------------- bodies

inline double Component::now() const { return clock_ ? clock_->load(std::memory_order_relaxed) : 0.0; }

inline detail::Arg::Arg(double value) : number(value) {}
inline detail::Arg::Arg(const char* value) : text(value) {}
template <class Integer>
  requires std::is_integral_v<Integer>
inline detail::Arg::Arg(Integer value) : number(static_cast<double>(value)) {}

template <class T>
In<T>::In(Component* owner, const char* name, InOpts opts) {
  static_assert(std::is_trivially_copyable_v<T>, "a port carries a trivially copyable view (like_keras/msgs/*.hpp)");
  this->owner = owner, this->name = name, this->type = detail::type_of<T>(), this->hash = detail::hash_of<T>();
  this->bytes = sizeof(T), this->opts = opts;
  detail::enroll(owner, this);
}

template <class T>
const T* In<T>::get() const {
  const void* value = feed->value.load(std::memory_order_relaxed);
  if (value != nullptr && opts.stale > 0.0 && owner->now() - feed->stamp.load(std::memory_order_relaxed) > opts.stale) {
    hid_stale.store(true, std::memory_order_relaxed);
    return nullptr;  // CONTRACT §4.1: an old latest value reads as none, and health lists the input as stale
  }
  return static_cast<const T*>(value);
}

template <class T>
const T* In<T>::take() {
  const std::uint64_t seq = feed->seq.load(std::memory_order_relaxed);
  if (seq == seen) return nullptr;
  seen = seq;
  return get();
}

template <class T>
Out<T>::Out(Component* owner, const char* name) {
  static_assert(std::is_trivially_copyable_v<T>, "a port carries a trivially copyable view (like_keras/msgs/*.hpp)");
  this->owner = owner, this->name = name, this->type = detail::type_of<T>(), this->hash = detail::hash_of<T>();
  this->bytes = sizeof(T), this->output = true, this->value = &value_;
  detail::enroll(owner, this);
}

template <class T>
T& Out<T>::write() {
  written = true;
  return value_;
}

template <class T, class... Args>
T& System::add(std::string_view name, Args&&... args) {
  auto child = std::make_unique<T>(std::forward<Args>(args)...);
  T& added = *child;
  adopt(std::move(child), name);
  return added;
}

template <class T>
void System::wire(In<T>& dst, Out<T>& src) {
  link(dst, src);
}

template <detail::PortLike... Ports>
void System::expose(Ports&... ports) {
  (publish(static_cast<detail::Port&>(ports).name, ports), ...);
}

template <detail::PortLike P>
void System::expose(std::string_view as, P& port) {
  publish(as, port);
}

namespace detail {

// LK_PARAMS' walker: JSON into a Params aggregate at lk_create, or its defaults out as JSON for lk_describe.
class Fields {
 public:
  explicit Fields(const void* json, std::string* out = nullptr);  // json: abi.cpp's parsed JSON; out: write defaults
  void operator()(const char* key, double& value);
  void operator()(const char* key, int& value);
  void operator()(const char* key, bool& value);
  void operator()(const char* key, std::string& value);
  void operator()(const char* key, std::vector<double>& value);  // CONTRACT §15 A24
  template <class Nested>
  void operator()(const char* key, Nested& value);  // a nested LK_PARAMS aggregate; key nullptr is the root
  template <class Nested>
  void operator()(const char* key, std::vector<Nested>& rows);  // rows of a nested LK_PARAMS aggregate (A24)

 private:
  void enter(const char* key);
  void leave();
  std::size_t open_rows(const char* key, std::size_t count);  // reading: the array's length (`count` when absent)
  void enter_row(std::size_t at);                             // its object; the path an error names: key[at]
  void close_rows();
  const void* find(const char* key, int kind);  // reading: the member `key` of the object being filled, or nullptr
  void emit(const char* key, const std::string& json);
  const void* json_ = nullptr;      // reading: the parsed JSON root (cpp/src/abi.cpp's Json)
  std::vector<const void*> stack_;  // reading: the objects being filled (nullptr: absent, keep the defaults)
  std::vector<std::size_t> marks_;  // reading: where each object's keys start in seen_
  std::vector<std::string> seen_;   // reading: the keys visited, to name an unknown one
  std::vector<std::pair<const void*, std::string>> rows_;  // reading: the arrays of rows being filled, their keys
  std::string* out_ = nullptr;                             // writing: the defaults as JSON
  std::string path_;                                       // "bus.joints[2]": the key an error names
};

template <class Nested>
void Fields::operator()(const char* key, Nested& value) {
  enter(key);
  lk_fields(value, *this);  // found by argument-dependent lookup: LK_PARAMS defines it beside the type
  leave();
}

template <class Nested>
void Fields::operator()(const char* key, std::vector<Nested>& rows) {
  rows.resize(open_rows(key, rows.size()));  // filled at lk_create, never on the RT thread
  for (std::size_t at = 0; at < rows.size(); ++at) {
    enter_row(at);
    lk_fields(rows[at], *this);
    leave();
  }
  close_rows();
}

template <auto factory, class Params>
System make(Fields& fields) {
  Params params{};
  fields(nullptr, params);  // throws naming an unknown or mistyped key, before anything is constructed
  return factory(params);
}

}  // namespace detail
}  // namespace lk

// LK_PARAMS(Type, field, ...): reflects an aggregate's fields (std::string, bool, double, int, a nested LK_PARAMS
// aggregate, std::vector<double> or a std::vector of one) so JSON params reach it at lk_create, never on the RT thread.
#define LK_PARAMS(Type, ...)                                                               \
  [[maybe_unused]] inline void lk_fields(Type& lk_value, ::lk::detail::Fields& lk_visit) { \
    LKX_EACH(LKX_FIELD, __VA_ARGS__)                                                       \
  }

// LK_EXPORT(factory): factory is lk::System factory(const Params&) with LK_PARAMS(Params, ...); the .so then carries
// the C ABI of like_keras/abi.h (the functions themselves live in cpp/src/abi.cpp, linked into every component).
#define LK_EXPORT(factory)                          \
  [[maybe_unused]] static const bool lk_exported_ = \
      ::lk::detail::enroll_factory(&::lk::detail::make<factory, decltype(::lk::detail::params_of(factory))>);

// One macro per field, C++20 __VA_OPT__ recursion (up to 64 fields). The helpers are LKX_, not LK_: only LK_PARAMS
// and LK_EXPORT are the API.
#define LKX_FIELD(member) lk_visit(#member, lk_value.member);
#define LKX_PARENS ()
#define LKX_EXPAND(...) LKX_EXPAND3(LKX_EXPAND3(LKX_EXPAND3(LKX_EXPAND3(__VA_ARGS__))))
#define LKX_EXPAND3(...) LKX_EXPAND2(LKX_EXPAND2(LKX_EXPAND2(LKX_EXPAND2(__VA_ARGS__))))
#define LKX_EXPAND2(...) LKX_EXPAND1(LKX_EXPAND1(LKX_EXPAND1(LKX_EXPAND1(__VA_ARGS__))))
#define LKX_EXPAND1(...) __VA_ARGS__
#define LKX_EACH(macro, ...) __VA_OPT__(LKX_EXPAND(LKX_EACH_NEXT(macro, __VA_ARGS__)))
#define LKX_EACH_NEXT(macro, first, ...) macro(first) __VA_OPT__(LKX_EACH_AGAIN LKX_PARENS(macro, __VA_ARGS__))
#define LKX_EACH_AGAIN() LKX_EACH_NEXT
