// The C ABI of every component .so (CONTRACT §6.3), the wire between Arrow C Data and the C++ views (§7.1, §7.4),
// and LK_PARAMS' JSON. No Arrow, JSON or TOML library: the two Arrow structs are declared in abi.h, and params
// arrive as JSON of which this parses the subset LK_PARAMS needs.
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <like_keras/msgs/control.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <like_keras/msgs/std.hpp>
#include <like_keras/msgs/trajectory.hpp>
#include <optional>
#include <span>
#include <stdexcept>

#include "box.hpp"

namespace lk::detail {
namespace {

// ---------------------------------------------------------------- the v0 types, field by field in dataclass order

// One function per type walks its wire children in robot_msgs' field order (the schema-hash order, CONTRACT §7.1),
// with one visitor to decode, one to encode and one to pin strings. Fields a view leaves out (Header.meta, a plan's
// channels and limits) are skipped on the way in and sent empty on the way out.
template <class V>
void fields(rm::Header& m, V& v) {
  v.scalar(m.stamp), v.text(m.frame_id), v.empty_map();
}
template <class V>
void fields(rm::JointState& m, V& v) {
  v.names(m.name, m.size), v.numbers(m.position, m.size), v.numbers(m.velocity, m.size);
  v.numbers(m.effort, m.size), v.nest(m.header);
}
template <class V>
void fields(rm::ServoState& m, V& v) {
  v.names(m.name, m.size), v.numbers(m.position, m.size), v.numbers(m.velocity, m.size), v.numbers(m.effort, m.size);
  v.numbers(m.temperature, m.size), v.numbers(m.voltage, m.size), v.numbers(m.fault, m.size), v.scalar(m.seq);
  v.nest(m.header);
}
template <class V>
void fields(rm::ServoCommand& m, V& v) {
  v.names(m.name, m.size), v.numbers(m.position, m.size), v.numbers(m.velocity, m.size);
  v.numbers(m.feedforward_effort, m.size), v.numbers(m.stiffness, m.size), v.numbers(m.damping, m.size);
  v.numbers(m.max_effort, m.size), v.scalar(m.state_stamp), v.scalar(m.state_seq), v.nest(m.header);
}
template <class V>
void fields(rm::JointTrajectory& m, V& v) {
  v.span(m.t), v.skip(), v.nest(m.header), v.names(m.joint_names, m.joints), v.span(m.positions), v.span(m.velocities);
  v.span(m.accelerations), v.span(m.effort), v.skip();
}
template <class V>
void fields(rm::LogRecord& m, V& v) {
  v.nest(m.header), v.text(m.path), v.scalar(m.tick), v.scalar(m.level), v.text(m.message);
}
template <class V>
void fields(rm::DiagnosticRecord& m, V& v) {
  v.nest(m.header), v.text(m.path), v.text(m.level), v.text(m.code), v.text(m.message), v.text(m.mode);
  v.scalar(m.expected_period_s), v.scalar(m.uptime_s), v.names(m.stale_inputs, m.size);
}
template <class V>
void fields(rm::Metrics& m, V& v) {
  v.nest(m.header), v.text(m.path), v.names(m.names, m.size), v.numbers(m.values, m.size);
}

// ---------------------------------------------------------------- decode: views into a received array

struct Reader {
  const ArrowArray* node;
  std::int64_t row;  // the row of node's children this message is (node->offset + the message's row)
  std::int64_t next = 0;
  bool ok = true;

  const ArrowArray* child() {
    if (next < node->n_children) return node->children[next++];
    ok = false;
    return nullptr;
  }
  static bool valid(const ArrowArray& array, std::int64_t at) {
    const auto* bits = static_cast<const std::uint8_t*>(array.buffers[0]);
    return array.null_count == 0 || bits == nullptr || (bits[at / 8] >> (at % 8) & 1);
  }
  static std::pair<std::int64_t, std::int64_t> range(const ArrowArray& array, std::int64_t at) {
    const auto* offsets = static_cast<const std::int32_t*>(array.buffers[1]);
    return {offsets[at], offsets[at + 1]};
  }
  static std::string_view text_at(const ArrowArray& array, std::int64_t at) {
    const auto [begin, end] = range(array, at);
    return {static_cast<const char*>(array.buffers[2]) + begin, static_cast<std::size_t>(end - begin)};
  }
  template <class N, std::size_t C>
  void copy_items(const ArrowArray& list, std::array<N, C>& out) {
    const auto [begin, end] = range(list, list.offset + row);
    const ArrowArray& items = *list.children[0];
    if (end - begin > static_cast<std::int64_t>(C)) {
      ok = false;  // more joints than lk::kMaxJoints: refused, never cut
      return;
    }
    if (end > begin)
      std::memcpy(out.data(), static_cast<const N*>(items.buffers[1]) + items.offset + begin,
                  (end - begin) * sizeof(N));
  }
  template <class N, std::size_t C>
  void numbers(std::array<N, C>& out, std::size_t&) {
    if (const ArrowArray* list = child()) copy_items(*list, out);
  }
  template <class N, std::size_t C>
  void numbers(std::optional<std::array<N, C>>& out, std::size_t&) {
    const ArrowArray* list = child();
    if (list == nullptr || !valid(*list, list->offset + row)) return out.reset();
    copy_items(*list, out.emplace());
  }
  template <std::size_t C>
  void names(std::array<std::string_view, C>& out, std::size_t& size) {
    const ArrowArray* list = child();
    if (list == nullptr) return;
    const auto [begin, end] = range(*list, list->offset + row);
    if (end - begin > static_cast<std::int64_t>(C)) return void(ok = false);
    const ArrowArray& items = *list->children[0];
    for (auto at = begin; at < end; ++at) out[at - begin] = text_at(items, items.offset + at);
    size = static_cast<std::size_t>(end - begin);
  }
  template <class N>
  void scalar(N& out) {
    if (const ArrowArray* array = child()) out = static_cast<const N*>(array->buffers[1])[array->offset + row];
  }
  template <class N>
  void scalar(std::optional<N>& out) {
    const ArrowArray* array = child();
    if (array == nullptr || !valid(*array, array->offset + row)) return out.reset();
    out = static_cast<const N*>(array->buffers[1])[array->offset + row];
  }
  void text(std::string_view& out) {
    if (const ArrowArray* array = child()) out = text_at(*array, array->offset + row);
  }
  std::span<const double> items_of(const ArrowArray& list) {
    const auto [begin, end] = range(list, list.offset + row);
    const ArrowArray& items = *list.children[0];
    return {static_cast<const double*>(items.buffers[1]) + items.offset + begin, static_cast<std::size_t>(end - begin)};
  }
  void span(std::span<const double>& out) {
    if (const ArrowArray* list = child()) out = items_of(*list);
  }
  void span(std::optional<std::span<const double>>& out) {
    const ArrowArray* list = child();
    if (list == nullptr || !valid(*list, list->offset + row)) return out.reset();
    out = items_of(*list);
  }
  template <class M>
  void nest(M& out) {
    const ArrowArray* array = child();
    if (array == nullptr) return;
    Reader inner{array, array->offset + row};
    fields(out, inner);
    ok = ok && inner.ok;
  }
  void skip() { child(); }
  void empty_map() { child(); }
};

template <class T>
bool decode(const ArrowArray& array, void* view) {
  if (array.length < 1) return false;
  T& message = *static_cast<T*>(view);
  message = T{};
  Reader reader{&array, array.offset};
  fields(message, reader);
  return reader.ok && reader.next == array.n_children;
}

// ---------------------------------------------------------------- pin: an exposed Out's strings, on the RT thread

struct Pinner {
  Arena& arena;
  bool ok = true;
  void text(std::string_view& text) {
    if (arena.used + text.size() > arena.size) return void(ok = false);
    if (!text.empty()) std::memcpy(arena.data + arena.used, text.data(), text.size());
    text = {arena.data + arena.used, text.size()};
    arena.used += text.size();
  }
  template <std::size_t C>
  void names(std::array<std::string_view, C>& names, std::size_t size) {
    for (std::size_t at = 0; at < std::min(size, C); ++at) text(names[at]);
  }
  template <class M>
  void nest(M& message) {
    fields(message, *this);
  }
  template <class... Any>
  void numbers(Any&&...) {}
  template <class... Any>
  void scalar(Any&&...) {}
  template <class... Any>
  void span(Any&&...) {}
  void skip() {}
  void empty_map() {}
};

template <class T>
bool pin(void* view, Arena& arena) {
  Pinner pinner{arena};
  fields(*static_cast<T*>(view), pinner);
  return pinner.ok;
}

// ---------------------------------------------------------------- encode: a fresh array a host thread owns

struct Built {  // every node and buffer of one encoded message; the root's release deletes it
  std::deque<ArrowArray> nodes;
  std::deque<std::vector<const void*>> buffers;
  std::deque<std::vector<ArrowArray*>> children;
  std::deque<std::string> bytes;
};

void release_built(ArrowArray* array) {
  delete static_cast<Built*>(array->private_data);
  array->release = nullptr;
}
void release_child(ArrowArray* array) { array->release = nullptr; }  // children go with their root

struct Writer {
  Built& built;
  std::vector<ArrowArray*> kids{};

  const void* keep(const void* data, std::size_t size) {
    static const char kNothing[8] = {};
    return size == 0 ? kNothing : built.bytes.emplace_back(static_cast<const char*>(data), size).data();
  }
  const void* ends(std::size_t end) {
    const std::int32_t pair[2] = {0, static_cast<std::int32_t>(end)};
    return keep(pair, sizeof pair);
  }
  ArrowArray* node(std::int64_t length, bool null, std::vector<const void*> buffers,
                   std::vector<ArrowArray*> children = {}) {
    if (null) buffers[0] = keep("\0", 1);  // the validity bitmap: row 0 is null
    auto& kept_buffers = built.buffers.emplace_back(std::move(buffers));
    auto& kept_children = built.children.emplace_back(std::move(children));
    return &built.nodes.emplace_back(ArrowArray{length, null ? 1 : 0, 0, static_cast<std::int64_t>(kept_buffers.size()),
                                                static_cast<std::int64_t>(kept_children.size()), kept_buffers.data(),
                                                kept_children.empty() ? nullptr : kept_children.data(), nullptr,
                                                release_child, nullptr});
  }
  template <class N>
  void list(const N* data, std::size_t size, bool null) {
    ArrowArray* items = node(static_cast<std::int64_t>(size), false, {nullptr, keep(data, size * sizeof(N))});
    kids.push_back(node(1, null, {nullptr, ends(size)}, {items}));
  }
  template <class N, std::size_t C>
  void numbers(const std::array<N, C>& in, std::size_t size) {
    list(in.data(), size, false);
  }
  template <class N, std::size_t C>
  void numbers(const std::optional<std::array<N, C>>& in, std::size_t size) {
    in ? list(in->data(), size, false) : list<N>(nullptr, 0, true);
  }
  template <std::size_t C>
  void names(const std::array<std::string_view, C>& in, std::size_t size) {
    std::vector<std::int32_t> offsets{0};
    std::string chars;
    for (std::size_t at = 0; at < size; ++at)
      chars += in[at], offsets.push_back(static_cast<std::int32_t>(chars.size()));
    ArrowArray* items = node(static_cast<std::int64_t>(size), false,
                             {nullptr, keep(offsets.data(), offsets.size() * 4), keep(chars.data(), chars.size())});
    kids.push_back(node(1, false, {nullptr, ends(size)}, {items}));
  }
  template <class N>
  void scalar(const N& value) {
    kids.push_back(node(1, false, {nullptr, keep(&value, sizeof value)}));
  }
  template <class N>
  void scalar(const std::optional<N>& value) {
    const N held = value.value_or(N{});
    kids.push_back(node(1, !value, {nullptr, keep(&held, sizeof held)}));
  }
  void text(std::string_view text) {
    kids.push_back(node(1, false, {nullptr, ends(text.size()), keep(text.data(), text.size())}));
  }
  template <class M>
  void nest(M& message) {
    Writer inner{built};
    fields(message, inner);
    kids.push_back(node(1, false, {nullptr}, std::move(inner.kids)));
  }
  void empty_map() {  // map<utf8, utf8> with no entries (Header.meta, which the views leave out)
    const std::int32_t zero = 0;
    ArrowArray* key = node(0, false, {nullptr, keep(&zero, 4), nullptr});
    ArrowArray* value = node(0, false, {nullptr, keep(&zero, 4), nullptr});
    ArrowArray* entries = node(0, false, {nullptr}, {key, value});
    kids.push_back(node(1, false, {nullptr, ends(0)}, {entries}));
  }
};

template <class T>
void encode(const void* view, ArrowArray* out) {
  auto* built = new Built;
  Writer writer{*built};
  fields(*const_cast<T*>(static_cast<const T*>(view)), writer);
  *out = *writer.node(1, false, {nullptr}, std::move(writer.kids));
  out->release = release_built, out->private_data = built;
}

template <class T>
constexpr Wire wire() {
  if constexpr (std::is_same_v<T, rm::JointTrajectory>)
    return {T::kHash, decode<T>, nullptr, pin<T>};  // sample-major: In only in v0 (CONTRACT §7.4)
  else
    return {T::kHash, decode<T>, encode<T>, pin<T>};
}

constexpr Wire kWires[] = {wire<rm::Header>(),       wire<rm::LogRecord>(),      wire<rm::DiagnosticRecord>(),
                           wire<rm::Metrics>(),      wire<rm::JointState>(),     wire<rm::ServoState>(),
                           wire<rm::ServoCommand>(), wire<rm::JointTrajectory>()};

// ---------------------------------------------------------------- JSON: the subset LK_PARAMS reads and writes

struct Json {
  enum Kind { kNull, kBool, kNumber, kString, kObject, kArray } kind = kNull;
  bool boolean = false, integral = false;
  double number = 0.0;
  std::string text;
  std::vector<std::pair<std::string, Json>> members;  // an object's (or an array's, keys empty)
};
const char* const kKinds[] = {"null", "a bool", "a number", "a string", "an object", "an array"};

struct Parser {
  const char* at;
  [[noreturn]] void fail(const char* what) {
    throw std::invalid_argument(std::string("params JSON: ") + what + " at '" +
                                std::string(at, std::strlen(at) > 20 ? 20 : std::strlen(at)) + "'");
  }
  void space() { at += std::strspn(at, " \t\r\n"); }
  bool eat(char c) {
    space();
    return *at == c ? (++at, true) : false;
  }
  void utf8(std::string& out, std::uint32_t code) {
    if (code < 0x80) return void(out += static_cast<char>(code));
    if (code < 0x800)
      return void((out += static_cast<char>(0xC0 | code >> 6), out += static_cast<char>(0x80 | (code & 0x3F))));
    if (code < 0x10000)
      out += static_cast<char>(0xE0 | code >> 12);
    else
      out += static_cast<char>(0xF0 | code >> 18), out += static_cast<char>(0x80 | (code >> 12 & 0x3F));
    out += static_cast<char>(0x80 | (code >> 6 & 0x3F)), out += static_cast<char>(0x80 | (code & 0x3F));
  }
  std::uint32_t hex4() {
    std::uint32_t code = 0;
    if (std::from_chars(at, at + 4, code, 16).ptr != at + 4) fail("a bad \\u escape");
    at += 4;
    return code;
  }
  std::string string() {
    if (!eat('"')) fail("expected a string");
    std::string out;
    for (; *at != '"'; ++at) {
      if (*at == '\0') fail("an unterminated string");
      if (*at != '\\') {
        out += *at;
        continue;
      }
      const char escaped = *++at;
      if (escaped != 'u') {
        static constexpr char kNames[] = "\"\\/bfnrt", kMeant[] = "\"\\/\b\f\n\r\t";
        const char* known = escaped == '\0' ? nullptr : std::strchr(kNames, escaped);
        if (known == nullptr) fail("a bad escape");
        out += kMeant[known - kNames];
        continue;
      }
      ++at;
      std::uint32_t code = hex4();  // json.dumps writes non-ASCII as \uXXXX, a pair above the BMP
      if (code >= 0xD800 && code < 0xDC00 && at[0] == '\\' && at[1] == 'u')
        at += 2, code = 0x10000 + ((code - 0xD800) << 10) + (hex4() - 0xDC00);
      utf8(out, code);
      --at;
    }
    ++at;
    return out;
  }
  Json value() {
    space();
    Json out;
    if (*at == '{' || *at == '[') {
      const char close = *at++ == '{' ? '}' : ']';
      out.kind = close == '}' ? Json::kObject : Json::kArray;
      if (eat(close)) return out;
      do {
        std::string key;
        if (close == '}') {
          space();
          key = string();
          if (!eat(':')) fail("expected ':'");
        }
        out.members.emplace_back(std::move(key), value());
      } while (eat(','));
      if (!eat(close)) fail("expected ',' or the closing bracket");
    } else if (*at == '"') {
      out.kind = Json::kString, out.text = string();
    } else if (std::strncmp(at, "true", 4) == 0 || std::strncmp(at, "false", 5) == 0) {
      out.kind = Json::kBool, out.boolean = *at == 't', at += out.boolean ? 4 : 5;
    } else if (std::strncmp(at, "null", 4) == 0) {
      at += 4;
    } else {
      char* end = nullptr;
      out.kind = Json::kNumber, out.number = std::strtod(at, &end);
      if (end == at) fail("unexpected character");
      out.integral =
          std::isfinite(out.number) && std::string_view(at, end - at).find_first_of(".eE") == std::string_view::npos;
      at = end;
    }
    return out;
  }
};

std::string quoted(std::string_view text) {
  std::string out = "\"";
  for (const char c : text) {
    if (c == '"' || c == '\\')
      out += '\\', out += c;
    else if (static_cast<unsigned char>(c) < 0x20) {
      char escaped[8];
      std::snprintf(escaped, sizeof escaped, "\\u%04x", c);
      out += escaped;
    } else
      out += c;
  }
  return out + '"';
}

std::string number(double value) {
  // WHY these spellings (CONTRACT §15 A33, consumer-016): JSON has none for NaN and infinity, and std::to_chars writes
  // nan and inf, which no JSON reader takes; Python's json and strtod (lk_create) both read NaN, Infinity, -Infinity
  if (std::isnan(value)) return "NaN";
  if (std::isinf(value)) return value > 0 ? "Infinity" : "-Infinity";
  char text[32];
  char* end = std::to_chars(text, text + sizeof text, value).ptr;  // the shortest exact form
  // WHY a point on an integral double (CONTRACT §15 A41): Python reads 10 as an int, so a double default of 10.0 made
  // the box's datasheet differ in JSON kind from its Python twin's 10.0
  const bool integral = std::find_if(text, end, [](char c) { return c == '.' || c == 'e'; }) == end;
  return std::string(text, end) + (integral ? ".0" : "");
}

const Json* member(const Json& object, std::string_view key) {
  for (const auto& [name, value] : object.members)
    if (name == key) return &value;
  return nullptr;
}

Factory exported = nullptr;

void copy_error(char* err, std::size_t err_len, const std::string& what) {
  if (err != nullptr && err_len > 0) std::snprintf(err, err_len, "%s", what.c_str());
}

}  // namespace

// ---------------------------------------------------------------- shared with rt.cpp and the macros

const Wire* find_wire(std::uint64_t hash) {
  for (const Wire& candidate : kWires)
    if (candidate.hash == hash) return &candidate;
  return nullptr;
}

void release(ArrowArray& array) {
  if (array.release == nullptr) return;
  array.release(&array);
  array.release = nullptr;
}

bool enroll_factory(Factory make) {
  exported = make;
  return true;
}

Fields::Fields(const void* json, std::string* out) : json_(json), out_(out) {}

void Fields::enter(const char* key) {
  if (out_ != nullptr) {
    if (!out_->empty() && out_->back() != '{' && out_->back() != '[') *out_ += ", ";
    *out_ += key ? quoted(key) + ": {" : "{";
    return;
  }
  const Json* object = static_cast<const Json*>(json_);
  if (key != nullptr) {
    object = static_cast<const Json*>(find(key, Json::kObject));
    path_ += path_.empty() ? key : "." + std::string(key);
  } else if (object != nullptr && object->kind != Json::kObject) {
    throw std::invalid_argument(std::string("params: expected an object, got ") + kKinds[object->kind]);
  }
  stack_.push_back(object), marks_.push_back(seen_.size());
}

void Fields::leave() {
  if (out_ != nullptr) return void(*out_ += "}");
  const auto* object = static_cast<const Json*>(stack_.back());
  const std::size_t mark = marks_.back();
  for (const auto& [name, value] : object != nullptr ? object->members : std::vector<std::pair<std::string, Json>>{}) {
    if (std::find(seen_.begin() + static_cast<std::ptrdiff_t>(mark), seen_.end(), name) != seen_.end()) continue;
    std::string known;
    for (auto at = seen_.begin() + static_cast<std::ptrdiff_t>(mark); at != seen_.end(); ++at)
      known += (known.empty() ? "" : ", ") + *at;
    // WHY the full path to the key: in a table the row is what the author must find (rows[2].valu, CONTRACT §15 A24)
    throw std::invalid_argument("params." + (path_.empty() ? "" : path_ + ".") + name +
                                ": unknown key (known: " + known + ")");
  }
  seen_.resize(mark), stack_.pop_back(), marks_.pop_back();
  const auto dot = path_.rfind('.');
  path_.resize(dot == std::string::npos ? 0 : dot);
}

std::size_t Fields::open_rows(const char* key, std::size_t count) {
  if (out_ != nullptr) {
    if (out_->back() != '{') *out_ += ", ";
    *out_ += quoted(key) + ": [";
    return count;
  }
  const auto* array = static_cast<const Json*>(find(key, Json::kArray));
  rows_.emplace_back(array, key);
  return array != nullptr ? array->members.size() : count;  // absent: the default rows stay
}

void Fields::enter_row(std::size_t at) {
  if (out_ != nullptr) return void(*out_ += out_->back() == '[' ? "{" : ", {");
  const auto* array = static_cast<const Json*>(rows_.back().first);
  const Json* row = array != nullptr ? &array->members[at].second : nullptr;
  path_ += (path_.empty() ? "" : ".") + rows_.back().second + "[" + std::to_string(at) + "]";
  if (row != nullptr && row->kind != Json::kObject)
    throw std::invalid_argument("params." + path_ + ": expected an object, got " + kKinds[row->kind]);
  stack_.push_back(row), marks_.push_back(seen_.size());
}

void Fields::close_rows() {
  if (out_ != nullptr) return void(*out_ += "]");
  rows_.pop_back();
}

const void* Fields::find(const char* key, int kind) {
  seen_.emplace_back(key);
  const auto* object = static_cast<const Json*>(stack_.back());
  const Json* found = object ? member(*object, key) : nullptr;
  if (found != nullptr && found->kind != kind)
    throw std::invalid_argument("params." + (path_.empty() ? "" : path_ + ".") + key + ": expected " + kKinds[kind] +
                                ", got " + kKinds[found->kind]);
  return found;  // absent: the default stays
}

void Fields::emit(const char* key, const std::string& json) {
  if (out_->back() != '{') *out_ += ", ";
  *out_ += quoted(key) + ": " + json;
}

void Fields::operator()(const char* key, double& value) {
  if (out_ != nullptr) return emit(key, number(value));
  if (const auto* found = static_cast<const Json*>(find(key, Json::kNumber))) value = found->number;
}

void Fields::operator()(const char* key, int& value) {
  if (out_ != nullptr) return emit(key, std::to_string(value));
  const auto* found = static_cast<const Json*>(find(key, Json::kNumber));
  if (found == nullptr) return;
  if (!found->integral || found->number < -2147483648.0 || found->number > 2147483647.0)
    throw std::invalid_argument("params." + (path_.empty() ? "" : path_ + ".") + key + ": expected an int, got " +
                                number(found->number));
  value = static_cast<int>(found->number);
}

void Fields::operator()(const char* key, bool& value) {
  if (out_ != nullptr) return emit(key, value ? "true" : "false");
  if (const auto* found = static_cast<const Json*>(find(key, Json::kBool))) value = found->boolean;
}

void Fields::operator()(const char* key, std::string& value) {
  if (out_ != nullptr) return emit(key, quoted(value));
  if (const auto* found = static_cast<const Json*>(find(key, Json::kString))) value = found->text;
}

void Fields::operator()(const char* key, std::vector<double>& values) {
  if (out_ != nullptr) {
    std::string json;
    for (const double value : values) json += (json.empty() ? "" : ", ") + number(value);
    return emit(key, "[" + json + "]");
  }
  const auto* found = static_cast<const Json*>(find(key, Json::kArray));
  if (found == nullptr) return;
  values.clear();
  for (const auto& [_, item] : found->members) {
    if (item.kind != Json::kNumber)
      throw std::invalid_argument("params." + (path_.empty() ? "" : path_ + ".") + key + "[" +
                                  std::to_string(values.size()) + "]: expected a number, got " + kKinds[item.kind]);
    values.push_back(item.number);
  }
}

std::string Box::sheet() const {  // lk_describe(): the exposed ports, in expose() order (the ports' indices)
  std::string ports;
  for (const auto& [name, port] : system.exposed_) {
    char hash[17];
    std::snprintf(hash, sizeof hash, "%016llx", static_cast<unsigned long long>(port->hash));
    const bool in = !port->output;
    ports += (ports.empty() ? "" : ", ") + quoted(name) + ": {\"dir\": \"" + (in ? "in" : "out") +
             "\", \"type\": " + quoted(port->type) + ", \"hash\": \"" + hash +
             "\", \"queue\": " + (in && port->opts.queue > 0 ? std::to_string(port->opts.queue) : "null") +
             ", \"stale\": " + (in && port->opts.stale > 0.0 ? number(port->opts.stale) : "null") +
             ", \"lagged\": " + (in && port->opts.lagged ? "true" : "false") +
             (in && port->opts.optional ? ", \"optional\": true}" : "}");  // CONTRACT §15 A23
  }
  return "{\"like_keras\": \"0.1.0\", \"abi\": " + std::to_string(LK_ABI_VERSION) +
         ", \"system\": " + quoted(system.name_) + ", \"ports\": {" + ports +
         "}, \"facilities\": [\"log\", \"health\", \"metrics\"]";
}

}  // namespace lk::detail

// ---------------------------------------------------------------- the C ABI (CONTRACT §6.3)

namespace {
lk_box* build(const char* params_json, std::string* defaults) {
  using lk::detail::Json;
  if (lk::detail::exported == nullptr) throw std::invalid_argument("this .so has no LK_EXPORT(factory)");
  Json root;
  if (params_json != nullptr) {
    lk::detail::Parser parser{params_json};
    root = parser.value();
    parser.space();
    if (*parser.at != '\0') parser.fail("trailing characters");
  } else {
    root.kind = Json::kObject;
  }
  lk::detail::Fields fields(defaults ? nullptr : &root, defaults);
  return new lk_box(lk::detail::exported(fields));
}
}  // namespace

#define LKX_API extern "C" __attribute__((visibility("default")))

LKX_API uint32_t lk_abi_version(void) { return LK_ABI_VERSION; }

LKX_API const char* lk_describe(void) {
  // WHY build the System from its default Params: ports exist only once the components are constructed. That is
  // construction without activate, so no device opens and no thread starts (CONTRACT §4.5 forbids I/O here).
  static const std::string sheet = [] {
    try {
      std::string params;
      const std::unique_ptr<lk_box> box(build(nullptr, &params));
      return box->sheet() + ", \"params\": " + params + "}";
    } catch (const std::exception& error) {
      return "{\"like_keras\": \"0.1.0\", \"abi\": 1, \"ports\": {}, \"error\": " + lk::detail::quoted(error.what()) +
             "}";
    }
  }();
  return sheet.c_str();
}

LKX_API lk_box* lk_create(const char* params_json, char* err, size_t err_len) {
  try {
    return build(params_json, nullptr);
  } catch (const std::exception& error) {
    lk::detail::copy_error(err, err_len, error.what());
    return nullptr;
  }
}

LKX_API int lk_start(lk_box* box, const lk_place* place, char* err, size_t err_len) {
  std::string error = box->started ? "lk_start: this box was started already (lk_create a new one)" : "";
  if (error.empty() && box->start(*place, error)) return 0;
  lk::detail::copy_error(err, err_len, error);
  return -1;
}

LKX_API int lk_step(lk_box* box, double now) {
  if (!box->place.stepped || box->failed.load()) return -1;
  box->step(now);
  box->bury();
  if (!box->failed.load()) return 0;
  box->deactivate_all();  // CONTRACT §5.7: the group deactivates; health says why
  return -1;
}

LKX_API int lk_push(lk_box* box, uint32_t in_port, struct ArrowArray* msg) { return box->push(in_port, msg); }

LKX_API int lk_take(lk_box* box, uint32_t out_port, struct ArrowArray* out) { return box->take(out_port, out); }

LKX_API int lk_wait(lk_box* box, int timeout_ms) {
  pollfd ready{box->wake_fd, POLLIN, 0};
  if (::poll(&ready, 1, timeout_ms) <= 0) return 0;
  std::uint64_t count = 0;
  (void)!::read(box->wake_fd, &count, sizeof count);
  return 1;
}

LKX_API void lk_stop(lk_box* box) { box->stop(); }

LKX_API int lk_raise(lk_box* box, int process, int thread, int below) {  // CONTRACT §15 A35, hosts only
  try {
    return box->raise(process, thread, below);
  } catch (const std::exception&) {
    return 0;
  }
}

LKX_API int lk_lower(lk_box* box, int process, int final) {
  try {
    return box->lower(process, final != 0);
  } catch (const std::exception&) {
    return 0;
  }
}

LKX_API void lk_sent(lk_box* box, uint32_t out_port, int sent) { box->sent(out_port, sent != 0); }

LKX_API void lk_destroy(lk_box* box) { delete box; }
