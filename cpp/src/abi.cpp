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
#include <tuple>
#include <type_traits>

#include "box.hpp"

namespace lk::detail {
namespace {

// ---------------------------------------------------------------- the v0 types, field by field in dataclass order

// The generated headers give every view a `rm::fields(view, visitor)` that walks its wire children in robot_msgs'
// field order (the schema-hash order, CONTRACT §7.1); found here by argument-dependent lookup. One visitor decodes,
// one encodes and one pins strings. WHY generated, not listed here: a hand-kept list drifted from robot_msgs and a
// C++ ServoState went out one child (`truth`) short under the full schema's hash.

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
  template <class N>
  static N item(const ArrowArray& array, std::int64_t at) {  // element `at` of a g, l, b or u array, or a +l of g
    if constexpr (std::is_same_v<N, bool>) {
      return static_cast<const std::uint8_t*>(array.buffers[1])[at / 8] >> (at % 8) & 1;
    } else if constexpr (std::is_same_v<N, std::string_view>) {
      const auto [begin, end] = range(array, at);
      return {static_cast<const char*>(array.buffers[2]) + begin, static_cast<std::size_t>(end - begin)};
    } else if constexpr (std::is_same_v<N, std::span<const double>>) {
      const auto [begin, end] = range(array, at);
      const ArrowArray& items = *array.children[0];
      return {static_cast<const double*>(items.buffers[1]) + items.offset + begin,
              static_cast<std::size_t>(end - begin)};
    } else {
      return static_cast<const N*>(array.buffers[1])[at];
    }
  }
  // the list (or map) at this row, or nullptr: missing (ok = false), null where nullable (`out` reset), or longer
  // than the view's capacity (ok = false: refused, never cut)
  template <class Out = std::nullptr_t>
  const ArrowArray* rows(std::size_t capacity, std::int64_t& begin, std::int64_t& end, Out* nullable = nullptr) {
    const ArrowArray* list = child();
    if (list == nullptr) return nullptr;
    if constexpr (!std::is_same_v<Out, std::nullptr_t>)
      if (!valid(*list, list->offset + row)) return nullable->reset(), nullptr;
    std::tie(begin, end) = range(*list, list->offset + row);
    if (end - begin > static_cast<std::int64_t>(capacity)) return ok = false, nullptr;
    return list;
  }
  template <class N, std::size_t C>
  void copy_items(const ArrowArray& list, std::int64_t begin, std::int64_t end, std::array<N, C>& out) {
    const ArrowArray& items = *list.children[0];
    for (auto at = begin; at < end; ++at) out[at - begin] = item<N>(items, items.offset + at);
  }
  template <class N, std::size_t C>
  void numbers(std::array<N, C>& out, std::size_t&) {
    std::int64_t begin = 0, end = 0;
    if (const ArrowArray* list = rows(C, begin, end)) copy_items(*list, begin, end, out);
  }
  template <class N, std::size_t C>
  void numbers(std::optional<std::array<N, C>>& out, std::size_t&) {
    std::int64_t begin = 0, end = 0;
    if (const ArrowArray* list = rows(C, begin, end, &out)) copy_items(*list, begin, end, out.emplace());
  }
  template <std::size_t C>
  void names(std::array<std::string_view, C>& out, std::size_t& size) {
    std::int64_t begin = 0, end = 0;
    if (const ArrowArray* list = rows(C, begin, end))
      copy_items(*list, begin, end, out), size = static_cast<std::size_t>(end - begin);
  }
  template <std::size_t C>
  void names(std::optional<std::array<std::string_view, C>>& out, std::size_t&) {  // the count is the required names'
    std::int64_t begin = 0, end = 0;
    if (const ArrowArray* list = rows(C, begin, end, &out)) copy_items(*list, begin, end, out.emplace());
  }
  template <class V, std::size_t C>
  void map(rm::Map<V, C>& out) {
    std::int64_t begin = 0, end = 0;
    const ArrowArray* map = rows(C, begin, end);
    if (map == nullptr) return;
    const ArrowArray& entries = *map->children[0];
    const ArrowArray &keys = *entries.children[0], &values = *entries.children[1];
    for (auto at = begin; at < end; ++at) {
      out.keys[at - begin] = item<std::string_view>(keys, keys.offset + entries.offset + at);
      out.values[at - begin] = item<V>(values, values.offset + entries.offset + at);
    }
    out.size = static_cast<std::size_t>(end - begin);
  }
  template <class N>
  void scalar(N& out) {
    if (const ArrowArray* array = child()) out = item<N>(*array, array->offset + row);
  }
  template <class N>
  void scalar(std::optional<N>& out) {
    const ArrowArray* array = child();
    if (array == nullptr || !valid(*array, array->offset + row)) return out.reset();
    out = item<N>(*array, array->offset + row);
  }
  void text(std::string_view& out) { scalar(out); }
  void span(std::span<const double>& out) { scalar(out); }
  void span(std::optional<std::span<const double>>& out) { scalar(out); }
  template <class M>
  void nest(M& out) {
    if (const ArrowArray* array = child()) read_struct(*array, out);
  }
  template <class M>
  void nest(std::optional<M>& out) {
    const ArrowArray* array = child();
    if (array == nullptr || !valid(*array, array->offset + row)) return out.reset();
    read_struct(*array, out.emplace());
  }
  template <class M>
  void read_struct(const ArrowArray& array, M& out) {
    Reader inner{&array, array.offset + row};
    fields(out, inner);
    ok = ok && inner.ok && inner.next == array.n_children;  // a child the view does not know is refused, as at the root
  }
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
  template <std::size_t C>
  void names(std::optional<std::array<std::string_view, C>>& names, std::size_t size) {
    if (names) this->names(*names, size);
  }
  template <class V, std::size_t C>
  void map(rm::Map<V, C>& map) {
    names(map.keys, map.size);
    if constexpr (std::is_same_v<V, std::string_view>) names(map.values, map.size);
  }
  template <class M>
  void nest(M& message) {
    fields(message, *this);
  }
  template <class M>
  void nest(std::optional<M>& message) {
    if (message) fields(*message, *this);
  }
  template <class... Any>
  void numbers(Any&&...) {}
  template <class... Any>
  void scalar(Any&&...) {}
  template <class... Any>
  void span(Any&&...) {}
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
  ArrowArray* column(const N* data, std::size_t size, bool null = false) {  // `size` elements of a g, l, b or u array
    if constexpr (std::is_same_v<N, std::string_view>) {
      std::vector<std::int32_t> offsets{0};
      std::string chars;
      for (std::size_t at = 0; at < size; ++at)
        chars += data[at], offsets.push_back(static_cast<std::int32_t>(chars.size()));
      return node(static_cast<std::int64_t>(size), null,
                  {nullptr, keep(offsets.data(), offsets.size() * 4), keep(chars.data(), chars.size())});
    } else if constexpr (std::is_same_v<N, bool>) {
      std::string bits((size + 7) / 8, '\0');
      for (std::size_t at = 0; at < size; ++at) bits[at / 8] = static_cast<char>(bits[at / 8] | data[at] << (at % 8));
      return node(static_cast<std::int64_t>(size), null, {nullptr, keep(bits.data(), bits.size())});
    } else {
      return node(static_cast<std::int64_t>(size), null, {nullptr, keep(data, size * sizeof(N))});
    }
  }
  template <class N>
  void list(const N* data, std::size_t size, bool null) {
    kids.push_back(node(1, null, {nullptr, ends(size)}, {column(data, size)}));
  }
  template <class N, std::size_t C>
  void numbers(const std::array<N, C>& in, std::size_t size) {
    list(in.data(), std::min(size, C), false);
  }
  template <class N, std::size_t C>
  void numbers(const std::optional<std::array<N, C>>& in, std::size_t size) {
    in ? list(in->data(), std::min(size, C), false) : list<N>(nullptr, 0, true);
  }
  template <std::size_t C>
  void names(const std::array<std::string_view, C>& in, std::size_t size) {
    numbers(in, size);
  }
  template <std::size_t C>
  void names(const std::optional<std::array<std::string_view, C>>& in, std::size_t size) {
    numbers(in, size);
  }
  template <class V, std::size_t C>
  void map(const rm::Map<V, C>& in) {
    const std::size_t size = std::min(in.size, C);
    ArrowArray* entries = node(static_cast<std::int64_t>(size), false, {nullptr},
                               {column(in.keys.data(), size), column(in.values.data(), size)});
    kids.push_back(node(1, false, {nullptr, ends(size)}, {entries}));
  }
  template <class N>
  void scalar(const N& value) {
    kids.push_back(column(&value, 1));
  }
  template <class N>
  void scalar(const std::optional<N>& value) {
    const N held = value.value_or(N{});
    kids.push_back(column(&held, 1, !value));
  }
  void text(std::string_view text) { scalar(text); }
  template <class M>
  void nest(M& message, bool null = false) {
    Writer inner{built};
    fields(message, inner);
    kids.push_back(node(1, null, {nullptr}, std::move(inner.kids)));
  }
  template <class M>
  void nest(std::optional<M>& message) {  // null: a default view under a null slot, as codec._zero writes (§7.1)
    M blank{};
    message ? nest(*message) : nest(blank, true);
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

Fields::Fields(const void* json, std::string* out, RowDefaults* tables) : json_(json), out_(out), tables_(tables) {}

namespace {
void up(std::string& path) {  // a walker's path one object up: "bus.joints[2].limits" -> "bus.joints[2]"
  const auto dot = path.rfind('.');
  path.resize(dot == std::string::npos ? 0 : dot);
}
}  // namespace

void Fields::enter(const char* key) {
  if (out_ != nullptr) {
    if (!out_->empty() && out_->back() != '{' && out_->back() != '[') *out_ += ", ";
    *out_ += key ? quoted(key) + ": {" : "{";
    if (key != nullptr) path_ += path_.empty() ? key : "." + std::string(key);
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
  if (out_ != nullptr) {
    *out_ += "}";
    return up(path_);
  }
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
  up(path_);
}

std::size_t Fields::open_rows(const char* key) {
  if (out_ != nullptr) {
    if (out_->back() != '{') *out_ += ", ";
    *out_ += quoted(key) + ": [";
    rows_.emplace_back(nullptr, key);
    return kAbsent;
  }
  const auto* array = static_cast<const Json*>(find(key, Json::kArray));
  rows_.emplace_back(array, key);
  return array != nullptr ? array->members.size() : kAbsent;  // absent: the default rows stay
}

void Fields::enter_row(std::size_t at) {
  if (out_ != nullptr) {
    *out_ += out_->back() == '[' ? "{" : ", {";
    path_ += (path_.empty() ? "" : ".") + rows_.back().second + "[]";
    return;
  }
  const auto* array = static_cast<const Json*>(rows_.back().first);
  const Json* row = array != nullptr ? &array->members[at].second : nullptr;
  path_ += (path_.empty() ? "" : ".") + rows_.back().second + "[" + std::to_string(at) + "]";
  if (row != nullptr && row->kind != Json::kObject)
    throw std::invalid_argument("params." + path_ + ": expected an object, got " + kKinds[row->kind]);
  stack_.push_back(row), marks_.push_back(seen_.size());
}

void Fields::close_rows() {
  if (out_ != nullptr) *out_ += "]";
  rows_.pop_back();
}

void Fields::note_rows(const char* key, void (*write_row)(Fields&)) {
  if (tables_ == nullptr) return;
  const std::string path = path_.empty() ? key : path_ + "." + key;
  for (const auto& [noted, _] : *tables_)
    if (noted == path) return;  // a table inside a row: the same for every row
  std::string row;
  Fields writer(nullptr, &row, tables_);  // its own tables are noted under this one's rows: "joints[].limits"
  writer.path_ = path + "[]";
  write_row(writer);
  tables_->emplace_back(path, std::move(row));
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
lk_box* build(const char* params_json, std::string* defaults, lk::detail::RowDefaults* tables = nullptr) {
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
  lk::detail::Fields fields(defaults ? nullptr : &root, defaults, tables);
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
      std::string params, rows;
      lk::detail::RowDefaults tables;
      const std::unique_ptr<lk_box> box(build(nullptr, &params, &tables));
      // CONTRACT §15 A44: a given row is filled from its table's row defaults, so lk.native() needs them
      for (const auto& [path, row] : tables) rows += (rows.empty() ? "" : ", ") + lk::detail::quoted(path) + ": " + row;
      return box->sheet() + ", \"params\": " + params + ", \"row_defaults\": {" + rows + "}}";
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
