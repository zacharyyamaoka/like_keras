// lk-dora-host <plan.json> <process>: one component .so as a dora 0.5.0 node (CONTRACT §6.4).
//
// The only C++ that links dora; the .so never does. It reads <process>.host beside plan.json (the plan's facts for
// this process as plain lines, written by like_keras/backends/dora/emit.py), dlopens the .so, says lk_hello, and
// runs the box through its C ABI (abi.h): RX pushes what arrives, TX sends what the box publishes, the RT thread
// inside the box never touches dora, so a daemon stall shows only as input age (T4). Under lockstep the box is
// stepped here instead, on the conservative schedule the Python host keeps (host.py): frames carry lk_t / lk_h.
//
// WHY these dora calls and no others (CONTRACT §6.4, docs/friction/contract-006.md): init_dora_node, next_event,
// event_type, event_as_arrow_input_with_info (the input id and the lk_t metadata; event_as_arrow_input drops both),
// send_arrow_output, new_metadata/set_int/get_int (lk_t, lk_h). Never the C node API (dora_next_event and
// dora_send_output alias one &mut DoraContext with no lock, and it reads only UInt8 inputs), never
// allocate_data_sample / DataSampleHandle / send_data_sample* or init_dora_node_from_id / init_dora_node_flexible /
// node_id / dataflow_id (removed in dora 1.0), never operators.
#include <dlfcn.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "dora-node-api.h"
#include "like_keras/abi.h"

#ifndef LK_DORA_VERSION
#define LK_DORA_VERSION "0.5.0"
#endif

struct ArrowSchema {  // the Arrow C Data Interface's schema (abi.h declares the array only)
  const char* format;
  const char* name;
  const char* metadata;
  int64_t flags;
  int64_t n_children;
  ArrowSchema** children;
  ArrowSchema* dictionary;
  void (*release)(ArrowSchema*);
  void* private_data;
};

namespace {

constexpr int64_t kForever = int64_t{1} << 62;
constexpr double kHelloTimeoutS = 2.0;  // CONTRACT §6.4: a hang or a skew is refused within 2 s

std::string g_node, g_markers;
std::atomic<bool> g_initialized{false};

void mark(const std::string& kind, const std::string& text = "") {
  std::ofstream(g_markers + "/" + kind + "." + g_node) << text;
}

[[noreturn]] void refuse(int code, const std::string& message, bool final_refusal) {
  std::fprintf(stderr, "like_keras %s: %s\n", g_node.c_str(), message.c_str());
  if (final_refusal) mark("failed", g_node + ": " + message);
  mark("exit");
  std::fflush(nullptr);
  _exit(code);
}

int64_t to_ns(double seconds) { return std::isinf(seconds) ? kForever : std::llround(seconds * 1e9); }

// ------------------------------------------------------------------ schemas from emit.py's `name:format:null:n;`

struct Tree {
  std::string name, format;
  bool nullable = false;
  std::vector<Tree> children;
};

Tree parse_tree(const std::string& line, size_t& at) {
  Tree tree;
  size_t end = line.find(';', at);
  std::stringstream fields(line.substr(at, end - at));
  std::string nullable, count;
  std::getline(fields, tree.name, ':');
  std::getline(fields, tree.format, ':');
  std::getline(fields, nullable, ':');
  std::getline(fields, count, ':');
  tree.nullable = nullable == "1";
  at = end + 1;
  for (int child = 0, n = std::stoi(count); child < n; ++child) tree.children.push_back(parse_tree(line, at));
  return tree;
}

void noop_release(ArrowSchema* schema) { schema->release = nullptr; }
void noop_release_array(ArrowArray* array) { array->release = nullptr; }

struct Schema {  // a persistent ArrowSchema tree; each send gets a copy of the root (dora consumes the root)
  std::deque<ArrowSchema> nodes;
  std::deque<std::vector<ArrowSchema*>> links;
  ArrowSchema* build(const Tree& tree) {
    ArrowSchema& node = nodes.emplace_back();
    auto& children = links.emplace_back();
    for (const Tree& child : tree.children) children.push_back(build(child));
    node = {tree.format.c_str(), tree.name.c_str(), nullptr, tree.nullable ? 2 : 0,
            static_cast<int64_t>(children.size()), children.empty() ? nullptr : children.data(), nullptr,
            noop_release, nullptr};
    return &node;
  }
};

bool same_formats(const ArrowSchema* schema, const Tree& tree) {
  if (schema == nullptr || tree.format != schema->format || schema->n_children != (int64_t)tree.children.size())
    return false;
  for (size_t child = 0; child < tree.children.size(); ++child)
    if (!same_formats(schema->children[child], tree.children[child])) return false;
  return true;
}

void release(ArrowArray& array) {
  if (array.release != nullptr) array.release(&array);
  array.release = nullptr;
}

void release(ArrowSchema& schema) {
  if (schema.release != nullptr) schema.release(&schema);
  schema.release = nullptr;
}

// ------------------------------------------------------------------ the plan's facts, the box, the wiring

std::string json_string(const std::string& json, const std::string& key, size_t from = 0) {
  size_t at = json.find("\"" + key + "\": \"", from);
  if (at == std::string::npos) return "";
  at += key.size() + 5;
  return json.substr(at, json.find('"', at) - at);
}

struct Input {
  std::string id, source, hash;
  int port = 0, queue = 0;
  Tree tree;
  bool checked = false;
  int64_t horizon = 0;  // ns: every frame with an earlier step time has arrived
  std::deque<std::pair<int64_t, ArrowArray>> pending;
};

struct Output {
  std::string id, hash;
  int port = 0;
  Tree tree;
  Schema schema;
  ArrowSchema* root = nullptr;
  int64_t promised = -1;
};

struct Box {
  uint32_t (*abi_version)();
  const char* (*describe)();
  lk_box* (*create)(const char*, char*, size_t);
  int (*start)(lk_box*, const lk_place*, char*, size_t);
  int (*step)(lk_box*, double);
  int (*push)(lk_box*, uint32_t, ArrowArray*);
  int (*take)(lk_box*, uint32_t, ArrowArray*);
  int (*wait)(lk_box*, int);
  void (*stop)(lk_box*);
  void (*destroy)(lk_box*);
  lk_box* box = nullptr;
};

template <class F>
void bind(void* library, const char* name, F& function) {
  function = reinterpret_cast<F>(dlsym(library, name));
  if (function == nullptr) refuse(1, std::string("the .so has no ") + name + ": is it a like_keras component?", true);
}

struct Host {
  std::string so, path, params, until_text, like_keras = "?";
  double hz = 0, until = 0;
  int fifo = 0, stepped = 0;
  std::vector<int> cpus;
  std::set<std::string> peers, acked;
  std::map<std::string, std::string> seen;  // peer -> the nonce of its hello verified here
  std::vector<Input> inputs;
  std::deque<Output> outputs;
  int exposed_outs = 0;
  std::string nonce;
  Box box;

  std::mutex sending;  // one OutputSender, two senders (TX and the hello answers on RX)
  std::mutex lock;
  std::condition_variable arrived;
  bool ready = false, stopping = false, started = false;
  int signalled = 0;  // SIGTERM, SIGHUP or SIGINT, taken on the sigwait thread (CONTRACT §15 A18)
  std::map<Input*, ArrowArray> held;  // real time, before lk_start: the newest frame of each input
  struct Frame {
    Input* input;
    int64_t sent, horizon;
    ArrowArray array;
  };
  std::deque<Frame> incoming;

  void read(const std::string& file) {
    std::ifstream in(file);
    if (!in) refuse(1, "cannot read " + file + " (written by the like_keras launcher)", true);
    std::string line;
    while (std::getline(in, line)) {
      std::stringstream fields(line);
      std::string key;
      fields >> key;
      if (key == "so") fields >> so;
      if (key == "path") fields >> path;
      if (key == "until") fields >> until_text;
      if (key == "markers") fields >> g_markers;
      if (key == "log") {  // stderr there as well: a refusal stays readable after dora restarts the node
        std::string log;
        fields >> log;
        if (std::freopen(log.c_str(), "a", stderr) != nullptr) std::setvbuf(stderr, nullptr, _IOLBF, 0);
      }
      if (key == "params") std::getline(fields >> std::ws, params);
      if (key == "peer") {
        std::string peer;
        fields >> peer;
        peers.insert(peer);
      }
      if (key == "place") {
        std::string cpu_list;
        fields >> hz >> fifo >> stepped >> cpu_list;
        std::stringstream split(cpu_list);
        for (std::string cpu; std::getline(split, cpu, ',');)
          if (cpu != "-") cpus.push_back(std::stoi(cpu));
      }
      if (key == "out") {
        Output& output = outputs.emplace_back();
        std::string tree;
        fields >> output.port >> output.id >> output.hash >> tree;
        size_t at = 0;
        output.tree = parse_tree(tree, at);
        output.root = output.schema.build(output.tree);
      }
      if (key == "in") {
        Input& input = inputs.emplace_back();
        std::string tree;
        fields >> input.port >> input.id >> input.source >> input.hash >> input.queue >> tree;
        size_t at = 0;
        input.tree = parse_tree(tree, at);
      }
    }
    until = until_text == "-" ? 0.0 : std::stod(until_text);
  }
  Input* input_named(const std::string& id) {
    for (Input& input : inputs)
      if (input.id == id) return &input;
    return nullptr;
  }
};

Host g_host;

// ------------------------------------------------------------------ sending

void send(::DoraNode& node, Output& output, ArrowArray* array, int64_t sent, int64_t horizon) {
  ArrowSchema schema = *output.root;  // dora consumes this copy; the children stay ours
  std::lock_guard guard(g_host.sending);
  DoraResult result;
  if (sent >= 0) {
    auto metadata = new_metadata();
    metadata->set_int("lk_t", sent);
    if (horizon >= 0) metadata->set_int("lk_h", horizon);
    result = send_arrow_output(node.send_output, output.id, reinterpret_cast<uint8_t*>(array),
                               reinterpret_cast<uint8_t*>(&schema), std::move(metadata));
  } else {
    result = send_arrow_output(node.send_output, output.id, reinterpret_cast<uint8_t*>(array),
                               reinterpret_cast<uint8_t*>(&schema));
  }
  if (!std::string(result.error).empty())
    std::fprintf(stderr, "like_keras %s: send %s: %s\n", g_node.c_str(), output.id.c_str(), result.error.c_str());
  release(*array);
}

void send_empty(::DoraNode& node, const std::string& id, int64_t promise) {  // a promise under lockstep
  ArrowArray array{0, 0, 0, 0, 0, nullptr, nullptr, nullptr, noop_release_array, nullptr};
  ArrowSchema schema{"n", "", nullptr, 2, 0, nullptr, nullptr, noop_release, nullptr};
  auto metadata = new_metadata();
  metadata->set_int("lk_t", promise);
  std::lock_guard guard(g_host.sending);
  send_arrow_output(node.send_output, id, reinterpret_cast<uint8_t*>(&array), reinterpret_cast<uint8_t*>(&schema),
                    std::move(metadata));
}

void say_hello(::DoraNode& node) {
  Host& host = g_host;
  std::string ports, seen;
  for (const Output& output : host.outputs) ports += (ports.empty() ? "\"" : ", \"") + output.id + "\": \"" + output.hash + "\"";
  for (const auto& [peer, nonce] : host.seen) seen += (seen.empty() ? "\"" : ", \"") + peer + "\": \"" + nonce + "\"";
  std::string hello = "{\"like_keras\": \"" + host.like_keras + "\", \"abi\": " + std::to_string(host.box.abi_version()) +
                      ", \"backend\": \"dora\", \"dora\": \"" LK_DORA_VERSION "\", \"node\": \"" + g_node +
                      "\", \"nonce\": \"" + host.nonce + "\", \"ports\": {" + ports + "}, \"seen\": {" + seen + "}}";
  const void* buffers[2] = {nullptr, hello.data()};
  ArrowArray array{(int64_t)hello.size(), 0, 0, 2, 0, buffers, nullptr, nullptr, noop_release_array, nullptr};
  ArrowSchema schema{"C", "", nullptr, 2, 0, nullptr, nullptr, noop_release, nullptr};
  std::lock_guard guard(host.sending);
  send_arrow_output(node.send_output, "lk_hello", reinterpret_cast<uint8_t*>(&array),
                    reinterpret_cast<uint8_t*>(&schema));
}

// ------------------------------------------------------------------ receiving (the RX thread)

void hello_from(::DoraNode& node, const std::string& peer, const ArrowArray& array) {
  Host& host = g_host;
  std::string hello(static_cast<const char*>(array.buffers[1]) + array.offset, array.length);
  const std::string mine = "like_keras " + host.like_keras + ", dora " LK_DORA_VERSION;
  for (const auto& [key, ours] : {std::pair<std::string, std::string>{"like_keras", host.like_keras},
                                  {"backend", "dora"}, {"dora", LK_DORA_VERSION}})
    if (json_string(hello, key) != ours)
      refuse(3, "node " + peer + " runs " + key + " " + json_string(hello, key) + ", node " + g_node + " runs " + key +
                    " " + ours + " (" + mine + "): every node of a deployment runs the same versions", true);
  for (const Input& input : host.inputs)
    if (input.source == peer && json_string(hello, input.id) != input.hash)
      refuse(3, input.id + ": node " + peer + " sends schema hash " + json_string(hello, input.id) + ", node " +
                    g_node + " reads it as " + input.hash + ": rebuild against the same robot_msgs", true);
  std::string nonce = json_string(hello, "nonce");
  std::fprintf(stderr, "lk_hello from %s (%.8s)\n", peer.c_str(), nonce.c_str());
  bool changed = host.seen[peer] != nonce;
  host.seen[peer] = nonce;
  size_t seen_at = hello.find("\"seen\": {");
  if (seen_at != std::string::npos && json_string(hello, g_node, seen_at) == host.nonce) host.acked.insert(peer);
  if (changed) say_hello(node);
  std::lock_guard guard(host.lock);
  if (host.acked == host.peers) host.ready = true;
  host.arrived.notify_all();
}

void receive(::DoraNode& node) {
  Host& host = g_host;
  for (;;) {
    auto event = next_event(node.events);
    DoraEventType type = event_type(event);
    if (type == DoraEventType::Stop || type == DoraEventType::AllInputsClosed) break;
    if (type != DoraEventType::Input) continue;
    ArrowArray array{};
    ArrowSchema schema{};
    auto info = event_as_arrow_input_with_info(std::move(event), reinterpret_cast<uint8_t*>(&array),
                                               reinterpret_cast<uint8_t*>(&schema));
    std::string id(info.id);
    if (!std::string(info.error).empty()) {
      std::fprintf(stderr, "like_keras %s: %s: %s\n", g_node.c_str(), id.c_str(), info.error.c_str());
      continue;
    }
    if (id.rfind("lk_hello.", 0) == 0) {
      hello_from(node, id.substr(9), array);
    } else if (Input* input = host.input_named(id)) {
      bool data = array.length > 0;
      if (data && !input->checked) {  // the schema's formats, compared once per port (CONTRACT §6.4)
        if (!same_formats(&schema, input->tree)) {
          std::fprintf(stderr, "like_keras %s: %s: an Arrow layout other than its type's; dropped\n", g_node.c_str(),
                       id.c_str());
          release(schema), release(array);
          continue;
        }
        input->checked = true;
      }
      if (host.stepped) {
        int64_t sent = 0, horizon = 0;
        try {
          sent = horizon = info.metadata->get_int("lk_t");
          if (data) horizon = info.metadata->get_int("lk_h");
        } catch (const std::exception&) {  // a data frame that is not its output's last of the step: no lk_h
        }
        std::lock_guard guard(host.lock);
        host.incoming.push_back({input, sent, horizon, array});
        host.arrived.notify_all();
        array.release = nullptr;
      } else {
        std::unique_lock guard(host.lock);
        if (!host.started) {  // held until the box runs; an empty frame (a retraction) replaces it too
          if (host.held.count(input)) release(host.held[input]);
          host.held[input] = array;
          array.release = nullptr;
        } else {
          guard.unlock();
          host.box.push(host.box.box, input->port, data ? &array : nullptr);  // NULL retracts (abi.h)
        }
      }
    }
    release(schema);
    release(array);
  }
  std::lock_guard guard(host.lock);
  host.stopping = true;
  host.arrived.notify_all();
}

// ------------------------------------------------------------------ real time: the TX thread

void transmit(::DoraNode& node) {
  Host& host = g_host;
  auto next_facilities = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  for (;;) {
    {
      std::lock_guard guard(host.lock);
      if (host.stopping) return;
    }
    bool news = host.box.wait(host.box.box, 100) > 0;
    bool facilities = std::chrono::steady_clock::now() >= next_facilities;
    if (facilities) next_facilities += std::chrono::seconds(1);  // percentiles and text are built here (A6)
    for (Output& output : host.outputs) {
      if (output.port < host.exposed_outs ? !news : !facilities) continue;
      ArrowArray array{};
      while (host.box.take(host.box.box, output.port, &array) == 1) send(node, output, &array, -1, -1);
    }
  }
}

// ------------------------------------------------------------------ lockstep (CONTRACT §5.5)

// WHY (CONTRACT §15 A10): under lockstep every health and metrics record is stamped at the facility tick, its
// publish time on the logical clock, as a Python component's is, and uptime_s shifts by the same amount. The box
// stamps its last step's time (0.98 s at the tick 1.0, at 50 Hz), so a Python <-> C++ swap shifted every stamp by a
// period. lk_take moved the record here (abi.h), its buffers are this host's until it is sent: rewritten in place.
void restamp(ArrowArray& record, const Tree& tree, double stamp) {
  const auto number = [](ArrowArray* array) {
    return const_cast<double*>(static_cast<const double*>(array->buffers[1])) + array->offset;
  };
  double *header_stamp = nullptr, *uptime = nullptr;
  bool operational = false;
  for (size_t at = 0; at < tree.children.size() && at < size_t(record.n_children); ++at) {
    const Tree& field = tree.children[at];
    ArrowArray* array = record.children[at];
    if (field.name == "header")
      for (size_t inner = 0; inner < field.children.size(); ++inner)
        if (field.children[inner].name == "stamp") header_stamp = number(array->children[inner]);
    if (field.name == "uptime_s") uptime = number(array);
    if (field.name == "mode") {
      const auto* ends = static_cast<const int32_t*>(array->buffers[1]) + array->offset;
      operational =
          std::string(static_cast<const char*>(array->buffers[2]) + ends[0], ends[1] - ends[0]) == "operational";
    }
  }
  if (header_stamp == nullptr) return;
  if (uptime != nullptr && operational) *uptime += stamp - *header_stamp;  // C++ sends 0 unless operational
  *header_stamp = stamp;
}

int run_lockstep(::DoraNode& node) {
  Host& host = g_host;
  int64_t tick = 0, second = 0;
  bool dead = false;
  std::map<Output*, std::vector<ArrowArray>> frames;
  auto next_time = [&] { return std::min(double(second), host.hz > 0 ? tick / host.hz : INFINITY); };
  for (;;) {
    double now = next_time();
    if (now >= host.until) break;
    int64_t now_ns = to_ns(now);
    {
      std::unique_lock guard(host.lock);
      for (;;) {
        for (Host::Frame& frame : host.incoming) {
          frame.input->horizon = std::max(frame.input->horizon, frame.horizon);
          if (frame.array.length > 0) frame.input->pending.emplace_back(frame.sent, frame.array);
          else release(frame.array);
        }
        host.incoming.clear();
        bool waiting = false;
        for (const Input& input : host.inputs) waiting |= input.horizon < now_ns;
        if (!waiting || host.stopping) break;
        host.arrived.wait(guard);
      }
      if (host.stopping) break;
    }
    for (Input& input : host.inputs) {  // what was sent strictly before now: a latest input gets the newest only
      while (!input.pending.empty() && input.pending.front().first < now_ns) {
        auto [sent, array] = input.pending.front();
        input.pending.pop_front();
        bool newest = input.pending.empty() || input.pending.front().first >= now_ns;
        if (input.queue > 0 || newest) host.box.push(host.box.box, input.port, &array);
        release(array);
      }
    }
    bool facilities = double(second) <= now;
    if (facilities) {
      ++second;  // the facilities first at equal times, as inproc
    } else {
      ++tick;
      if (!dead && host.box.step(host.box.box, now) != 0) dead = true;  // §5.7: the box deactivated itself
    }
    for (Output& output : host.outputs) {
      if ((output.port < host.exposed_outs) == facilities) continue;
      ArrowArray array{};
      while (host.box.take(host.box.box, output.port, &array) == 1) {
        if (output.port > host.exposed_outs) restamp(array, output.tree, now);  // health, metrics; never a log line
        frames[&output].push_back(array), array = {};
      }
    }
    int64_t promise = to_ns(next_time());
    for (Output& output : host.outputs) {
      auto& sent = frames[&output];
      for (size_t index = 0; index < sent.size(); ++index)
        send(node, output, &sent[index], now_ns, index + 1 == sent.size() ? promise : -1);
      if (!sent.empty()) output.promised = promise;
      sent.clear();
      if (output.promised < promise) send_empty(node, output.id, promise), output.promised = promise;
    }
  }
  for (Output& output : host.outputs) send_empty(node, output.id, kForever);
  mark("done");
  std::unique_lock guard(host.lock);
  host.arrived.wait(guard, [&] { return host.stopping; });
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: lk-dora-host <plan.json> <process>\n");
    return 2;
  }
  Host& host = g_host;
  g_node = argv[2];
  std::string plan = argv[1];
  host.read(plan.substr(0, plan.rfind('/') + 1) + g_node + ".host");
  // CONTRACT §15 A18: SIGTERM (systemctl stop, kill), SIGHUP and SIGINT end the node the way dora's Stop does, so
  // every component's deactivate (the controller's hand-back) runs. Blocked here, before any thread exists (dora's
  // inherit the mask), and taken on one sigwait thread; the stop itself runs on main once TX has returned, so lk_take
  // never runs on two threads at once (as the raw-dora node's node_main.cpp does, credit).
  sigset_t ending;
  sigemptyset(&ending);
  for (int number : {SIGTERM, SIGHUP, SIGINT}) sigaddset(&ending, number);
  pthread_sigmask(SIG_BLOCK, &ending, nullptr);
  std::thread([ending] {
    int number = 0;
    while (sigwait(&ending, &number) != 0) {
    }
    std::lock_guard guard(g_host.lock);
    g_host.signalled = number, g_host.stopping = true;
    g_host.arrived.notify_all();
  }).detach();
  std::thread([] {  // a 2 s watchdog around init_dora_node (CONTRACT §6.4: exit 4 naming the pinned version)
    std::this_thread::sleep_for(std::chrono::duration<double>(kHelloTimeoutS));
    if (!g_initialized.load())
      refuse(4, "init_dora_node did not return within 2 s: is a dora " LK_DORA_VERSION " daemon running?", false);
  }).detach();
  ::DoraNode node = init_dora_node();
  g_initialized.store(true);
  void* library = dlopen(host.so.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (library == nullptr) refuse(1, std::string("dlopen: ") + dlerror(), true);
  Box& box = host.box;
  bind(library, "lk_abi_version", box.abi_version), bind(library, "lk_describe", box.describe);
  bind(library, "lk_create", box.create), bind(library, "lk_start", box.start), bind(library, "lk_step", box.step);
  bind(library, "lk_push", box.push), bind(library, "lk_take", box.take), bind(library, "lk_wait", box.wait);
  bind(library, "lk_stop", box.stop), bind(library, "lk_destroy", box.destroy);
  std::string description = box.describe();
  host.like_keras = json_string(description, "like_keras");
  for (size_t at = description.find("\"dir\": \"out\""); at != std::string::npos;
       at = description.find("\"dir\": \"out\"", at + 1))
    ++host.exposed_outs;  // the facility ports come after the exposed Outs (abi.h)
  char error[1024] = {};
  box.box = box.create(host.params.c_str(), error, sizeof error);
  if (box.box == nullptr) refuse(1, std::string("lk_create refused: ") + error, true);
  std::random_device entropy;
  char nonce[33];
  std::snprintf(nonce, sizeof nonce, "%08x%08x%08x%08x", entropy(), entropy(), entropy(), entropy());
  host.nonce = nonce;
  say_hello(node);
  std::thread rx(receive, std::ref(node));
  {
    std::unique_lock guard(host.lock);
    bool ok = host.arrived.wait_for(guard, std::chrono::duration<double>(kHelloTimeoutS),
                                    [&] { return host.ready || host.peers.empty() || host.stopping; });
    if (!ok)
      refuse(3, "no verified lk_hello from every peer within 2 s (this node: like_keras " + host.like_keras +
                    ", dora " LK_DORA_VERSION "; theirs: unknown, they never answered)", false);
  }
  lk_place place{host.hz, host.cpus.empty() ? nullptr : host.cpus.data(), int(host.cpus.size()), host.fifo,
                 host.stepped ? 0 : 1, host.stepped};
  if (box.start(box.box, &place, error, sizeof error) != 0) {  // §5.7, T7: refused before any data frame
    mark("refused", error);
    refuse(1, std::string("activate refused: ") + error, host.stepped != 0);
  }
  {
    std::lock_guard guard(host.lock);
    for (auto& [input, array] : host.held) host.box.push(host.box.box, input->port, array.length ? &array : nullptr);
    host.held.clear();
    host.started = true;
  }
  mark("started");
  int code = 0;
  if (host.stepped) {
    code = run_lockstep(node);
  } else {
    std::thread tx(transmit, std::ref(node));
    tx.join();
  }
  box.stop(box.box);  // the RT thread joined, the components deactivated in reverse order
  if (host.signalled != 0) {  // RX may wait in next_event for good: neither joined nor raced by lk_destroy
    mark("exit");
    std::fflush(nullptr);
    _exit(128 + host.signalled);
  }
  rx.join();
  box.destroy(box.box);
  mark("exit");
  std::fflush(nullptr);
  _exit(code);  // WHY _exit: dora's runtime threads are not ours to join
}
