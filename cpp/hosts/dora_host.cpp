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
#include <dirent.h>
#include <dlfcn.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <iterator>
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
// WHY a node taking a signal outlives its box for a while (fix round 1 after the first consumer, item 2): the launcher
// stops a dataflow by signalling its nodes first, upstream first (A30, A37), then `dora stop`. Exiting at once, the
// node would be restarted by dora (restart_policy: always) and take over the bus again mid-stop; waiting for dora's
// Stop, its last records reach live readers and it exits with restarts already disabled. Bounded, for a signal from
// anyone else, at longer than the launcher's whole drain (DRAIN_S, 10 s): a node of its first wave waits while later
// waves drain (A37).
constexpr double kLingerS = 15.0;

std::string g_node, g_markers;
std::atomic<bool> g_initialized{false};
::DoraNode* g_dora = nullptr;          // once init_dora_node has returned
std::thread::id g_main;                // the thread whose final refusals wait for dora's Stop (A34)
std::atomic<bool> g_receiving{false};  // the RX thread runs: it takes the Stop, main waits to hear of it
void hold_until_stopped();             // below, beside the Host it waits on

void mark(const std::string& kind, const std::string& text = "") {
  std::ofstream(g_markers + "/" + kind + "." + g_node) << text;
}

[[noreturn]] void refuse(int code, const std::string& message, bool final_refusal) {
  std::fprintf(stderr, "like_keras %s: %s\n", g_node.c_str(), message.c_str());
  if (final_refusal) mark("failed", g_node + ": " + message);
  // WHY wait for dora's Stop (CONTRACT §15 A34, consumer-017): every node runs restart_policy `always` (§9.2), so a
  // node that exited on a final refusal (activate, lk_create, dlopen) was restarted 30-100 times a second on the
  // cores the powered arm shares. The launcher raises on the marker and stops the dataflow; that Stop disables
  // restarts, and only then does this node exit, never having sent a data frame.
  if (final_refusal && g_dora != nullptr && std::this_thread::get_id() == g_main) hold_until_stopped();
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
  // CONTRACT §15 A35: the boundary's realtime and send accounting; nullptr in a .so built before them
  int (*raise)(lk_box*, int, int, int) = nullptr;
  int (*lower)(lk_box*, int, int) = nullptr;
  void (*sent)(lk_box*, uint32_t, int) = nullptr;
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
  bool ready = false, stopping = false, started = false, closed = false;  // closed: dora's Stop or no input left
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

void hold_until_stopped() {
  if (!g_receiving.load()) {  // no RX thread yet: main reads the events itself, up to the Stop
    for (;;) {
      auto event = next_event(g_dora->events);
      const DoraEventType type = event_type(event);
      if (type == DoraEventType::Stop || type == DoraEventType::AllInputsClosed) return;
    }
  }
  std::unique_lock guard(g_host.lock);
  g_host.arrived.wait(guard, [] { return g_host.closed || g_host.signalled != 0; });
}

// ------------------------------------------------------------------ sending

bool send(::DoraNode& node, Output& output, ArrowArray* array, int64_t sent, int64_t horizon) {
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
  const bool went = std::string(result.error).empty();
  if (!went)
    std::fprintf(stderr, "like_keras %s: send %s: %s\n", g_node.c_str(), output.id.c_str(), result.error.c_str());
  release(*array);
  return went;
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

// ------------------------------------------------------------------ the boundary's realtime (CONTRACT §15 A35)
//
// The box's ladder (abi.h: lk_raise, lk_lower; cpp/src/rt.cpp) has the RT thread and, since this process blocks
// SIGXCPU in every thread, a sentinel. This raises the rest of the boundary below them, as the raw-dora node does
// (moteus_controller_node node_main.cpp and rt.cpp PrivateDaemonRealtime, fix rounds 2-6, lifted with credit): RX, TX
// and dora's own threads in this process at the RT thread's priority - kIoBelow, then the daemon's data-path threads
// at - kDaemonBelow, in that order (rtkit caps grants per user: the least important asks last). The daemon only when
// it is private: this node's parent is a `dora run`, or a `dora daemon` every node of which runs this plan; a shared
// daemon serves other dataflows and is never touched. It is realtime only while every peer is connected: a node of
// this plan that exits (a pidfd each, Siblings) or an input dora closes demotes it at once; it is raised again once
// its threads are quiet (none ran kQuietRunNs without sleeping for kQuietNs, and kQuietNs after the last departure at
// least), and lk_stop demotes it for good, waiting for any grant in flight.
// WHY the I/O (consumer-018): with only the RT thread raised, `target` lost 2-3 % of its states at a subscriber (the
// raw node 0-0.1 %), with a hand-off tail ~10x the raw node's. WHY the daemon (raw node, round 2): with its own dora
// threads raised, every remaining gap over 50 ms was the nice-19 daemon's. WHY the pause (round 5, measured): when a
// node disconnects, the daemon thread that served it runs without sleeping, 8-71 ms at load 25-33 and 535 ms at load
// 42; still SCHED_RR it crossed RLIMIT_RTTIME and the daemon died (exit 137 / 152). WHY raise it again (round 6):
// demoted for the rest of a run, up to 20.7 % of the node's states went unpublished. Not covered, as in the raw node:
// a node SIGKILLed outright leaves its daemon raised.
constexpr int kIoBelow = 10, kDaemonBelow = 15;  // RT 20 (rtkit's highest here): I/O 10, daemon 5, as the raw node
constexpr int64_t kQuietNs = 50'000'000, kQuietRunNs = 5'000'000, kQuietLookNs = 5'000'000;

struct Boundary {
  int daemon = 0;                // this node's parent, read at the start (a dead daemon's orphan gets another parent)
  std::vector<int> dora_threads; // the threads init_dora_node started in this process
  std::atomic<int> rx{0}, tx{0}; // their thread ids, once they run
  std::mutex lock;
  std::condition_variable changed;
  bool ours = false, paused = false, over = false;  // ours: found private, so raised (and only then lowered)
  int64_t paused_at = 0;  // CLOCK_MONOTONIC ns of the last departure
} g_boundary;

int64_t monotonic_ns() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return int64_t{now.tv_sec} * 1'000'000'000 + now.tv_nsec;
}

std::string proc_text(const std::string& path) {
  std::ifstream file(path);
  return std::string(std::istreambuf_iterator<char>(file), {});
}

std::vector<int> numbers_in(const std::string& text) {  // a /proc children file: pids, space separated
  std::vector<int> found;
  std::stringstream words(text);
  for (int number = 0; words >> number;) found.push_back(number);
  return found;
}

std::vector<int> tasks_of(int process) {
  std::vector<int> found;
  if (DIR* tasks = opendir(("/proc/" + std::to_string(process) + "/task").c_str())) {
    while (const dirent* task = readdir(tasks))
      if (std::isdigit(static_cast<unsigned char>(task->d_name[0]))) found.push_back(std::atoi(task->d_name));
    closedir(tasks);
  }
  return found;
}

std::vector<std::string> command_line(int process) {
  std::vector<std::string> words;
  std::stringstream text(proc_text("/proc/" + std::to_string(process) + "/cmdline"));
  for (std::string word; std::getline(text, word, '\0');) words.push_back(word);
  return words;
}

// Private: a `dora run` (one dataflow, its daemon inside the CLI: the raw node's rule), or a `dora daemon` whose every
// node (child process) runs this plan; `nodes` gets them. like_keras seam: dora's CLI and how its daemon spawns nodes.
bool private_daemon(int daemon, const std::string& plan, std::vector<int>& nodes) {
  const auto argv = command_line(daemon);
  nodes.clear();
  for (int task : tasks_of(daemon))
    for (int node : numbers_in(proc_text("/proc/" + std::to_string(daemon) + "/task/" + std::to_string(task) +
                                         "/children")))
      nodes.push_back(node);
  for (size_t at = 0; at + 1 < argv.size(); ++at) {
    if (argv[at] != "dora" && !(argv[at].size() > 5 && argv[at].ends_with("/dora"))) continue;
    return argv[at + 1] == "run" ||
           (argv[at + 1] == "daemon" && !nodes.empty() && std::all_of(nodes.begin(), nodes.end(), [&](int node) {
             const auto words = command_line(node);
             return std::find(words.begin(), words.end(), plan) != words.end();
           }));
  }
  return false;
}

// The other nodes of this plan, each a pidfd: one that exits is a peer leaving. WHY not only dora's InputClosed:
// under restart_policy `always` (§9.2) dora 0.5 restarts a killed peer and closes no input, yet its daemon still
// serves that disconnect (measured: a SIGKILLed subscriber, no event here).
struct Siblings {
  std::map<int, int> watched;  // pid -> pidfd
  ~Siblings() {
    for (const auto& [pid, fd] : watched) close(fd);
  }
  void watch(const std::vector<int>& nodes) {
    for (int node : nodes)
      if (node != ::getpid() && !watched.count(node))
        if (const int fd = static_cast<int>(::syscall(SYS_pidfd_open, node, 0)); fd >= 0) watched[node] = fd;
  }
  bool left() {  // any exited since the last look (no wait)
    bool gone = false;
    for (auto at = watched.begin(); at != watched.end();) {
      pollfd exited{at->second, POLLIN, 0};
      if (::poll(&exited, 1, 0) == 1) {
        close(at->second), at = watched.erase(at), gone = true;
      } else {
        ++at;
      }
    }
    return gone;
  }
};

// The daemon's threads on its nodes' data path. WHY skip some (raw node, round 3): rtkit supervises at most 25 threads
// per user, and dora's Ctrl-C handler and its zenoh runtimes (app-N, net-N, acc-N, rx-N, tx-N: links to daemons on
// other machines, none here) carry no node's data. like_keras seam: the names dora gives its daemon's threads.
std::vector<int> daemon_data_path(int daemon) {
  std::vector<int> found;
  for (int task : tasks_of(daemon)) {
    std::string name = proc_text("/proc/" + std::to_string(daemon) + "/task/" + std::to_string(task) + "/comm");
    static const char* const kZenoh[] = {"app-", "net-", "acc-", "rx-", "tx-"};
    while (!name.empty() && std::isspace(static_cast<unsigned char>(name.back()))) name.pop_back();
    if (name == "ctrl-c" ||
        std::any_of(std::begin(kZenoh), std::end(kZenoh), [&](const char* zenoh) { return name.starts_with(zenoh); }))
      continue;
    found.push_back(task);
  }
  return found;
}

void pause_daemon() {  // RX, when a peer's input closes: the daemon's threads go back now, before its spin
  {
    std::lock_guard guard(g_boundary.lock);
    if (!g_boundary.ours || g_boundary.over) return;
    g_boundary.paused = true, g_boundary.paused_at = monotonic_ns();
  }
  g_boundary.changed.notify_all();
  const int demoted = g_host.box.lower(g_host.box.box, g_boundary.daemon, 0);
  std::fprintf(stderr, "like_keras %s: a peer left: the daemon (pid %d) SCHED_OTHER until quiet, %d demoted\n",
               g_node.c_str(), g_boundary.daemon, demoted);
}

// Blocks until the daemon's data-path threads are quiet (above); false when the boundary is over or the daemon gone.
// WHY schedstat and the voluntary switches: RLIMIT_RTTIME counts CPU time since a thread last slept, and the round-5
// spinner read state S while on a CPU.
bool wait_until_quiet() {
  std::map<int, std::pair<long long, long long>> seen;  // thread -> (voluntary switches, on-CPU ns at that count)
  int64_t quiet_from = -1;
  for (;;) {
    int64_t paused_at = 0;
    {
      std::unique_lock guard(g_boundary.lock);
      if (g_boundary.over) return false;
      paused_at = g_boundary.paused_at;
    }
    const std::string base = "/proc/" + std::to_string(g_boundary.daemon) + "/task/";
    const std::vector<int> threads = daemon_data_path(g_boundary.daemon);
    if (threads.empty()) return false;
    long long longest = 0;
    for (int thread : threads) {
      const std::string status = proc_text(base + std::to_string(thread) + "/status");
      const size_t at = status.find("voluntary_ctxt_switches:");
      long long on_cpu = -1;
      std::stringstream(proc_text(base + std::to_string(thread) + "/schedstat")) >> on_cpu;
      if (at == std::string::npos || on_cpu < 0) continue;  // exited meanwhile
      const long long switches = std::atoll(status.c_str() + at + 24);
      auto& [count, since] = seen[thread];
      if (count != switches) count = switches, since = on_cpu;
      longest = std::max(longest, on_cpu - since);
    }
    const int64_t now = monotonic_ns();
    quiet_from = longest > kQuietRunNs ? -1 : quiet_from < 0 ? now : quiet_from;
    if (quiet_from >= 0 && now - quiet_from >= kQuietNs && now - paused_at >= kQuietNs) return true;
    std::unique_lock guard(g_boundary.lock);
    g_boundary.changed.wait_for(guard, std::chrono::nanoseconds(kQuietLookNs), [] { return g_boundary.over; });
  }
}

// The I/O threads, then the daemon: now, at 1 s and 3 s (rtkit's grant is per thread, and the daemon starts some
// later), then every second, each time checked still private (else demoted for good, and never asked again), and
// again after each pause once quiet. Every grant goes through the box's ladder, so lk_stop ends them all.
void elevate(std::string plan) {
  Box& box = g_host.box;
  if (box.raise == nullptr) return;  // a .so built before A35: SCHED_OTHER, as before
  while (g_boundary.tx.load() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  std::vector<int> io = g_boundary.dora_threads;
  io.push_back(g_boundary.rx.load()), io.push_back(g_boundary.tx.load());
  int raised = 0;
  for (int thread : io) raised += box.raise(box.box, 0, thread, kIoBelow);
  std::fprintf(stderr, "like_keras %s: RX, TX and dora's threads: %d of %zu realtime, %d below the RT thread\n",
               g_node.c_str(), raised, io.size(), kIoBelow);
  const int daemon = g_boundary.daemon;
  const auto began = std::chrono::steady_clock::now();
  const auto at = [&](double seconds) {
    return began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
  };
  Siblings siblings;
  std::vector<int> nodes;
  for (double due_s = 0.0; daemon > 1;) {
    bool paused = false;
    {
      std::unique_lock guard(g_boundary.lock);
      const auto look = std::chrono::steady_clock::now() + std::chrono::nanoseconds(kQuietLookNs * 4);
      g_boundary.changed.wait_until(guard, std::min(at(due_s), look),
                                    [] { return g_boundary.over || g_boundary.paused; });
      if (g_boundary.over) return;
      paused = g_boundary.paused;
    }
    if (!paused && siblings.left()) {
      pause_daemon();
      continue;
    }
    if (paused && !wait_until_quiet()) return;
    if (!paused && std::chrono::steady_clock::now() < at(due_s)) continue;
    if (!paused) due_s = due_s < 1.0 ? 1.0 : due_s < 3.0 ? 3.0 : due_s + 1.0;
    const bool private_now = private_daemon(daemon, plan, nodes);
    siblings.watch(nodes);
    bool was_ours = false;
    {
      std::lock_guard guard(g_boundary.lock);
      if (g_boundary.over) return;
      g_boundary.paused = false, was_ours = std::exchange(g_boundary.ours, private_now);
    }
    if (!private_now) {  // never private: never touched; shared since: what it was given goes back, for good
      if (was_ours) box.lower(box.box, daemon, 1);
      std::fprintf(stderr, "like_keras %s: the daemon (pid %d) is not private to this dataflow: left %s\n",
                   g_node.c_str(), daemon, was_ours ? "SCHED_OTHER for good" : "alone");
      return;
    }
    const std::vector<int> threads = daemon_data_path(daemon);
    int realtime = 0;
    for (int thread : threads) {
      {
        std::lock_guard guard(g_boundary.lock);
        if (g_boundary.paused || g_boundary.over) break;
      }
      realtime += box.raise(box.box, daemon, thread, kDaemonBelow);
    }
    if (paused || !was_ours)
      std::fprintf(stderr, "like_keras %s: the daemon (pid %d)%s: %d of %zu data-path threads realtime\n",
                   g_node.c_str(), daemon, paused ? " is quiet again" : ", private", realtime, threads.size());
    std::lock_guard guard(g_boundary.lock);  // a grant that landed during a new pause goes back at once
    if (g_boundary.paused) box.lower(box.box, daemon, 0);
  }
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
  g_boundary.rx.store(static_cast<int>(::gettid()));
  for (;;) {
    auto event = next_event(node.events);
    DoraEventType type = event_type(event);
    if (type == DoraEventType::Stop || type == DoraEventType::AllInputsClosed) break;
    if (type == DoraEventType::InputClosed) pause_daemon();  // a peer left (A35)
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
  host.stopping = host.closed = true;
  host.arrived.notify_all();
}

// ------------------------------------------------------------------ real time: the TX thread

void transmit(::DoraNode& node) {
  Host& host = g_host;
  g_boundary.tx.store(static_cast<int>(::gettid()));
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
      while (host.box.take(host.box.box, output.port, &array) == 1) {
        const bool went = send(node, output, &array, -1, -1);
        // CONTRACT §15 A35: the box counts an exposed Out's steps that never went out and its worst send gap
        if (output.port < host.exposed_outs && host.box.sent != nullptr) host.box.sent(host.box.box, output.port, went);
      }
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

// CONTRACT §15 A28: after the last deactivate, the facility ports once more, so what the box logged in its last
// partial second and in deactivate(), and its last health and metrics, are sent on every stop. Under lockstep
// (`sent` >= 0) each frame keeps the output's last promise and health and metrics are stamped at the last step.
void drain(::DoraNode& node, int64_t sent, double stamp) {
  Host& host = g_host;
  for (Output& output : host.outputs) {
    ArrowArray array{};
    while (output.port >= host.exposed_outs && host.box.take(host.box.box, output.port, &array) == 1) {
      if (sent >= 0 && output.port > host.exposed_outs) restamp(array, output.tree, stamp);
      send(node, output, &array, sent >= 0 ? std::max(sent, output.promised) : -1, -1);
    }
  }
}

int run_lockstep(::DoraNode& node) {
  Host& host = g_host;
  int64_t tick = 0, second = 0;
  double last_step = 0.0;
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
      ++tick, last_step = now;
      if (!dead && host.box.step(host.box.box, now) != 0) dead = true;  // §5.7: the box deactivated itself
    }
    for (Output& output : host.outputs) {
      // WHY the log port after every step too: a reader gets a Python component's line in the step that logs it, and
      // must get a native box's alike, in another process as in one (fix round 1 after the first consumer, item 3)
      const bool log = output.port == host.exposed_outs;
      if (!log && (output.port < host.exposed_outs) == facilities) continue;
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
  host.box.stop(host.box.box);  // deactivate in reverse order, then the last records before the last promise (A28)
  drain(node, 0, last_step);
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
  g_boundary.daemon = static_cast<int>(::getppid());  // the dora daemon that started this node (A35)
  g_node = argv[2], g_main = std::this_thread::get_id();
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
  // CONTRACT §15 A35: SIGXCPU blocked in every thread too, dora's included, so the box's ladder reads it on its
  // sentinel (cpp/src/rt.cpp) and demotes whichever realtime thread ran past RLIMIT_RTTIME's soft limit
  sigset_t xcpu;
  sigemptyset(&xcpu), sigaddset(&xcpu, SIGXCPU);
  pthread_sigmask(SIG_BLOCK, &xcpu, nullptr);
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
  const std::vector<int> before = tasks_of(::getpid());
  ::DoraNode node = init_dora_node();
  g_dora = &node;
  g_initialized.store(true);
  for (int thread : tasks_of(::getpid()))  // dora's own (its event stream, its daemon channel), raised with RX and TX
    if (std::find(before.begin(), before.end(), thread) == before.end()) g_boundary.dora_threads.push_back(thread);
  void* library = dlopen(host.so.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (library == nullptr) refuse(1, std::string("dlopen: ") + dlerror(), true);
  Box& box = host.box;
  bind(library, "lk_abi_version", box.abi_version), bind(library, "lk_describe", box.describe);
  bind(library, "lk_create", box.create), bind(library, "lk_start", box.start), bind(library, "lk_step", box.step);
  bind(library, "lk_push", box.push), bind(library, "lk_take", box.take), bind(library, "lk_wait", box.wait);
  bind(library, "lk_stop", box.stop), bind(library, "lk_destroy", box.destroy);
  box.raise = reinterpret_cast<decltype(box.raise)>(dlsym(library, "lk_raise"));  // optional (A35)
  box.lower = reinterpret_cast<decltype(box.lower)>(dlsym(library, "lk_lower"));
  box.sent = reinterpret_cast<decltype(box.sent)>(dlsym(library, "lk_sent"));
  // WHY: a .so built before A35 has no sentinel, only A19's SIGXCPU handler; RX, TX and its RT thread start from
  // here and inherit main's mask, so the signal reaches its handler again (dora's threads stay SCHED_OTHER then)
  if (box.raise == nullptr) pthread_sigmask(SIG_UNBLOCK, &xcpu, nullptr);
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
  g_receiving.store(true);
  std::thread rx(receive, std::ref(node));
  {
    std::unique_lock guard(host.lock);
    bool ok = host.arrived.wait_for(guard, std::chrono::duration<double>(kHelloTimeoutS),
                                    [&] { return host.ready || host.peers.empty() || host.stopping; });
    if (!ok)
      refuse(3, "no verified lk_hello from every peer within 2 s (this node: like_keras " + host.like_keras +
                    ", dora " LK_DORA_VERSION "; theirs: unknown, they never answered)", false);
  }
  for (const Input& input : host.inputs)  // WHY (§15 A23, abi.h): a queue input this node wires gates; one it leaves
    if (input.queue > 0) box.push(box.box, input.port, nullptr);  // unwired (an optional one) reads NULL, never gates
  lk_place place{host.hz, host.cpus.empty() ? nullptr : host.cpus.data(), int(host.cpus.size()), host.fifo,
                 host.stepped ? 0 : 1, host.stepped};
  if (box.start(box.box, &place, error, sizeof error) != 0) {  // §5.7, T7: refused before any data frame
    mark("refused", error);
    refuse(1, std::string("activate refused: ") + error, true);  // final, in real time and lockstep alike (A34)
  }
  {
    std::lock_guard guard(host.lock);
    for (auto& [input, array] : host.held) host.box.push(host.box.box, input->port, array.length ? &array : nullptr);
    host.held.clear();
    host.started = true;
  }
  mark("started", std::to_string(::getpid()));  // the launcher signals this node first on a stop (item 2)
  int code = 0;
  if (host.stepped) {
    code = run_lockstep(node);
  } else {
    std::thread tx(transmit, std::ref(node));
    std::thread elevator(elevate, plan);  // the boundary's realtime, after lk_start (the grant order, A35)
    tx.join();
    {
      std::lock_guard guard(g_boundary.lock);
      g_boundary.over = true;
    }
    g_boundary.changed.notify_all();
    box.stop(box.box);  // the RT thread joined, the components deactivated in reverse order, every grant ended
    elevator.join();
    drain(node, -1, 0.0);
    mark("drained");
  }
  if (host.signalled != 0) {  // RX may wait in next_event for good: neither joined nor raced by lk_destroy
    std::unique_lock guard(host.lock);
    host.arrived.wait_for(guard, std::chrono::duration<double>(kLingerS), [&] { return host.closed; });
    mark("exit");
    std::fflush(nullptr);
    _exit(host.closed ? 0 : 128 + host.signalled);  // after dora's Stop: a stop, as without the signal
  }
  rx.join();
  box.destroy(box.box);
  mark("exit");
  std::fflush(nullptr);
  _exit(code);  // WHY _exit: dora's runtime threads are not ours to join
}
