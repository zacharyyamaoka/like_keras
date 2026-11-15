// The fake moteus bus of the reference consumer (CONTRACT §10): what MoteusBus drives when `twin = true`.
//
// A board file shared with kpi/tasks/arm_cell/fake_bus.py (same bytes, same rules), so a Python test can read the
// boards a C++ node fed, and a restarted node finds the boards as the killed one left them (T6, T7):
// - BUS_LOCK: an flock on `lock_path`, taken in activate before the board file opens; a second bus refuses (R1, T7).
// - take_over: a stop only when a board latched its watchdog (mode 11), then a hold in the same tick (R2).
// - exchange: one query + command frame per tick, `exchange_s` on the wire (3.5 ms in real time, 0 in lockstep).
//   The reply is what the boards held before this frame; the command applies after it (ideal PD tracking), so the
//   bus.command input is lagged by one exchange, as on the real bus.
// - the board watchdog: a board fed in position mode that goes `watchdog_s` (0.25 s) without a frame falls into
//   timeout mode 11 and ignores position commands until a stop.
// WHY no real bus here: kpi/ runs beside a powered arm; `twin = false` is refused, never opened (NO HARDWARE).
#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <like_keras/lk.hpp>
#include <like_keras/msgs/control.hpp>

namespace fake_bus {

inline constexpr std::uint32_t kMagic = 0x42464B4Cu;  // "LKFB" little-endian
inline constexpr std::uint32_t kVersion = 1;
inline constexpr std::size_t kBoards = 16;
inline constexpr std::size_t kRing = 65536;
inline constexpr std::int64_t kModeStopped = 0;
inline constexpr std::int64_t kModeFault = 1;
inline constexpr std::int64_t kModePosition = 10;
inline constexpr std::int64_t kModeTimeout = 11;
inline constexpr std::int32_t kEventQuery = 1;
inline constexpr std::int32_t kEventStop = 2;
inline constexpr std::int32_t kEventPosition = 3;

struct Board {
  double q;
  double qd;
  double last_fed;  // board time of the last position frame; 0 = not fed since take_over
  std::int64_t mode;
  std::int64_t fault;
  double reserved;
};

struct Event {
  double t;     // the bus's scheduled time (now())
  double wall;  // CLOCK_REALTIME when the frame went out: wall - t is the tick-start jitter the device saw
  double q0;    // board 0's position after the frame: what the arm was told, readable after the node is gone
  std::int32_t pid;
  std::int32_t kind;
  std::int32_t mode_after;  // board 0's mode after the frame
  std::int32_t reserved;
};

struct BoardFile {
  std::uint32_t magic;
  std::uint32_t version;
  std::uint32_t joints;
  std::uint32_t reserved;
  double watchdog_s;
  std::uint64_t events;
  std::uint64_t lapses;
  std::uint64_t reserved2;
  Board boards[kBoards];
  Event ring[kRing];
};
static_assert(offsetof(BoardFile, boards) == 48 && sizeof(Board) == 48 && sizeof(Event) == 40);
static_assert(offsetof(BoardFile, ring) == 48 + 48 * kBoards);

inline double wall_time() {
  timespec now{};
  clock_gettime(CLOCK_REALTIME, &now);
  return static_cast<double>(now.tv_sec) + static_cast<double>(now.tv_nsec) * 1e-9;
}

}  // namespace fake_bus

class MoteusBusCore {
 public:
  ~MoteusBusCore() { stop_feeding(); }

  // R1: the lock first, then the board file ("the port"). Throws (the activate refusal) before touching a board.
  template <class Params>
  void open_exclusive(const Params& params) {
    if (!params.twin) throw std::runtime_error("REFUSED: kpi has no real bus; set twin = true (the fake bus)");
    if (params.lock_path.empty()) throw std::runtime_error("REFUSED: the bus needs lock_path (BUS_LOCK, R1)");
    if (params.joints < 1 || params.joints > static_cast<int>(fake_bus::kBoards))
      throw std::runtime_error("REFUSED: joints must be 1-16");
    lock_fd_ = ::open(params.lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (lock_fd_ < 0) throw std::runtime_error("REFUSED: cannot open bus lock " + params.lock_path);
    if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      char holder[32] = {0};
      (void)!::pread(lock_fd_, holder, sizeof(holder) - 1, 0);
      ::close(lock_fd_);
      lock_fd_ = -1;
      throw std::runtime_error("REFUSED: bus lock " + params.lock_path + " held by pid " + holder);
    }
    const std::string pid = std::to_string(::getpid());
    (void)!::ftruncate(lock_fd_, 0);
    (void)!::pwrite(lock_fd_, pid.data(), pid.size(), 0);
    pid_ = static_cast<std::int32_t>(::getpid());
    joints_ = static_cast<std::size_t>(params.joints);
    exchange_s_ = params.exchange_s;
    if (params.board_file.empty()) {
      owned_ = std::make_unique<fake_bus::BoardFile>();
      file_ = owned_.get();
    } else {
      board_fd_ = ::open(params.board_file.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
      if (board_fd_ < 0 || ::ftruncate(board_fd_, sizeof(fake_bus::BoardFile)) != 0)
        throw std::runtime_error("REFUSED: cannot open board file " + params.board_file);
      void* mapped = ::mmap(nullptr, sizeof(fake_bus::BoardFile), PROT_READ | PROT_WRITE, MAP_SHARED, board_fd_, 0);
      if (mapped == MAP_FAILED) throw std::runtime_error("REFUSED: cannot map board file " + params.board_file);
      file_ = static_cast<fake_bus::BoardFile*>(mapped);
    }
    if (file_->magic != fake_bus::kMagic || file_->joints != joints_) {
      std::memset(static_cast<void*>(file_), 0, sizeof(fake_bus::BoardFile));
      file_->magic = fake_bus::kMagic;
      file_->version = fake_bus::kVersion;
      file_->joints = static_cast<std::uint32_t>(joints_);
    }
    file_->watchdog_s = params.watchdog_s;
    for (std::size_t joint = 0; joint < joints_; ++joint) names_[joint] = "j" + std::to_string(joint + 1);
  }

  // R2: never a bare stop. A stop only clears a latched watchdog, and the hold follows in the same tick.
  void take_over() {
    bool latched = false;
    for (std::size_t joint = 0; joint < joints_; ++joint) latched |= file_->boards[joint].mode == fake_bus::kModeTimeout;
    record(0.0, fake_bus::kEventQuery);
    if (latched) {
      for (std::size_t joint = 0; joint < joints_; ++joint) {
        file_->boards[joint].mode = fake_bus::kModeStopped;
        file_->boards[joint].qd = 0.0;
      }
      record(0.0, fake_bus::kEventStop);
    }
    hold_.size = joints_;
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      fake_bus::Board& board = file_->boards[joint];
      if (board.mode != fake_bus::kModeFault) board.mode = fake_bus::kModePosition;
      board.qd = 0.0;
      board.last_fed = 0.0;  // the watchdog counts from the first tick's frame
      hold_.name[joint] = names_[joint];
      hold_.position[joint] = board.q;
    }
    hold_.velocity.emplace();
    for (std::size_t joint = 0; joint < joints_; ++joint) (*hold_.velocity)[joint] = 0.0;
    record(0.0, fake_bus::kEventPosition);
  }

  // One frame: the reply (what the boards held) into `state`, then `command` (or the last one sent) applied.
  void exchange(double now, const rm::ServoCommand* command, rm::ServoState& state) {
    const double wall = fake_bus::wall_time();
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      fake_bus::Board& board = file_->boards[joint];
      if (board.mode == fake_bus::kModePosition && board.last_fed > 0.0 && now - board.last_fed > file_->watchdog_s) {
        board.mode = fake_bus::kModeTimeout;
        ++file_->lapses;
      }
    }
    state.size = joints_;
    state.velocity.emplace();
    state.fault.emplace();
    state.effort.reset();
    state.temperature.reset();
    state.voltage.reset();
    for (std::size_t joint = 0; joint < joints_; ++joint) {
      const fake_bus::Board& board = file_->boards[joint];
      state.name[joint] = names_[joint];
      state.position[joint] = board.q;
      (*state.velocity)[joint] = board.qd;
      (*state.fault)[joint] = board.fault;
    }
    state.seq = ++seq_;
    state.header.stamp = now;
    state.header.frame_id = std::string_view{};
    if (command != nullptr) hold_ = *command;  // the frame a missing command repeats (R5: a stale input never stops the feed)
    for (std::size_t joint = 0; joint < std::min(joints_, hold_.size); ++joint) {
      fake_bus::Board& board = file_->boards[joint];
      if (board.mode == fake_bus::kModeTimeout || board.mode == fake_bus::kModeFault) continue;
      board.mode = fake_bus::kModePosition;
      if (!std::isnan(hold_.position[joint])) board.q = hold_.position[joint];
      board.qd = hold_.velocity ? (*hold_.velocity)[joint] : 0.0;
      board.last_fed = now;
    }
    record(now, fake_bus::kEventPosition, wall);
    if (exchange_s_ > 0.0) {  // the frame's time on the wire; device emulation, never a test's sleep
      const timespec wire{0, static_cast<long>(exchange_s_ * 1e9)};
      ::clock_nanosleep(CLOCK_MONOTONIC, 0, &wire, nullptr);
    }
  }

  // Shutdown is to stop feeding: each board falls into its timeout mode on its own (R2). Never a stop frame.
  void stop_feeding() {
    if (board_fd_ >= 0) {
      ::munmap(file_, sizeof(fake_bus::BoardFile));
      ::close(board_fd_);
      board_fd_ = -1;
    }
    owned_.reset();
    file_ = nullptr;
    if (lock_fd_ >= 0) {
      ::close(lock_fd_);  // releases the flock
      lock_fd_ = -1;
    }
  }

  std::uint64_t lapses() const { return file_ ? file_->lapses : 0; }

 private:
  void record(double now, std::int32_t kind, double wall = fake_bus::wall_time()) {
    fake_bus::Event& event = file_->ring[file_->events % fake_bus::kRing];
    event = fake_bus::Event{now, wall, file_->boards[0].q, pid_, kind, static_cast<std::int32_t>(file_->boards[0].mode), 0};
    ++file_->events;
  }

  fake_bus::BoardFile* file_ = nullptr;
  std::unique_ptr<fake_bus::BoardFile> owned_;
  int lock_fd_ = -1;
  int board_fd_ = -1;
  std::int32_t pid_ = 0;
  std::size_t joints_ = 0;
  double exchange_s_ = 0.0;
  std::int64_t seq_ = 0;
  std::array<std::string, fake_bus::kBoards> names_{};
  rm::ServoCommand hold_{};
};
