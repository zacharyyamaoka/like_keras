// CONTRACT §15 A31 (consumer-014): a conversion may see its owner. `static Converted from(const T&, const Owner&)`,
// Owner being the component class that declares the In<T, Converted>; like_keras calls it on the pushing thread
// (A17), only between activate() and deactivate(), and the owner may only read what it set before activate()
// returned (its params, and config built there). This box is the consumer's plan admission written against that
// seam: each value is converted with its own box's gain (a param) and offset (built in activate), so two boxes of
// this .so in one process each convert with their own config, with no process-wide global (consumer-014's g_admission).
// It echoes (raw, converted, converted outside the window) for every value take() hands it; conversions made outside
// activate()..deactivate() are also counted in the file counts_path, mapped in activate, so ones after deactivate()
// count too. Driven by kpi/polyglot/test_polyglot_friction.py.
#include <fcntl.h>
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
enum Count : int { kConverted, kOutside, kCounts };  // int64 each in counts_path, in this order
}  // namespace

struct Scaler;

struct Scaled {
  double raw = 0.0;
  double converted = 0.0;
  bool outside = false;  // converted while its owner was not between activate() and deactivate()

  static Scaled from(const rm::JointState& message, const Scaler& owner);  // A31: the owner-aware form
};

struct Scaler : lk::Component {
  struct Params {
    double gain = 1.0;
    std::string counts_path;
  };
  lk::In<rm::JointState, Scaled> value{this, "value"};
  lk::Out<rm::JointState> echo{this, "echo"};

  explicit Scaler(const Params& params) : params_(params) {}

  void activate() override {
    offset_ = 1000.0 * params_.gain;  // config built in activate: what from() reads besides the params
    if (!params_.counts_path.empty()) {
      const int file = ::open(params_.counts_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
      if (file < 0 || ::ftruncate(file, kCounts * sizeof(std::int64_t)) != 0) {
        throw std::runtime_error("owner_conversion: cannot open counts_path");
      }
      void* memory = ::mmap(nullptr, kCounts * sizeof(std::int64_t), PROT_READ | PROT_WRITE, MAP_SHARED, file, 0);
      ::close(file);
      if (memory == MAP_FAILED) throw std::runtime_error("owner_conversion: cannot map counts_path");
      counts_.store(static_cast<std::int64_t*>(memory), std::memory_order_release);  // never unmapped
    }
    // WHY a pause: activate may block (§6.1); values pushed meanwhile must not be converted before it returns
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    active_.store(true, std::memory_order_release);  // the last thing activate does
  }

  void forward() override {
    const Scaled* scaled = value.take();
    if (scaled == nullptr) return;
    rm::JointState& out = echo.write();
    out.size = 3;
    out.name[0] = "raw";
    out.name[1] = "converted";
    out.name[2] = "outside";
    out.position[0] = scaled->raw;
    out.position[1] = scaled->converted;
    out.position[2] = scaled->outside ? 1.0 : 0.0;
    out.velocity.reset();
    out.effort.reset();
    out.header.stamp = now();
  }

  void deactivate() override { active_.store(false, std::memory_order_release); }  // the first thing it does

  void bump(Count which) const {
    if (std::int64_t* counts = counts_.load(std::memory_order_acquire)) {
      std::atomic_ref<std::int64_t>(counts[which]).fetch_add(1);
    }
  }

  Params params_;
  double offset_ = std::nan("");  // NaN until activate: a conversion that ran before it shows
  std::atomic<bool> active_{false};
  std::atomic<std::int64_t*> counts_{nullptr};
};
LK_PARAMS(Scaler::Params, gain, counts_path)

Scaled Scaled::from(const rm::JointState& message, const Scaler& owner) {
  Scaled scaled;
  scaled.outside = !owner.active_.load(std::memory_order_acquire);
  owner.bump(kConverted);
  if (scaled.outside) owner.bump(kOutside);
  scaled.raw = message.size > 0 ? message.position[0] : std::nan("");
  scaled.converted = scaled.raw * owner.params_.gain + owner.offset_;
  return scaled;
}

lk::System owner_conversion(const Scaler::Params& params) {
  lk::System s{"owner_conversion"};
  auto& scaler = s.add<Scaler>("scaler", params);
  s.expose(scaler.value, scaler.echo);
  return s;
}
LK_EXPORT(owner_conversion)
