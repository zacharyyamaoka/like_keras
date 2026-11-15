// The empty-step components of criterion 9 in C++: every facility must work with zero user lines.
#include <like_keras/lk.hpp>

struct Empty : lk::Component {
  struct Params {
    double gain = 1.0;
  };
  explicit Empty(const Params& params) : params_(params) {}
  void forward() override {}
  Params params_;
};
LK_PARAMS(Empty::Params, gain)

struct Talker : lk::Component {
  void forward() override { log.info("step %g", now()); }
};

lk::System empty_box(const Empty::Params& params) {
  lk::System s{"empty_box"};
  s.add<Empty>("empty", params);
  s.add<Talker>("talker");
  return s;
}
LK_EXPORT(empty_box)
