// A component that links a static library of default visibility (default_visibility.cpp), built by lk_component():
// its .so must export the lk_* C ABI and nothing else (CONTRACT §15 A28; tests/test_component_exports.py).
#include <like_keras/lk.hpp>

int default_visibility_scaled(int value);

struct Scaler : lk::Component {
  void activate() override { scaled_ = default_visibility_scaled(21); }
  void forward() override {}
  int scaled_ = 0;
};

struct ScalerParams {
  int unused = 0;
};
LK_PARAMS(ScalerParams, unused)

lk::System links_static(const ScalerParams&) {
  lk::System s{"links_static"};
  s.add<Scaler>("scaler");
  return s;
}
LK_EXPORT(links_static)
