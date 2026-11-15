// Criterion 8 part 5: one C++ component, as a user writes it. <= 2 files, <= 10 lines naming the framework.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

struct Gain : lk::Component {
  struct Params {
    double k = 2.0;
  };
  lk::In<rm::JointState> input{this, "input"};
  lk::Out<rm::JointState> output{this, "output"};
  explicit Gain(const Params& params) : k_(params.k) {}
  void forward() override {
    const rm::JointState* in = input.get();
    if (in == nullptr) return;
    rm::JointState& out = output.write();
    out = *in;
    for (std::size_t joint = 0; joint < in->size; ++joint) out.position[joint] = k_ * in->position[joint];
  }
  double k_;
};
LK_PARAMS(Gain::Params, k)

lk::System gain(const Gain::Params& params) {
  lk::System s{"gain"};
  auto& stage = s.add<Gain>("gain", params);
  s.expose(stage.input, stage.output);
  return s;
}
LK_EXPORT(gain)
