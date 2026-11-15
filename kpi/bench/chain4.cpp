// Criterion 2's in-process number: a 4-component C++ chain (head -> link -> link -> tail) on one thread, the same
// work kpi/bench/bench_chain.cpp does by hand. The difference per tick, over 3 edges, is the cost of an edge.
#include <like_keras/lk.hpp>
#include <like_keras/msgs/sensor.hpp>

struct Head : lk::Component {
  lk::Out<rm::JointState> out{this, "out"};
  void forward() override {
    rm::JointState& state = out.write();
    state.size = 7;
    state.position[0] = now();
  }
};

struct Link : lk::Component {
  lk::In<rm::JointState> in{this, "in"};
  lk::Out<rm::JointState> out{this, "out"};
  void forward() override {
    const rm::JointState* state = in.get();
    if (state == nullptr) return;
    rm::JointState& next = out.write();
    next.size = state->size;
    next.position[0] = state->position[0] + 1.0;
  }
};

struct Tail : lk::Component {
  lk::In<rm::JointState> in{this, "in"};
  void forward() override {
    if (const rm::JointState* state = in.get()) sum += state->position[0];
  }
  double sum = 0.0;
};

struct ChainParams {
  int unused = 0;
};
LK_PARAMS(ChainParams, unused)

lk::System chain4(const ChainParams&) {
  lk::System s{"chain4"};
  auto& head = s.add<Head>("head");
  auto& first = s.add<Link>("first");
  auto& second = s.add<Link>("second");
  auto& tail = s.add<Tail>("tail");
  s.wire(first.in, head.out);
  s.wire(second.in, first.out);
  s.wire(tail.in, second.out);
  return s;
}
LK_EXPORT(chain4)
