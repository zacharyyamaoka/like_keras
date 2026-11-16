// CONTRACT §15 A18 (consumer-003): SIGTERM and SIGHUP end a deployment the way SIGINT does, so every component's
// deactivate runs (the controller's hand-back) and its last log line is drained after it. This box's deactivate
// writes the file `path` and logs once. Driven by kpi/facilities/hand_back_child.py, ended by a signal.
#include <like_keras/lk.hpp>

#include <cstdio>
#include <string>

struct HandBack : lk::Component {
  struct Params {
    std::string path;
  };
  explicit HandBack(const Params& params) : params_(params) {}
  void forward() override {}
  void deactivate() override {
    if (std::FILE* file = std::fopen(params_.path.c_str(), "w")) {
      std::fputs("handed back\n", file);
      std::fclose(file);
    }
    log.info("handed back");
  }
  Params params_;
};
LK_PARAMS(HandBack::Params, path)

lk::System hand_back(const HandBack::Params& params) {
  lk::System s{"hand_back"};
  s.add<HandBack>("hand_back", params);
  return s;
}
LK_EXPORT(hand_back)
