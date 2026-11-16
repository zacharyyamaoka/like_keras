// A static library compiled with default visibility, as the consumer's mcn_core was (consumer-013): every function
// and every std:: instantiation in it would be exported by a component .so that links it, without the version script
// lk_component() adds (CONTRACT §15 A28). Linked by links_static.cpp; checked by tests/test_component_exports.py.
#include <string>
#include <vector>

std::vector<std::string>& default_visibility_names() {
  static std::vector<std::string> names;
  return names;
}

int default_visibility_scaled(int value) {
  default_visibility_names().push_back(std::to_string(value));
  return 2 * value;
}
