// The ruler's C++ reader of an LKF frame (CONTRACT §7.5): an ArrowArray tree whose buffers point into the frame.
// The node layout (length, buffer count, child count, depth first) comes from the `.nodes` file make_golden.py
// writes beside the frame, so this needs no schema and no Arrow library (kpi/alloc_probe.cpp uses it).
#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef ARROW_C_DATA_INTERFACE
#define ARROW_C_DATA_INTERFACE
extern "C" {
struct ArrowArray {
  int64_t length;
  int64_t null_count;
  int64_t offset;
  int64_t n_buffers;
  int64_t n_children;
  const void** buffers;
  struct ArrowArray** children;
  struct ArrowArray* dictionary;
  void (*release)(struct ArrowArray*);
  void* private_data;
};
}
#endif

namespace kpi {

struct Frame {
  std::vector<unsigned char> bytes;
  std::vector<const void*> buffers;     // every buffer, depth first
  std::vector<ArrowArray> arrays;       // every node, depth first
  std::vector<std::vector<ArrowArray*>> children;
  std::vector<std::vector<const void*>> node_buffers;
  int released = 0;                     // how many times the consumer released the root
};

inline std::vector<unsigned char> read_file(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("cannot read " + path);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

// Load `path` (an LKF frame) and `path` with .lkf replaced by .nodes. The root's release only counts: the frame
// owns the memory, so a consumer releasing it on any thread never frees or allocates.
inline std::unique_ptr<Frame> load_frame(const std::string& path) {
  auto frame = std::make_unique<Frame>();
  frame->bytes = read_file(path);
  const unsigned char* data = frame->bytes.data();
  if (frame->bytes.size() < 24 || std::memcmp(data, "LKF1", 4) != 0) throw std::runtime_error("not an LKF frame");
  uint32_t count = 0;
  std::memcpy(&count, data + 4, 4);
  std::size_t cursor = 24 + 8 * static_cast<std::size_t>(count);
  for (uint32_t index = 0; index < count; ++index) {
    uint64_t size = 0;
    std::memcpy(&size, data + 24 + 8 * index, 8);
    frame->buffers.push_back(size == 0 ? nullptr : data + cursor);
    cursor += size + ((8 - size % 8) % 8);
  }
  std::ifstream nodes(path.substr(0, path.size() - 4) + ".nodes");
  struct Node { int64_t length, buffers, children; };
  std::vector<Node> layout;
  for (Node node{}; nodes >> node.length >> node.buffers >> node.children;) layout.push_back(node);
  frame->arrays.resize(layout.size());
  frame->children.resize(layout.size());
  frame->node_buffers.resize(layout.size());
  std::size_t next_buffer = 0, next_node = 0;
  // Depth first: a node's buffers, then its children (each a full subtree).
  struct Builder {
    Frame& frame;
    std::vector<Node>& layout;
    std::size_t& next_buffer;
    std::size_t& next_node;
    ArrowArray* build() {
      const std::size_t index = next_node++;
      const Node node = layout.at(index);
      for (int64_t b = 0; b < node.buffers; ++b) frame.node_buffers[index].push_back(frame.buffers.at(next_buffer++));
      for (int64_t c = 0; c < node.children; ++c) frame.children[index].push_back(build());
      ArrowArray& array = frame.arrays[index];
      array.length = node.length;
      array.null_count = frame.node_buffers[index][0] == nullptr ? 0 : -1;
      array.offset = 0;
      array.n_buffers = node.buffers;
      array.n_children = node.children;
      array.buffers = frame.node_buffers[index].data();
      array.children = frame.children[index].empty() ? nullptr : frame.children[index].data();
      array.dictionary = nullptr;
      array.release = [](ArrowArray* self) { self->release = nullptr; };
      array.private_data = &frame;
      return &array;
    }
  } builder{*frame, layout, next_buffer, next_node};
  builder.build();
  frame->arrays[0].release = [](ArrowArray* self) {
    ++static_cast<Frame*>(self->private_data)->released;
    self->release = nullptr;
  };
  return frame;
}

}  // namespace kpi
