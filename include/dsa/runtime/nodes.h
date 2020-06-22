#pragma once

#include <vector>
#include <utility>

#include "dsa/arch/ssinst.h"

namespace dsa {
namespace runtime {

struct Node;

struct Connect {
  int src_idx;
  int l, r;
  int dst_id;
  int dst_operand;
};

struct Packet {
  uint64_t available_at;
  uint64_t value;
};

struct Graph {
  uint64_t time{0};
  std::vector<Node*> nodes;
};

struct Node {
  int id;
  Graph *parent;
  std::vector<std::vector<std::pair<int, int>>> outs;
};

struct Switch : Node {
  std::vector<int> route;
};

struct ProcessingElement : Node {
  int num_inputs;
  int buffer_size;
  OpCode inst;
};

}
}