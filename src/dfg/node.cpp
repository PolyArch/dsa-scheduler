#include "dsa/dfg/node.h"

#include "dsa/debug.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

int Node::slot_for_use(dsa::dfg::Edge* edge, int node_slot) {
  int slot = node_slot + edge->l / 8;
  DSA_CHECK(slot < 8);
  return slot;
}

bool Node::is_temporal() { return _ssdfg->meta[_group_id].is_temporal; }

Node::Node(SSDfg* ssdfg, V_TYPE v, const std::string& name)
    : _ssdfg(ssdfg), _name(name), _vtype(v) {
  _ID = ssdfg->nodes.size();
  _group_id = _ssdfg->meta.size() - 1;
}

dsa::dfg::Edge* Node::getLinkTowards(Node* to) {
  auto pred = [this, to](int eid) -> bool { return ssdfg()->edges[eid].use() == to; };
  for (auto& value : values) {
    auto& uses = value.uses;
    auto res = std::find_if(uses.begin(), uses.end(), pred);
    if (res != uses.end()) return &ssdfg()->edges[*res];
  }
  return nullptr;
}

/**
 * @brief Helper function to get the index of source edge to this node
 * @param sourceEdge Edge that may point to this node
 * @return int: index of this edge that points to this node
 *  -1 means that this edge does not point to this node
 */
int Node::sourceEdgeIdx(dsa::dfg::Edge* sourceEdge) {
  // Get the operands of this node
  std::vector<dsa::dfg::Operand> operands = ops();
  // Set the initial index of given edge
  int idx = -1;
  // Loop over all operands to see if edge id match
  for (int i = 0; i < operands.size(); ++i) {
    if (sourceEdge->id == operands[i].edges[0]) {
      idx = i;
      break;
    }
  }
  // return index found
  return idx;
}

// For each edge, the vid (valud id) is sinkEdgeIdx, so no helper function needed

Operand::Operand(SSDfg* parent, const std::vector<int>& es, OperandType type_)
    : parent(parent), edges(es), type(type_), fifos(es.size()) {
  for (auto eid : es) {
    auto& edge = parent->edges[eid];
    edge.val()->uses.push_back(eid);
  }
}

Operand::Operand(SSDfg* parent_, uint64_t imm_)
    : parent(parent_), imm(imm_), type(OperandType::data) {}

Operand::Operand(SSDfg* parent_, OperandType ty, uint64_t imm_)
    : parent(parent_), imm(imm_), type(ty) {}

Operand::Operand(SSDfg* parent_, int dtype, int idx)
    : parent(parent_),
      imm((((uint64_t)dtype) << 32) | idx),
      type(OperandType::local_reg) {}

bool Operand::isImm() { return type == OperandType::data && edges.empty(); }

bool Operand::isReg() { return type == OperandType::local_reg && edges.empty(); }

bool Operand::ready() {
  if (edges.empty()) {
    return true;
  }
  for (size_t i = 0; i < fifos.size(); ++i) {
    auto* e = &parent->edges[edges[i]];
    if (fifos[i].empty()) {
      DSA_LOG(FORWARD) << "fifo (" << e->name() << ") " << i << " no element!";
      return false;
    }
    if (e->use()->ssdfg()->cur_cycle() < fifos[i].front().available_at) {
      DSA_LOG(FORWARD) << "time away: " << e->use()->ssdfg()->cur_cycle() << " < "
                       << fifos[i].front().available_at;
      return false;
    }
  }
  return true;
}

uint64_t Operand::poll() {
  DSA_CHECK(ready());
  uint64_t res = 0;
  for (int i = fifos.size() - 1; i >= 0; --i) {
    auto* e = &parent->edges[edges[i]];
    uint64_t full = ~0ull >> (64 - e->bitwidth());
    uint64_t sliced = (fifos[i].front().value >> e->l) & full;
    res = (res << e->bitwidth()) | sliced;
  }
  return res;
}

void Operand::pop() {
  DSA_CHECK(ready());
  for (auto& elem : fifos) {
    elem.pop();
  }
}

bool Operand::predicate() {
  DSA_CHECK(ready());
  for (int i = fifos.size() - 1; i >= 0; --i) {
    if (!fifos[i].front().valid) {
      return false;
    }
  }
  return true;
}

Edge::Edge(SSDfg* parent, int sid, int vid, int uid, int l, int r)
    : sid(sid), vid(vid), uid(uid), parent(parent), l(l), r(r) {
  id = parent->edges.size();
}

Node* Edge::def() const {
  DSA_CHECK(sid >= 0 && sid < parent->nodes.size()) << sid << " " << parent->nodes.size();
  return parent->nodes[sid];
}

/**
 * @brief Helper function that tells whether this edge is stated edge defined by source
 * node
 * @return true: source node is input port and port is stated and vid of this edge is 0
 * @return false: Other cases
 */
bool Edge::sourceStated() {
  // Get the source node and cast it to input port
  dsa::dfg::InputPort* ip = dynamic_cast<dsa::dfg::InputPort*>(def());
  return ip != nullptr && ip->stated && vid == 0;
}

/**
 * @brief Helper function that tells whether this edge is stated edge defined by sink node
 * @return true: sink node is output port whose penetrate_state is not -1 and this edge is
 * first edge to this port
 * @return false: other case
 */
bool Edge::sinkStated() {
  // Get the sink node and cast it into output port
  dsa::dfg::OutputPort* op = dynamic_cast<dsa::dfg::OutputPort*>(use());
  // Get the source side index of this edge to output port
  int sourceEdgeIdx = op->sourceEdgeIdx(this);
  // Edge is stated defined by sink node
  // Sink node is output port, output port is penetratable, edge index to sink node is 0
  return op != nullptr && (op->penetrated_state >=0) && sourceEdgeIdx == 0;
}

Value* Edge::val() const {
  auto node = def();
  DSA_CHECK(vid < node->values.size()) << vid << " " << node->values.size();
  return &node->values[vid];
}

Node* Edge::use() const {
  DSA_CHECK(uid >= 0 && uid < parent->nodes.size());
  return parent->nodes[uid];
}

Node* Edge::get(int x) const { return x ? use() : def(); }

std::string Edge::name() const {
  std::stringstream ss;
  ss << def()->name() << "." << val()->index << "[" << l << ", " << r << "]"
     << "->" << use()->name();
  return ss.str();
}

dsa::dfg::Node* Value::node() const { return parent->nodes[nid]; }

void Value::push(uint64_t val, bool valid, int delay) {
  // TODO: Support FIFO length backpressure.
  fifo.push(sim::SpatialPacket(parent->cur_cycle() + delay, val, valid));
}

bool Value::forward(bool attempt) {
  if (fifo.empty()) return false;
  sim::SpatialPacket data(fifo.front());
  for (auto uid : uses) {
    int j = 0;
    auto* user = &parent->edges[uid];
    for (auto& operand : user->use()->ops()) {
      ++j;
      for (size_t i = 0; i < operand.edges.size(); ++i) {
        auto* edge = &parent->edges[operand.edges[i]];
        if (edge == user) {
          if ((int)operand.fifos[i].size() + 1 < edge->buf_len
              /*FIXME: The buffer size should be something more serious*/) {
            if (!attempt) {
              sim::SpatialPacket entry(data.available_at + edge->delay, data.value,
                                       data.valid);
              DSA_LOG(FORWARD) << parent->cur_cycle() << ": " << name() << " pushes "
                               << data.value << "(" << data.valid << ")"
                               << "to " << user->use()->name() << "'s " << j
                               << "th operand " << operand.fifos[i].size() + 1 << "/"
                               << edge->buf_len << " in " << edge->delay << " cycles("
                               << entry.available_at << ")";
              operand.fifos[i].push(entry);
            }
          } else {
            return false;
          }
        }
      }
    }
  }
  if (!attempt) {
    fifo.pop();
  }
  return true;
}

std::string Value::name() { return node()->name() + "." + std::to_string(index); }

}  // namespace dfg
}  // namespace dsa
