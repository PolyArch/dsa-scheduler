#include "dsa/dfg/node.h"

#include "dsa/debug.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

int Node::num_inc_edges() {
  int res = 0;
  for (auto& op : ops()) {
    res += op.edges.size();
  }
  return res;
}

int Node::slot_for_use(dsa::dfg::Edge* edge, int node_slot) {
  int slot = node_slot + edge->l / 8;
  CHECK(slot < 8);
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

Operand::Operand(SSDfg* parent, const std::vector<int>& es, OperandType type_)
    : parent(parent), edges(es), type(type_), fifos(es.size()) {
  for (auto eid : es) {
    auto& edge = parent->edges[eid];
    edge.val()->uses.push_back(eid);
  }
}

Operand::Operand(SSDfg *parent_, uint64_t imm_) :
  parent(parent_), imm(imm_), type(OperandType::data) {}

Operand::Operand(SSDfg *parent_, OperandType ty, uint64_t imm_) :
  parent(parent_), imm(imm_), type(ty) {}

Operand::Operand(SSDfg *parent_, int dtype, int idx) :
  parent(parent_), imm((((uint64_t) dtype) << 32) | idx), type(OperandType::local_reg) {
}

bool Operand::isImm() { return type == OperandType::data && edges.empty(); }

bool Operand::isReg() { return type == OperandType::local_reg && edges.empty(); }

bool Operand::ready() {
  if (edges.empty()) {
    return true;
  }
  for (size_t i = 0; i < fifos.size(); ++i) {
    auto* e = &parent->edges[edges[i]];
    if (fifos[i].empty()) {
      LOG(FORWARD) << "fifo (" << &fifos[i] << ") " << i << " no element!";
      return false;
    }
    if (e->use()->ssdfg()->cur_cycle() < fifos[i].front().available_at) {
      LOG(FORWARD) << "time away: " << e->use()->ssdfg()->cur_cycle() << " < "
                   << fifos[i].front().available_at;
      return false;
    }
  }
  return true;
}

uint64_t Operand::poll() {
  CHECK(ready());
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
  CHECK(ready());
  for (auto& elem : fifos) {
    elem.pop();
  }
}

bool Operand::predicate() {
  CHECK(ready());
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

Node* Edge::def() const { return parent->nodes[sid]; }

Value* Edge::val() const { return &parent->nodes[sid]->values[vid]; }

Node* Edge::use() const { return parent->nodes[uid]; }

Node* Edge::get(int x) const {
  return x ? use() : def();
}

std::string Edge::name() const {
  std::stringstream ss;
  ss << def()->name() << "." << val()->index << "[" << l << ", " << r << "]"
     << "->" << use()->name();
  return ss.str();
}

dsa::dfg::Node* Value::node() const { return parent->nodes[nid]; }

void Value::push(uint64_t val, bool valid, int delay) {
  // TODO: Support FIFO length backpressure.
  fifo.push(simulation::Data(parent->cur_cycle() + delay, val, valid));
}

bool Value::forward(bool attempt) {
  if (fifo.empty()) return false;
  simulation::Data data(fifo.front());
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
              simulation::Data entry(parent->cur_cycle() + edge->delay, data.value,
                                     data.valid);
              LOG(FORWARD) << parent->cur_cycle() << ": " << name() << " pushes "
                           << data.value << "(" << data.valid << ")"
                           << "to " << user->use()->name() << "'s " << j << "th operand "
                           << operand.fifos[i].size() + 1 << "/" << edge->buf_len
                           << " in " << edge->delay << " cycles(" << entry.available_at
                           << ")";
              if (!attempt) {
                LOG(FORWARD) << &fifo << " -> " << &operand.fifos[i];
              }
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
