#include "dsa/debug.h"
#include "dsa/dfg/node.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

Operand::Operand(SSDfg *parent, const std::vector<int> &es, OperandType type_) :
  parent(parent), edges(es), type(type_), fifos(es.size()) {
  for (auto eid : es) {
    auto &edge = parent->edges[eid];
    edge.val()->uses.push_back(eid);
  }
}

Operand::Operand(uint64_t imm_) :
  parent(parent), imm(imm_), type(OperandType::data) {}

bool Operand::is_imm() { return edges.empty(); }

bool Operand::valid() {
  for (auto eid : edges) {
    auto *e = &parent->edges[eid];
    SSDfgNode* n = e->def();
    if (n->invalid()) return false;
  }
  return true;
}

bool Operand::ready() {
  if (edges.empty()) {
    return true;
  }
  for (size_t i = 0; i < fifos.size(); ++i) {
    auto *e = &parent->edges[edges[i]];
    if (fifos[i].empty()) {
      LOG(FORWARD) << "no element!";
      return false;
    }
    if (e->use()->ssdfg()->cur_cycle() < fifos[i].front().available_at) {
      LOG(FORWARD) << "time away: " << e->use()->ssdfg()->cur_cycle()
                     << " < " << fifos[i].front().available_at;
      return false;
    }
  }
  return true;
}

uint64_t Operand::poll() {
  assert(ready());
  uint64_t res = 0;
  for (int i = fifos.size() - 1; i >= 0; --i) {
    auto *e = &parent->edges[edges[i]];
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

Edge::Edge(SSDfg* parent, int sid, int vid, int uid, int l, int r) :
  sid(sid), vid(vid), uid(uid), parent(parent), l(l), r(r) {
  id = parent->edges.size();
}

SSDfgNode* Edge::def() const {
  return parent->nodes[sid];
}

Value* Edge::val() const {
  return &parent->nodes[sid]->values[vid];
}

SSDfgNode* Edge::use() const {
  return parent->nodes[uid];
}

SSDfgNode* Edge::get(int x) const {
  if (x == 0) return def();
  if (x == 1) return use();
  CHECK(false);
}

std::string Edge::name() const {
  std::stringstream ss;
  ss << def()->name() << "." << val()->index
     << "[" << l << ", " << r << "]"
     << "->" << use()->name();
  return ss.str();
}

SSDfgNode* Value::node() const {
  return parent->nodes[nid];
}

void Value::push(uint64_t val, bool valid, int delay) {
  // TODO: Support FIFO length backpressure.
  fifo.push(simulation::Data(parent->cur_cycle() + delay, val, valid));
}

bool Value::forward(bool attempt) {
  if (fifo.empty()) return false;
  simulation::Data data(fifo.front());
  for (auto uid : uses) {
    int j = 0;
    auto *user = &parent->edges[uid];
    for (auto& operand : user->use()->ops()) {
      ++j;
      for (size_t i = 0; i < operand.edges.size(); ++i) {
        auto *edge = &parent->edges[operand.edges[i]];
        if (edge == user) {
          if ((int)operand.fifos[i].size() + 1 < edge->buf_len
              /*FIXME: The buffer size should be something more serious*/) {
            if (!attempt) {
              simulation::Data entry(parent->cur_cycle() + edge->delay, data.value, data.valid);
              LOG(FORWARD) << parent->cur_cycle() << ": " << name()
                             << " pushes " << data.value << "(" << data.valid << ")" << "to "
                             << user->use()->name() << "'s " << j << "th operand "
                             << operand.fifos[i].size() + 1 << "/" << edge->buf_len
                             << " in " << edge->delay << " cycles(" << entry.available_at << ")";
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

std::string Value::name() {
  return node()->name() + "." + std::to_string(index);
}

}
}