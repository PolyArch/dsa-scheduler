#include "dsa/dfg/utils.h"

#include <string>

#include "dsa/dfg/instruction.h"
#include "dsa/dfg/metadata.h"
#include "dsa/dfg/visitor.h"
#include "json.lex.h"
#include "json.tab.h"
#include "json/data.h"
#include "json/visitor.h"

namespace dsa {
namespace dfg {

struct Exporter : Visitor {
  plain::Object current;
  plain::Array nodes;

  void Visit(SSDfgNode* node) override {
    current["id"] = new json::Int(node->id());
    current["temporal"] = new json::Int(node->is_temporal());
    current["group"] = new json::Int(node->group_id());
    current["name"] = new json::String(node->name());

    plain::Array inputs;
    for (auto operand : node->ops()) {
      plain::Object operand_obj;
      if (operand.edges.empty()) {
        operand_obj["imm"] = new json::Int(operand.imm);
      } else {
        operand_obj["type"] = (new json::String(OPERAND_TYPE[(int)operand.type]));
        plain::Array edges;
        for (auto eid : operand.edges) {
          auto* edge = &node->ssdfg()->edges[eid];
          plain::Object edge_obj;
          edge_obj["id"] = new json::Int(edge->id);
          edge_obj["src_id"] = new json::Int(edge->def()->id());
          edge_obj["src_val"] = new json::Int(edge->val()->index);
          edge_obj["delay"] = new json::Int(edge->delay);
          edge_obj["l"] = new json::Int(edge->l);
          edge_obj["r"] = new json::Int(edge->r);
          edges.push_back(new json::Object(edge_obj));
        }
        operand_obj["edges"] = new json::Array(edges);
      }
      inputs.push_back(new json::Object(operand_obj));
    }
    current["inputs"] = new json::Array(inputs);
    nodes.push_back(new json::Object(current));
    current.clear();
  }
  void Visit(Instruction* inst) override {
    int opcode = inst->inst();
    current["op"] = new json::Int(inst->inst());
    current["inst"] = new json::String(name_of_inst(inst->inst()));
    current["ctrl"] = new json::Int(inst->predicate.bits());
    current["self"] = new json::Int(inst->self_predicate.bits());
    Visit(static_cast<SSDfgNode*>(inst));
  }
  void Visit(InputPort* in) override {
    current["width"] = new json::Int(in->get_port_width());
    current["length"] = new json::Int(in->get_vp_len());
    Visit(static_cast<SSDfgNode*>(in));
  }
  void Visit(OutputPort* out) override {
    current["width"] = new json::Int(out->get_port_width());
    current["length"] = new json::Int(out->get_vp_len());
    Visit(static_cast<SSDfgNode*>(out));
  }
};

void Export(SSDfg* dfg, const std::string& fname) {
  Exporter exporter;
  dfg->Apply(&exporter);
  auto fcompare = [](json::BaseNode* a, json::BaseNode* b) {
    auto oa = a->As<plain::Object>();
    auto ob = b->As<plain::Object>();
    CHECK(oa->count("id")) << "a has no id!";
    CHECK(ob->count("id")) << "b has no id!";
    auto ida = oa->operator[]("id")->As<int64_t>();
    auto idb = ob->operator[]("id")->As<int64_t>();
    CHECK(ida) << "The is of a is not an int!";
    CHECK(idb) << "The is of b is not an int!";
    return *ida < *idb;
  };
  auto& nodes = exporter.nodes;
  std::sort(nodes.begin(), nodes.end(), fcompare);
  for (int i = 0, n = nodes.size(); i < n; ++i) {
    auto elem_id = *nodes[i]->As<plain::Object>()->operator[]("id")->As<int64_t>();
    CHECK(elem_id == i) << elem_id << " != " << i;
  }

  json::Array json_array(exporter.nodes);
  std::ofstream ofs(fname);
  json::JSONPrinter printer(ofs);
  json_array.Accept(&printer);
}

SSDfg* Import(const std::string& s) {
  SSDfg* res = new SSDfg();
  MetaPort meta;

  FILE* fjson = fopen(s.c_str(), "r");
  struct params p;
  JSONrestart(fjson);
  JSONparse(&p);

  auto nodes = p.data->As<plain::Array>();
  CHECK(nodes);
  int last_group = -1;
  for (int i = 0, n = nodes->size(); i < n; ++i) {
    auto node_ptr = (*nodes)[i]->As<plain::Object>();
    CHECK(node_ptr);
    auto& node = *node_ptr;
    auto& inputs = *node["inputs"]->As<plain::Array>();
    auto& group_id = *node["group"]->As<int64_t>();
    auto& name = *node["name"]->As<std::string>();
    if (group_id != last_group) {
      res->start_new_dfg_group();
      res->group_prop(group_id).is_temporal = *node["temporal"]->As<int64_t>();
      last_group = group_id;
    }
    if (node.count("width")) {
      int length = *node["length"]->As<int64_t>();
      int width = *node["width"]->As<int64_t>();
      if (inputs.empty()) {
        res->emplace_back<InputPort>(length, width, name, res, meta);
      } else {
        res->emplace_back<OutputPort>(length, width, name, res, meta);
      }
    } else if (node.count("op")) {
      int opcode = *node["op"]->As<int64_t>();
      res->emplace_back<Instruction>(res, static_cast<OpCode>(opcode));
      auto& inst = res->instructions.back();
      auto& opname = *node["inst"]->As<std::string>();
      CHECK(std::string(name_of_inst(inst.inst())) == opname)
          << name_of_inst(inst.inst()) << " != " << opname;
      if (node.count("ctrl")) {
        uint64_t ctrl = *node["ctrl"]->As<int64_t>();
        inst.predicate = CtrlBits(ctrl);
      }
      if (node.count("self")) {
        uint64_t self = *node["self"]->As<int64_t>();
        inst.self_predicate = CtrlBits(self);
      }
    }
    auto& operands = *node["inputs"]->As<plain::Array>();
    for (int j = 0, m = operands.size(); j < m; ++j) {
      auto& obj = *operands[j]->As<plain::Object>();
      if (obj.count("imm")) {
        res->nodes[i]->ops().emplace_back(*obj["imm"]->As<int64_t>());
      } else {
        auto& type = *obj["type"]->As<std::string>();
        auto& edges = *obj["edges"]->As<plain::Array>();
        std::vector<int> es;
        for (auto edge : edges) {
          auto& edge_obj = *edge->As<plain::Object>();
          int src_id = *edge_obj["src_id"]->As<int64_t>();
          int src_val = *edge_obj["src_val"]->As<int64_t>();
          int delay = *edge_obj["delay"]->As<int64_t>();
          int l = *edge_obj["l"]->As<int64_t>();
          int r = *edge_obj["r"]->As<int64_t>();
          CHECK(src_id < i);
          Edge e_instance(res, src_id, src_val, i, l, r);
          es.push_back(*edge_obj["id"]->As<int64_t>());
          if (es.back() >= res->edges.size()) res->edges.resize(es.back() + 1);
          e_instance.delay = delay;
          e_instance.id = es.back();
          res->edges[es.back()] = e_instance;
        }
        res->nodes[i]->ops().emplace_back(res, es, Str2Flag(type));
      }
    }
  }

  fclose(fjson);
  delete p.data;

  struct FIFOAllocator : Visitor {
    void Visit(SSDfgNode* node) {
      for (auto& op : node->ops()) {
        op.fifos.resize(op.edges.size());
      }
    }
  } fa;
  res->Apply(&fa);

  return res;
}

}  // namespace dfg
}  // namespace dsa