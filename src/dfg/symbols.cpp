#include "dsa/dfg/symbols.h"

#include <vector>

#include "dsa/dfg/instruction.h"
#include "dsa/dfg/node.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

OperandType Str2Flag(const std::string& s) {
  for (int i = 0; i < (int)OperandType::unknown; ++i) {
    if (OPERAND_TYPE[i] == s) {
      return static_cast<OperandType>(i);
    }
  }
  CHECK(false) << "Unknown Qualifier: " << s;
  throw;
}

ControlEntry::ControlEntry(const std::string& s, ParseResult* controller_)
    : controller(controller_), bits(0) {
  flag = Str2Flag(s);
}

ControlEntry::ControlEntry(const std::string& s,
                           std::map<int, std::vector<std::string>>& bits_,
                           ParseResult* controller_)
    : controller(controller_), bits(CtrlBits(bits_).bits()) {
  flag = Str2Flag(s);
}

void UpdateNodeByArgs(Node* node, std::vector<ParseResult*>& args) {
  int iid = node->id();
  auto dfg = node->ssdfg();
  for (int i = 0, n = args.size(); i < n; ++i) {
    if (auto data = dynamic_cast<ConstDataEntry*>(args[i])) {
      node->ops().emplace_back(dfg, data->data);
    } else if (auto ve = dynamic_cast<ValueEntry*>(args[i])) {
      dfg->edges.emplace_back(dfg, ve->nid, ve->vid, iid, ve->l, ve->r);
      std::vector<int> es{dfg->edges.back().id};
      node->ops().emplace_back(dfg, es, OperandType::data);
    } else if (auto ne = dynamic_cast<NodeEntry*>(args[i])) {
      // TODO(@were): I am not sure if it is a good hack.
      auto operand = dfg->nodes[ne->nid];
      std::vector<ValueEntry*> ves;
      for (int j = 0, m = operand->values.size(); j < m; ++j) {
        ves.push_back(new ValueEntry(operand->id(), 0, 0, operand->bitwidth() - 1));
      }
      if (ves.size() == 1) {
        args[i] = ves[0];
      } else {
        auto ce = new ConvergeEntry();
        ce->entries = ves;
        args[i] = ce;
      }
      --i;
    } else if (auto ce = dynamic_cast<ConvergeEntry*>(args[i])) {
      std::vector<int> es;
      for (auto elem : ce->entries) {
        if (auto ne = dynamic_cast<ValueEntry*>(elem)) {
          dfg->edges.emplace_back(dfg, ne->nid, ne->vid, iid, ne->l, ne->r);
          es.push_back(dfg->edges.back().id);
        }
      }
      node->ops().emplace_back(dfg, es, OperandType::data);
    } else if (auto ce = dynamic_cast<ControlEntry*>(args[i])) {
      auto inst = dynamic_cast<Instruction*>(node);
      CHECK(inst);
      // External control
      if (ce->controller) {
        auto ne = dynamic_cast<ValueEntry*>(ce->controller);
        dfg->edges.emplace_back(dfg, ne->nid, ne->vid, iid, ne->l, ne->r);
        inst->predicate = ce->bits;
        std::vector<int> es{dfg->edges.back().id};
        inst->ops().emplace_back(dfg, es, ce->flag);
      } else {
        // Self control
        inst->self_predicate = ce->bits;
      }
    } else if (auto re = dynamic_cast<RegisterEntry*>(args[i])) {
      auto inst = dynamic_cast<Instruction*>(node);
      CHECK(inst);
      inst->ops().emplace_back(dfg, re->dtype, re->idx);
    } else {
      CHECK(false) << "Invalide Node type";
      throw;
    }
  }
}

TaskMapEntry::TaskMapEntry(ParseResult* controller_)
    : controller(controller_) {
  std::unordered_map<std::string, std::string> temp;
  port_map = temp;
}

RegisterEntry SymbolTable::isLocalRegister(const std::string &s) {
  if (s.find("Reg") == 0) {
    int dtype, idx;
    if (sscanf(s.c_str() + 3, "%d_%d", &dtype, &idx) == 2) {
      return RegisterEntry(dtype, idx);
    }
  }
  return {-1, 0};
}

/*TaskMapEntry::TaskMapEntry(
            std::unordered_map<std::string, std::string>& port_map_,
            ParseResult* controller_)
    : controller(controller_), port_map(TaskPortMap(port_map_).mapping()) {
}*/
}  // namespace dfg
}  // namespace dsa
