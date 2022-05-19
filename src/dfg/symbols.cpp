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
  DSA_CHECK(false) << "Unknown Qualifier: " << s;
  throw;
}

ControlEntry::ControlEntry(const std::string& s, ParseResult* controller_)
    : controller(controller_), raw() {
  flag = Str2Flag(s);
}

ControlEntry::ControlEntry(const std::string& s,
                           std::map<int, std::vector<std::string>>& bits_,
                           ParseResult* controller_, int bmss)
    : controller(controller_), raw(bits_), bmss(bmss) {
  flag = Str2Flag(s);
}

void UpdateNodeByArgs(Node* node, std::vector<ParseResult*>& args) {
  int iid = node->id();
  auto dfg = node->ssdfg();
  for (int i = 0, n = args.size(); i < n; ++i) {
    if (auto data = dynamic_cast<ConstDataEntry*>(args[i])) {
      node->ops().emplace_back(dfg, data->data);
    } else if (auto ve = dynamic_cast<ValueEntry*>(args[i])) {
      int startWidth = 0;
      for (auto operand : node->ops()) {
        

      }

      dfg->edges.emplace_back(dfg, ve->nid, ve->vid, iid, node->ops().size(), ve->l, ve->r, 0, ve->r - ve->l);
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
          dfg->edges.emplace_back(dfg, ne->nid, ne->vid, iid, 0, ne->l, ne->r, 0, ne->r - ne->l);
          es.push_back(dfg->edges.back().id);
        }
      }
      node->ops().emplace_back(dfg, es, OperandType::data);
    } else if (auto ce = dynamic_cast<ControlEntry*>(args[i])) {
      auto inst = dynamic_cast<Instruction*>(node);
      DSA_CHECK(inst);
      // External control
      if (ce->controller) {
        auto ne = dynamic_cast<ValueEntry*>(ce->controller);
        dfg->edges.emplace_back(dfg, ne->nid, ne->vid, iid, 0, ne->l, ne->r);
        inst->predicate = CtrlBits(ce->raw, ce->bmss);
        std::vector<int> es{dfg->edges.back().id};
        inst->ops().emplace_back(dfg, es, ce->flag);
      } else {
        // Self control
        inst->self_predicate = CtrlBits(ce->raw, -1);
      }
    } else if (auto re = dynamic_cast<RegisterEntry*>(args[i])) {
      auto inst = dynamic_cast<Instruction*>(node);
      DSA_CHECK(inst);
      inst->ops().emplace_back(dfg, node->bitwidth(), re->idx);
    } else {
      DSA_CHECK(false) << "Invalide Node type";
      throw;
    }
  }
}

TaskMapEntry::TaskMapEntry(ParseResult* controller_)
    : controller(controller_) {
  std::unordered_map<std::string, std::string> temp;
  port_map = temp;
}

SymbolTable::SymbolTable() {
  for (int i = 0; i < 8; ++i) {
    table_["$Reg" + std::to_string(i)] = new RegisterEntry(i);
  }
}

/*TaskMapEntry::TaskMapEntry(
            std::unordered_map<std::string, std::string>& port_map_,
            ParseResult* controller_)
    : controller(controller_), port_map(TaskPortMap(port_map_).mapping()) {
}*/
}  // namespace dfg
}  // namespace dsa
