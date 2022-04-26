#include "dsa/dfg/utils.h"

#include <algorithm>
#include <fstream>
#include <string>

#include "dsa/dfg/instruction.h"
#include "dsa/dfg/metadata.h"
#include "dsa/dfg/visitor.h"
#include "json/json.h"

namespace dsa {
namespace dfg {

struct Exporter : Visitor {
  Json::Value current;
  Json::Value nodes;

  void Visit(Node* node) override {
    current["id"] = node->id();
    current["temporal"] = node->is_temporal();
    current["group"] = node->group_id();
    current["name"] = node->name();
    // current["indirect"] = new json::Int(node->indirect());

    Json::Value inputs;
    for (auto operand : node->ops()) {
      Json::Value operand_obj;
      operand_obj["type"] = OPERAND_TYPE[(int)operand.type];
      if (operand.edges.empty()) { // inputs/outputs
        operand_obj["imm"] = operand.imm;
      } else {
        Json::Value edges;
        for (auto eid : operand.edges) {
          auto* edge = &node->ssdfg()->edges[eid];
          Json::Value edge_obj;
          edge_obj["id"] = edge->id;
          edge_obj["src_id"] = edge->def()->id();
          edge_obj["src_val"] = edge->val()->index;
          edge_obj["delay"] = edge->delay;
          edge_obj["l"] = edge->l;
          edge_obj["r"] = edge->r;
          edges.append(edge_obj);
        }
        operand_obj["edges"] = edges;
      }
      inputs.append(operand_obj);
    }
    current["inputs"] = inputs;
    nodes.append(current);
    current.clear();
  }
  void Visit(Instruction* inst) override {
    int opcode = inst->inst();
    current["op"] = inst->inst();
    current["inst"] = name_of_inst(inst->inst());
    auto f = [](const std::vector<int> &a) {
      Json::Value res(Json::ValueType::arrayValue);
      for (auto &elem : a) { res.append(elem); }
      return res;
    };
    current["ctrl"] = f(inst->predicate.encode());
    current["bmss"] = inst->predicate.bmss;
    current["self"] = f(inst->self_predicate.encode());
    Json::Value value_info(Json::ValueType::arrayValue);
    for (int i = 0; i < (int) inst->values.size(); ++i) {
      value_info.append(inst->values[i].reg);
      value_info.append(inst->values[i].symbol);
    }
    current["value_info"] = value_info;
    Visit(static_cast<Node*>(inst));
  }
  void Visit(InputPort *ip) override {
    current["stated"] = ip->stated;
    Visit(static_cast<VectorPort*>(ip));
  }
  void Visit(OutputPort *op) override {
    current["penetrate"] = op->penetrated_state;
    Visit(static_cast<VectorPort*>(op));
  }
  void Visit(VectorPort* vp) override {
    current["width"] = vp->bitwidth();
    current["lanes"] = vp->vectorLanes();
    Visit(static_cast<Node*>(vp));
  }
  void Visit(DMA* dma) override {
    current["type"] = "dma";
    Visit(static_cast<Array*>(dma));
  }
  void Visit(Scratchpad* sp) override {
    current["type"] = "spm";
    Visit(static_cast<Array*>(sp));
  }
  void Visit(Recurrance* rec) override {
    current["type"] = "rec";
    Visit(static_cast<Array*>(rec));
  }
  void Visit(Register* reg) override {
    current["type"] = "reg";
    Visit(static_cast<Array*>(reg));
  }
  void Visit(Generate* gen) override {
    current["type"] = "gen";
    Visit(static_cast<Array*>(gen));
  }
  void Visit(Array* arr) override {
    current["size"] = arr->size();
    Visit(static_cast<Node*>(arr));
  }
};

void Export(SSDfg* dfg, const std::string& fname) {
  Exporter exporter;
  dfg->Apply(&exporter);
  auto fcompare = [](Json::Value* a, Json::Value* b) {
    int ida = a->get("id", -1).asInt();
    int idb = b->get("id", -1).asInt();
    DSA_CHECK(ida != -1 && idb != -1);
    return ida < idb;
  };
  auto& nodes = exporter.nodes;
  // FIXME(@were): What if I do not sort here?
  // std::sort(nodes.begin(), nodes.end(), fcompare);
  for (int i = 0, n = nodes.size(); i < n; ++i) {
    DSA_CHECK(nodes[i]["id"].asInt() == i) << "id: " << nodes[i]["id"].asInt() << " != " << i;
  }

  Json::Value all_task_charac;
  bool any_prop=false;
  for(int task_type=0; task_type<NUM_GROUPS; ++task_type) {
    Json::Value current_prop;
    if(dfg->_task_type_characteristics[task_type].size()>0) {
      any_prop=true;
      // write in the below format
      current_prop["task_id"] = task_type;
      // task related information (it should print "type" first...)
      Json::Value task_prop;
      for(auto charac : dfg->_task_type_characteristics[task_type]) {
        task_prop[charac.first] = charac.second;
      }
      current_prop["task_characteristics"] = task_prop;
      exporter.nodes.append(current_prop);
      // all_task_charac.push_back(new json::Object(current_prop));
    }
  }

  Json::Value current;
  bool any_dep=false;
  for(int src_grp=0; src_grp<NUM_GROUPS; ++src_grp) {
    for(int dst_grp=0; dst_grp<NUM_GROUPS; ++dst_grp) {
      if(dfg->_dependence_characteristics[src_grp][dst_grp].size() > 0 ||
         dfg->_coalescer_dependence_characteristics[src_grp][dst_grp].size() > 0 ||
         dfg->_streaming_dependence_characteristics[src_grp][dst_grp].size() > 0) {
        any_dep=true;
        // write in the below format
        current["src_group"] = src_grp;
        current["dst_group"] = dst_grp;

        // coalescer related information (it should print "type" first...)
        if(1) { // dfg->_coalescer_dependence_characteristics[src_grp][dst_grp].size()>0) {
          Json::Value coal_map;
          for(auto charac : dfg->_coalescer_dependence_characteristics[src_grp][dst_grp]) {
            coal_map[charac.first] = charac.second;
          }
          current["coalescer_map_characteristics"] = coal_map;

          Json::Value coal_mapping;
          for(auto mapped_ports : dfg->_coalescer_dependence_maps[src_grp][dst_grp]) { // 2
            Json::Value map; // vector of mappings
            Json::Value src_ports;
            Json::Value dst_ports;
            src_ports.append(mapped_ports.first);
            dst_ports.append(mapped_ports.second);
            map["src_ports"] = src_ports;
            map["dst_ports"] = dst_ports;
            coal_mapping.append(map);
          }
          current["coalescer_mappings"] = coal_mapping;
        }

        if(1) { // dfg->_dependence_characteristics[src_grp][dst_grp].size()>0) {
          Json::Value map;
          for(auto charac : dfg->_dependence_characteristics[src_grp][dst_grp]) {
            map[charac.first] = charac.second;
          }
          current["map_characteristics"] = map;

          // argument related information
          Json::Value mapping;
          for(auto mapped_ports : dfg->_dependence_maps[src_grp][dst_grp]) { // 2
            Json::Value map; // vector of mappings
            Json::Value src_ports;
            Json::Value dst_ports;
            for(auto producer_port : mapped_ports.first) {
              src_ports.append(producer_port);
            }
            for(auto consumer_port : mapped_ports.second) {
              dst_ports.append(consumer_port);
            }
            map["src_ports"] = src_ports;
            map["dst_ports"] = dst_ports;
            mapping.append(map);
          }
          current["mappings"] = mapping;
        }

        // direct edge related information
        if(1) { // dfg->_streaming_dependence_characteristics[src_grp][dst_grp].size()>0) {
          Json::Value direct_map;
          for(auto charac : dfg->_streaming_dependence_characteristics[src_grp][dst_grp]) {
            std::cout << "[Export] First characteristic: " << charac.first << " second characteristic: " << charac.second << std::endl;
            direct_map[charac.first] = charac.second;
          }
          current["direct_map_characteristics"] = direct_map;

          // direct edge related information
          Json::Value direct_mapping;
          auto mapped_ports = dfg->_streaming_dependence_maps[src_grp][dst_grp];
          Json::Value map2; // vector of mappings
          Json::Value src_ports;
          Json::Value dst_ports;
          src_ports.append(mapped_ports.first);
          dst_ports.append(mapped_ports.second);
          map2["src_ports"] = src_ports;
          map2["dst_ports"] = dst_ports;
          direct_mapping.append(map2);
          current["direct_mappings"] = direct_mapping;
        }


        // push for each new entry, otherwise it was overwriting
        exporter.nodes.append(current);
      }
    }
  }

  std::ofstream ofs(fname);
  ofs << nodes;
}

SSDfg* Import(const std::string& s) {
  SSDfg* res = new SSDfg();
  MetaPort meta;

  Json::CharReaderBuilder crb;
  Json::Value *root = new Json::Value;
  std::string errs;
  std::ifstream ifs(s);
  Json::parseFromStream(crb, ifs, root, &errs);
  int last_group = -1;

  for (int i = 0, n = root->size(); i < n; ++i) {
    auto &node = (*root)[i];
    if(node.isMember("task_id")) { // a task dependence node
      int task_id = node["task_id"].asInt();
      res->create_new_task_type(task_id);
      // std::cout << "Creating new task type: " << task_id << " ";

      // FIXME: condition that type should come first; so we need to be in order
      auto &task_prop = node["task_characteristics"];
      for(int i=0; i<NUM_TASK_TYPE_CHARAC && !task_prop.empty(); ++i) {
        std::string arg_type = res->get_task_type_charac(i); // charac.first;
        std::string arg_value = task_prop[arg_type].asString();
        res->add_new_task_property(arg_type, arg_value);
        // std::cout << "Arg_type: " << arg_type << " arg_value: " << arg_value << "\n";
      }
      res->add_total_task_types();
      // std::cout << "Add new task id: " << task_id << " total task types: " << res->get_total_task_types() << std::endl;
    } else if(node.isMember("dst_group")) { // a task dependence node
      int src_grp = node["src_group"].asInt();
      int dst_grp = node["dst_group"].asInt();
      res->create_new_task_dependence_map(src_grp, dst_grp);

      // FIXME: condition that type should come first; so we need to be in order
      auto &coal_map_chars = node["coalescer_map_characteristics"];
      for(int i=0; i<NUM_TASK_DEP_CHARAC && !coal_map_chars.empty(); ++i) {
        std::string arg_type = res->get_task_dep_charac(i); // charac.first;
        std::string arg_value = coal_map_chars[arg_type].asString();
        res->add_new_task_dependence_characteristic(arg_type, arg_value);
      }

      auto &coal_mappings = node["coalescer_mappings"];
      for (int j = 0, m = coal_mappings.size(); j < m; ++j) { 
        auto &obj = coal_mappings[j];
        auto &src_ports = obj["src_ports"];
        auto &dst_ports = obj["dst_ports"];
        std::vector<std::string> producer_ports, consumer_ports;
        for(auto src : src_ports) {
          producer_ports.push_back(src.asString());
        }
        for(auto dst : dst_ports) {
          consumer_ports.push_back(dst.asString());
        }
        res->add_new_task_dependence_map(producer_ports, consumer_ports);
      }

      auto &direct_map_chars = node["direct_map_characteristics"];
      for(int i=0; i<NUM_TASK_DEP_CHARAC && !direct_map_chars.empty(); ++i) {
        std::string arg_type = res->get_task_dep_charac(i); // charac.first;
        std::string arg_value = direct_map_chars[arg_type].asString();
        res->add_new_task_dependence_characteristic(arg_type, arg_value);
      }
      auto &direct_mappings = node["direct_mappings"];
      for (int j = 0, m = direct_mappings.size(); j < m && !direct_map_chars.empty(); ++j) {
        // above could not be empty when this is fall??
        auto &obj = direct_mappings[j];
        auto &src_ports = obj["src_ports"];
        auto &dst_ports = obj["dst_ports"];
        std::vector<std::string> producer_ports, consumer_ports;
        for(auto src : src_ports) {
          producer_ports.push_back(src.asString());
        }
        for(auto dst : dst_ports) {
          consumer_ports.push_back(dst.asString());
        }
        res->add_new_task_dependence_map(producer_ports, consumer_ports);
      }

      auto &map_chars = node["map_characteristics"];
      for(int i = 0; i < NUM_TASK_DEP_CHARAC && !map_chars.empty(); ++i) {
        std::string arg_type = res->get_task_dep_charac(i); // charac.first;
        std::string arg_value = map_chars[arg_type].asString();
        res->add_new_task_dependence_characteristic(arg_type, arg_value);
      }

      auto &mappings = node["mappings"];
      for (int j = 0, m = mappings.size(); j < m; ++j) { 
        auto &obj = mappings[j];
        auto &src_ports = obj["src_ports"];
        auto &dst_ports = obj["dst_ports"];
        std::vector<std::string> producer_ports, consumer_ports;
        for(auto src : src_ports) {
          producer_ports.push_back(src.asString());
        }
        for(auto dst : dst_ports) {
          consumer_ports.push_back(dst.asString());
        }
        res->add_new_task_dependence_map(producer_ports, consumer_ports);
      }
    } else {
      auto& inputs = node["inputs"];
      // auto &indirect = *node["indirect"].asInt();
      auto group_id = node["group"].asInt();
      auto name = node["name"].asString();

      if (group_id != last_group) {
        res->meta.emplace_back();
        res->meta[group_id].is_temporal = node["temporal"].asInt();
        last_group = group_id;
      }
      if (node.isMember("width")) {
        int length = node["lanes"].asInt();
        int width = node["width"].asInt();
        if (node.isMember("stated")) {
          bool stated = node["stated"].asBool();
          res->emplace_back<InputPort>(length, width, name, res, meta, stated);
        } else if (node.isMember("penetrate")) {
          int penetrate = node["penetrate"].asInt();
          res->emplace_back<OutputPort>(length, width, name, res, meta, penetrate);
          res->nodes.back()->values.emplace_back(res, i, 0);
        } else {
          DSA_CHECK(false);
        }
      } else if (node.isMember("op")) {
        int opcode = node["op"].asInt();
        res->emplace_back<Instruction>(res, static_cast<OpCode>(opcode));
        auto& inst = res->instructions.back();
        auto opname = node["inst"].asString();
        DSA_CHECK(std::string(name_of_inst(inst.inst())) == opname)
            << name_of_inst(inst.inst()) << " != " << opname;
        auto f = [](Json::Value &v) {
          DSA_CHECK(v.isArray()) << v;
          std::vector<int> res;
          for (auto &elem : v) {
            res.push_back(elem.asInt());
          }
          return res;
        };
        if (node.isMember("ctrl")) {
          inst.predicate = CtrlBits(f(node["ctrl"]));
          inst.predicate.bmss = node["bmss"].asInt();
        }
        if (node.isMember("self")) {
          inst.self_predicate = CtrlBits(f(node["self"]));
        }
        auto &value_info = node["value_info"];
        for (int i = 0; i < (int) inst.values.size(); i += 2) {
          inst.values[i / 2].reg = value_info[i].asInt();
          inst.values[i / 2].symbol = value_info[i + 1].asString(); 
        }
      } else if (node.isMember("type")) {
        std::string type = node["type"].asString();
        int size = node["size"].asInt();
        if (type == "dma") {
          res->emplace_back<DMA>(size, name, res);
          res->nodes.back()->values.emplace_back(res, i, 0);
        } else if (type == "spm") {
          res->emplace_back<Scratchpad>(size, name, res);
          res->nodes.back()->values.emplace_back(res, i, 0);
        } else if (type ==  "rec") {
          res->emplace_back<Recurrance>(size, name, res);
          res->nodes.back()->values.emplace_back(res, i, 0);
        } else if (type == "gen") {
          res->emplace_back<Generate>(size, name, res);
          res->nodes.back()->values.emplace_back(res, i, 0);
        } else if (type == "reg") {
          res->emplace_back<Register>(size, name, res);
          res->nodes.back()->values.emplace_back(res, i, 0);
        } else {
          DSA_CHECK(false) << "Unknown type: " << type;
        }
      }
    }
  }

  for (int i = 0, n = root->size(); i < n; ++i) {
    auto &node = (*root)[i];
    if(node.isMember("task_id")) continue;
    if (node.isMember("dst_group")) continue;
    auto &operands = node["inputs"];
    for (int j = 0, m = operands.size(); j < m; ++j) {
      auto &obj = operands[j];
      auto type = obj["type"].asString();
      if (obj.isMember("imm")) {
        res->nodes[i]->ops().emplace_back(res, Str2Flag(type), obj["imm"].asInt64());
      } else {
        auto &edges = obj["edges"];
        std::vector<int> es;
        for (auto &edge : edges) {
          auto &edge_obj = edge;
          int src_id = edge_obj["src_id"].asInt();
          int src_val = edge_obj["src_val"].asInt();
          int delay = edge_obj["delay"].asInt();
          int l = edge_obj["l"].asInt();
          int r = edge_obj["r"].asInt();
          Edge e_instance(res, src_id, src_val, i, l, r);
          es.push_back(edge_obj["id"].asInt());
          if (es.back() >= res->edges.size()) res->edges.resize(es.back() + 1);
          e_instance.delay = delay;
          e_instance.id = es.back();
          res->edges[es.back()] = e_instance;
          //DSA_INFO << "Adding Edge" << e_instance.id << ": " << src_id << "(" << src_val << ")->" << i;

          if (e_instance.vid >= e_instance.def()->values.size()) {
            e_instance.def()->values.emplace_back(res, src_id, e_instance.vid);
          }
        }
        res->nodes[i]->ops().emplace_back(res, es, Str2Flag(type));
      }
    }
  }

  struct FIFOAllocator : Visitor {
    void Visit(Node* node) {
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
