#include <string>

#include "dsa/dfg/metadata.h"
#include "dsa/dfg/utils.h"
#include "dsa/dfg/visitor.h"
#include "json/data.h"
#include "json/visitor.h"
#include "json.tab.h"
#include "json.lex.h"

namespace dsa {
namespace dfg {

struct Exporter : Visitor {
  plain::Object current;
  plain::Array nodes;

  void Visit(SSDfgNode *node) override {
    current["id"] = new json::Int(node->id());
    current["temporal"] = new json::Int(node->is_temporal());
    current["group"] = new json::Int(node->group_id());
    current["name"] = new json::String(node->name());
    current["indirect"] = new json::Int(node->indirect());

    plain::Array inputs;
    for (auto operand : node->ops()) {
      plain::Object operand_obj;
      if (operand.edges.empty()) { // inputs/outputs
        operand_obj["imm"] = new json::Int(operand.imm);
      } else { // instructions
        operand_obj["type"] = (new json::String(OPERAND_TYPE[(int) operand.type]));
        plain::Array edges;
        for (auto eid : operand.edges) {
          auto *edge = &node->ssdfg()->edges[eid];
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
  void Visit(SSDfgInst *inst) override {
    current["op"] = new json::Int(inst->inst());
    current["inst"] = new json::String(name_of_inst(inst->inst()));
    current["ctrl"] = new json::Int(inst->predicate.bits());
    current["self"] = new json::Int(inst->self_predicate.bits());
    Visit(static_cast<SSDfgNode*>(inst));
  }
  void Visit(SSDfgVecInput *in) override {
    current["width"] = new json::Int(in->get_port_width());
    current["length"] = new json::Int(in->get_vp_len());
    Visit(static_cast<SSDfgNode*>(in));
  }
  void Visit(SSDfgVecOutput *out) override {
    current["width"] = new json::Int(out->get_port_width());
    current["length"] = new json::Int(out->get_vp_len());
    Visit(static_cast<SSDfgNode*>(out));
  }
};

void Export(SSDfg *dfg, const std::string &fname) {
  Exporter exporter;
  dfg->Apply(&exporter);
  auto fcompare = [](json::BaseNode *a, json::BaseNode *b) {
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
  auto &nodes = exporter.nodes;
  std::sort(nodes.begin(), nodes.end(), fcompare);
  for (int i = 0, n = nodes.size(); i < n; ++i) {
    auto elem_id = *nodes[i]->As<plain::Object>()->operator[]("id")->As<int64_t>();
    CHECK(elem_id == i) << elem_id << " != " << i;
  }

  plain::Array all_task_charac;
  bool any_prop=false;
  for(int task_type=0; task_type<NUM_GROUPS; ++task_type) {
    plain::Object current_prop;
    if(dfg->_task_type_characteristics[task_type].size()>0) {
      any_prop=true;
      // write in the below format
      current_prop["task_id"] = new json::Int(task_type);
      // task related information (it should print "type" first...)
      plain::Object task_prop;
      for(auto charac : dfg->_task_type_characteristics[task_type]) {
        task_prop[charac.first] = new json::String(charac.second);
      }
      current_prop["task_characteristics"] = new json::Object(task_prop);
      exporter.nodes.push_back(new json::Object(current_prop));
      // all_task_charac.push_back(new json::Object(current_prop));
    }
  }

  plain::Object current;
  bool any_dep=false;
  for(int src_grp=0; src_grp<NUM_GROUPS; ++src_grp) {
    for(int dst_grp=0; dst_grp<NUM_GROUPS; ++dst_grp) {
      if(dfg->_dependence_characteristics[src_grp][dst_grp].size()>0 || dfg->_coalescer_dependence_characteristics[src_grp][dst_grp].size()>0 || dfg->_streaming_dependence_characteristics[src_grp][dst_grp].size()>0) {
        any_dep=true;
        // write in the below format
        current["src_group"] = new json::Int(src_grp);
        current["dst_group"] = new json::Int(dst_grp);

        // coalescer related information (it should print "type" first...)
        if(1) { // dfg->_coalescer_dependence_characteristics[src_grp][dst_grp].size()>0) {
          plain::Object coal_map;
          for(auto charac : dfg->_coalescer_dependence_characteristics[src_grp][dst_grp]) {
            coal_map[charac.first] = new json::String(charac.second);
          }
          current["coalescer_map_characteristics"] = new json::Object(coal_map);

          plain::Array coal_mapping;
          for(auto mapped_ports : dfg->_coalescer_dependence_maps[src_grp][dst_grp]) { // 2
            plain::Object map; // vector of mappings
            plain::Array src_ports;
            plain::Array dst_ports;
            src_ports.push_back(new json::String(mapped_ports.first));
            dst_ports.push_back(new json::String(mapped_ports.second));
            map["src_ports"] = new json::Array(src_ports);  
            map["dst_ports"] = new json::Array(dst_ports); 
            coal_mapping.push_back(new json::Object(map));
          }
          current["coalescer_mappings"] = new json::Array(coal_mapping);
        }

        if(1) { // dfg->_dependence_characteristics[src_grp][dst_grp].size()>0) {
          plain::Object map;
          for(auto charac : dfg->_dependence_characteristics[src_grp][dst_grp]) {
            map[charac.first] = new json::String(charac.second);
          }
          current["map_characteristics"] = new json::Object(map);

          // argument related information
          plain::Array mapping;
          for(auto mapped_ports : dfg->_dependence_maps[src_grp][dst_grp]) { // 2
            plain::Object map; // vector of mappings
            plain::Array src_ports;
            plain::Array dst_ports;
            for(auto producer_port : mapped_ports.first) {
              src_ports.push_back(new json::String(producer_port));
            }
            for(auto consumer_port : mapped_ports.second) {
              dst_ports.push_back(new json::String(consumer_port));
            }
            map["src_ports"] = new json::Array(src_ports);  
            map["dst_ports"] = new json::Array(dst_ports); 
            mapping.push_back(new json::Object(map));
          }
          current["mappings"] = new json::Array(mapping);
        }

        // direct edge related information
        if(1) { // dfg->_streaming_dependence_characteristics[src_grp][dst_grp].size()>0) {
          plain::Object direct_map;
          for(auto charac : dfg->_streaming_dependence_characteristics[src_grp][dst_grp]) {
            std::cout << "[Export] First characteristic: " << charac.first << " second characteristic: " << charac.second << std::endl;
            direct_map[charac.first] = new json::String(charac.second);
          }
          current["direct_map_characteristics"] = new json::Object(direct_map);

          // direct edge related information
          plain::Array direct_mapping;
          auto mapped_ports = dfg->_streaming_dependence_maps[src_grp][dst_grp];
          plain::Object map2; // vector of mappings
          plain::Array src_ports;
          plain::Array dst_ports;
          src_ports.push_back(new json::String(mapped_ports.first));
          dst_ports.push_back(new json::String(mapped_ports.second));
          map2["src_ports"] = new json::Array(src_ports);  
          map2["dst_ports"] = new json::Array(dst_ports); 
          direct_mapping.push_back(new json::Object(map2));
          current["direct_mappings"] = new json::Array(direct_mapping);
        }


        // push for each new entry, otherwise it was overwriting
        exporter.nodes.push_back(new json::Object(current));
      }
    }
  }

  json::Array json_array(exporter.nodes);
  std::ofstream ofs(fname);
  json::JSONPrinter printer(ofs);
  json_array.Accept(&printer);
}

SSDfg* Import(const std::string &s) {
  SSDfg* res = new SSDfg();
  MetaPort meta;

  FILE *fjson = fopen(s.c_str(), "r");
  struct params p;
  JSONrestart(fjson);
  JSONparse(&p);

  auto nodes = p.data->As<plain::Array>();
  CHECK(nodes);
  int last_group = -1;
  for (int i = 0, n = nodes->size(); i < n; ++i) {
    auto node_ptr = (*nodes)[i]->As<plain::Object>();
    CHECK(node_ptr);
    auto &node = *node_ptr;
    
    // if(node.count("taskflow")) { // a task dependence node
    if(node.count("task_id")) { // a task dependence node
      int task_id = *node["task_id"]->As<int64_t>();
      res->create_new_task_type(task_id);
      std::cout << "Creating new task type: " << task_id << " ";

      // FIXME: condition that type should come first; so we need to be in order
      auto &task_prop = *node["task_characteristics"]->As<plain::Object>();
      for(int i=0; i<NUM_TASK_TYPE_CHARAC && !task_prop.empty(); ++i) {
        std::string arg_type = res->get_task_type_charac(i); // charac.first;
        std::string arg_value = *task_prop[arg_type]->As<std::string>();
        res->add_new_task_property(arg_type, arg_value);
        std::cout << "Arg_type: " << arg_type << " arg_value: " << arg_value << "\n";
      }
      res->add_total_task_types();
      std::cout << "Add new task id: " << task_id << " total task types: " << res->get_total_task_types() << std::endl;
    } else if(node.count("dst_group")) { // a task dependence node
      int src_grp = *node["src_group"]->As<int64_t>();
      int dst_grp = *node["dst_group"]->As<int64_t>();
      res->create_new_task_dependence_map(src_grp, dst_grp);

      // FIXME: condition that type should come first; so we need to be in order
      auto &coal_map_chars = *node["coalescer_map_characteristics"]->As<plain::Object>();
      for(int i=0; i<NUM_TASK_DEP_CHARAC && !coal_map_chars.empty(); ++i) {
        std::string arg_type = res->get_task_dep_charac(i); // charac.first;
        std::string arg_value = *coal_map_chars[arg_type]->As<std::string>();
        res->add_new_task_dependence_characteristic(arg_type, arg_value);
      }

      auto &coal_mappings = *node["coalescer_mappings"]->As<plain::Array>();
      for (int j = 0, m = coal_mappings.size(); j < m; ++j) { 
        auto &obj = *coal_mappings[j]->As<plain::Object>();
        auto &src_ports = *obj["src_ports"]->As<plain::Array>();
        auto &dst_ports = *obj["dst_ports"]->As<plain::Array>();
        std::vector<std::string> producer_ports, consumer_ports;
        for(auto src : src_ports) {
          producer_ports.push_back(*(src->As<std::string>()));
        }
        for(auto dst : dst_ports) {
          consumer_ports.push_back(*(dst->As<std::string>()));
        }
        res->add_new_task_dependence_map(producer_ports, consumer_ports);
      }

      auto &direct_map_chars = *node["direct_map_characteristics"]->As<plain::Object>();
      for(int i=0; i<NUM_TASK_DEP_CHARAC && !direct_map_chars.empty(); ++i) {
        std::string arg_type = res->get_task_dep_charac(i); // charac.first;
        std::string arg_value = *direct_map_chars[arg_type]->As<std::string>();
        res->add_new_task_dependence_characteristic(arg_type, arg_value);
      }

      auto &direct_mappings = *node["direct_mappings"]->As<plain::Array>();
      for (int j = 0, m = direct_mappings.size(); j < m && !direct_map_chars.empty(); ++j) { // above could not be empty when this is fall??
        auto &obj = *direct_mappings[j]->As<plain::Object>();
        auto &src_ports = *obj["src_ports"]->As<plain::Array>();
        auto &dst_ports = *obj["dst_ports"]->As<plain::Array>();
        std::vector<std::string> producer_ports, consumer_ports;
        for(auto src : src_ports) {
          producer_ports.push_back(*(src->As<std::string>()));
        }
        for(auto dst : dst_ports) {
          consumer_ports.push_back(*(dst->As<std::string>()));
        }
        res->add_new_task_dependence_map(producer_ports, consumer_ports);
      }

      auto &map_chars = *node["map_characteristics"]->As<plain::Object>();
      for(int i=0; i<NUM_TASK_DEP_CHARAC && !map_chars.empty(); ++i) {
        std::string arg_type = res->get_task_dep_charac(i); // charac.first;
        std::string arg_value = *map_chars[arg_type]->As<std::string>();
        res->add_new_task_dependence_characteristic(arg_type, arg_value);
      }

      auto &mappings = *node["mappings"]->As<plain::Array>();
      for (int j = 0, m = mappings.size(); j < m; ++j) { 
        auto &obj = *mappings[j]->As<plain::Object>();
        auto &src_ports = *obj["src_ports"]->As<plain::Array>();
        auto &dst_ports = *obj["dst_ports"]->As<plain::Array>();
        std::vector<std::string> producer_ports, consumer_ports;
        for(auto src : src_ports) {
          producer_ports.push_back(*(src->As<std::string>()));
        }
        for(auto dst : dst_ports) {
          consumer_ports.push_back(*(dst->As<std::string>()));
        }
        res->add_new_task_dependence_map(producer_ports, consumer_ports);
      }
    } else {
      auto &inputs = *node["inputs"]->As<plain::Array>();
      auto &group_id = *node["group"]->As<int64_t>();
      auto &name = *node["name"]->As<std::string>();
      auto &indirect = *node["indirect"]->As<int64_t>();

      if (group_id != last_group) {
        res->start_new_dfg_group();
        res->group_prop(group_id).is_temporal = *node["temporal"]->As<int64_t>();
        last_group = group_id;
      }
      if (node.count("width")) {
        int length = *node["length"]->As<int64_t>();
        int width = *node["width"]->As<int64_t>();
        if (inputs.empty()) {
          res->emplace_back<SSDfgVecInput>(length, width, name, res, meta, indirect);
        } else {
          res->emplace_back<SSDfgVecOutput>(length, width, name, res, meta, indirect);
        }
      } else if (node.count("op")) {
        int opcode = *node["op"]->As<int64_t>();
        res->emplace_back<SSDfgInst>(res, static_cast<OpCode>(opcode));
        auto &inst = res->instructions.back();
        if (node.count("ctrl")) {
          uint64_t ctrl = *node["ctrl"]->As<int64_t>();
          inst.predicate = CtrlBits(ctrl);
        }
        if (node.count("self")) {
          uint64_t self = *node["self"]->As<int64_t>();
          inst.self_predicate = CtrlBits(self);
        }
      }
      auto &operands = *node["inputs"]->As<plain::Array>();
      for (int j = 0, m = operands.size(); j < m; ++j) {
        auto &obj = *operands[j]->As<plain::Object>();
        if (obj.count("imm")) {
          res->nodes[i]->ops().emplace_back(*obj["imm"]->As<int64_t>());
        } else {
          auto &type = *obj["type"]->As<std::string>();
          auto &edges = *obj["edges"]->As<plain::Array>();
          std::vector<int> es;
          for (auto edge : edges) {
            auto &edge_obj = *edge->As<plain::Object>();
            int src_id = *edge_obj["src_id"]->As<int64_t>();
            int src_val = *edge_obj["src_val"]->As<int64_t>();
            int delay = *edge_obj["delay"]->As<int64_t>();
            int l = *edge_obj["l"]->As<int64_t>();
            int r = *edge_obj["r"]->As<int64_t>();
            CHECK(src_id < i);
            Edge e_instance(res, src_id, src_val, i, l, r);
            es.push_back(*edge_obj["id"]->As<int64_t>());
            if (es.back() >= res->edges.size())
              res->edges.resize(es.back() + 1);
            e_instance.delay = delay;
            e_instance.id = es.back();
            res->edges[es.back()] = e_instance;
          }
          res->nodes[i]->ops().emplace_back(res, es, Str2Flag(type));
        }
      }
    }
  }

  fclose(fjson);
  delete p.data;

  struct FIFOAllocator : Visitor {
    void Visit(SSDfgNode *node) {
      for (auto &op : node->ops()) {
        op.fifos.resize(op.edges.size());
      }
    }
  } fa;
  res->Apply(&fa);
  return res;
}
}
}
