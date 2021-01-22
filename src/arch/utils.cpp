#include "dsa/arch/utils.h"

#include <string>

#include "json.lex.h"
#include "json.tab.h"

namespace dsa {
namespace adg {

SpatialFabric* Import(std::string filename) {
  FILE* fjson = fopen(filename.c_str(), "r");
  struct params p;
  JSONrestart(fjson);
  JSONparse(&p);
  auto res = new SpatialFabric();
  auto cgra = p.data->As<plain::Object>();
  CHECK(cgra) << "CGRA root node not a JSON obj!";

  std::vector<ssnode*> sym_tab;
  {
    auto jsonNodes = cgra->operator[]("nodes");
    auto sf = res;
    // Vector Port Parameter
    int num_ivp = 0;
    int num_ovp = 0;  // Count the number of vector port
    int num_inputs = 0;
    int num_outputs = 0;  // Calculate the num of Input and Output in the fabric

    sym_tab.resize(jsonNodes->As<plain::Array>()->size(), nullptr);

    // Go over all nodes
    for (auto& jsonNode : *jsonNodes->As<plain::Array>()) {
      plain::Object cgranode = *jsonNode->As<plain::Object>();
      // Type and ID
      std::string nodeType = *cgranode["nodeType"]->As<std::string>();
      int id = *cgranode["id"]->As<int64_t>();
      ssnode* node;
      // Initialize Different Module
      if (nodeType == "switch") {
        node = new ssswitch(*cgranode["data_width"]->As<int64_t>(),
                            *cgranode["granularity"]->As<int64_t>(),
                            *cgranode["max_util"]->As<int64_t>(),
                            /*dynamic timing=*/true,
                            /*fifo depth=*/2);
      } else if (nodeType == "processing element" || nodeType == "function unit") {
        plain::Array insts = *cgranode["instructions"]->As<plain::Array>();
        plain::Array fucnt = *cgranode["fu count"]->As<plain::Array>();
        CHECK(insts.size() == fucnt.size());
        Capability fu_type("fu_" + std::to_string(id));
        for (int i = 0; i < insts.size(); ++i) {
          auto str = insts[i]->As<std::string>();
          auto cnt = fucnt[i]->As<int64_t>();
          CHECK(str) << "Instruction not a std::string!";
          CHECK(cnt) << "FU count not a int64_t!";
          fu_type.Add(dsa::inst_from_string(str->c_str()), *cnt);
        }
        node = new ssfu(*cgranode["data_width"]->As<int64_t>(),
                        *cgranode["granularity"]->As<int64_t>(),
                        *cgranode["max_util"]->As<int64_t>(),
                        /*dynamic timing=*/true,
                        /*fifo depth=*/2,
                        /*fu capability=*/fu_type);
      } else if (nodeType == "vector port") {
        bool is_input = false;
        int in_vec_width = *cgranode["num_input"]->As<int64_t>();
        int out_vec_width = *cgranode["num_output"]->As<int64_t>();
        int port_num = -1;
        // whether is a input/output vector port
        if (in_vec_width > 0) {
          is_input = false;
          port_num = num_ovp++;
          num_outputs += in_vec_width;
        } else if (out_vec_width > 0) {
          is_input = true;
          port_num = num_ivp++;
          num_inputs += out_vec_width;
        } else {
          continue;
        }
        auto vp = new ssvport(*cgranode["data_width"]->As<int64_t>(),
                              *cgranode["granularity"]->As<int64_t>(),
                              *cgranode["max_util"]->As<int64_t>(),
                              /*dynamic timing=*/true,
                              /*fifo depth=*/2);
        vp->port(port_num);
        node = vp;
      } else {
        CHECK(false) << id << "has unknown type" << nodeType << "\n";
      }
      sf->add_node(node);
      CHECK(id == node->id());
      sym_tab[id] = node;
    }
    CHECK(num_outputs > 0);
    CHECK(num_inputs > 0);
  }

  {
    auto jsonNodes = cgra->operator[]("links");
    auto sf = res;
    // Go over all links
    for (auto& jsonNode : *jsonNodes->As<plain::Array>()) {
      plain::Object cgralink = *jsonNode->As<plain::Object>();
      auto source = *cgralink["source"]->As<plain::Array>();
      auto sink = *cgralink["sink"]->As<plain::Array>();
      int source_id = *source[0]->As<int64_t>();
      int sink_id = *sink[0]->As<int64_t>();

      ssnode* from_module = sym_tab[source_id];
      ssnode* to_module = sym_tab[sink_id];
      CHECK(from_module && to_module);
      // connect
      from_module->add_link(to_module);
    }
  }

  res->post_process();
  fclose(fjson);
  delete p.data;
  return res;
}

}  // namespace adg
}  // namespace dsa