#include <string>

#include "json/json.h"
#include "dsa/arch/utils.h"


namespace dsa {
namespace adg {

SpatialFabric* Import(std::string filename) {
  Json::CharReaderBuilder crb;
  std::ifstream ifs(filename);
  std::string errs;
  Json::Value *cgra = new Json::Value();
  Json::parseFromStream(crb, ifs, cgra, &errs);
  FILE* fjson = fopen(filename.c_str(), "r");
  auto res = new SpatialFabric();
  DSA_CHECK(cgra->isObject());

  std::vector<ssnode*> sym_tab;
  {
    auto &jsonNodes = cgra->operator[]("nodes");
    auto sf = res;
    // Vector Port Parameter
    int num_ivp = 0;
    int num_ovp = 0;  // Count the number of vector port
    int num_inputs = 0;
    int num_outputs = 0;  // Calculate the num of Input and Output in the fabric

    DSA_CHECK(jsonNodes.isArray());
    sym_tab.resize(jsonNodes.size(), nullptr);

    // Go over all nodes
    for (int i = 0; i < jsonNodes.size(); ++i) {
      auto cgranode = jsonNodes[i];
      // Type and ID
      std::string nodeType = cgranode["nodeType"].asString();
      int id = cgranode["id"].asInt();
      ssnode* node;
      // Initialize Different Module
      if (nodeType == "switch") {
        node = new ssswitch(cgranode["data_width"].asInt(), cgranode["granularity"].asInt(),
                            cgranode["max_util"].asInt(), /*dynamic timing=*/true, /*fifo depth=*/2);
      } else if (nodeType == "processing element" || nodeType == "function unit") {
        DSA_CHECK(cgranode["instructions"].isArray());
        bool fu_array = cgranode["fu count"].isArray();
        // CHECK(cgranode["fu count"].isArray());
        auto &insts = cgranode["instructions"];
        // auto &fucnt = cgranode["fu count"];
        // CHECK(insts.size() == fucnt.size());
        Capability fu_type("fu_" + std::to_string(id));
        for (int i = 0; i < insts.size(); ++i) {
          auto str = insts[i].asString();
          auto cnt = fu_array ? cgranode["fu count"][i].asInt() : 1;
          fu_type.Add(dsa::inst_from_string(str.c_str()), cnt);
        }
        node = new ssfu(cgranode["data_width"].asInt(),
                        cgranode["granularity"].asInt(),
                        cgranode["max_util"].asInt(),
                        /*dynamic timing=*/true,
                        /*fifo depth=*/2,
                        /*fu capability=*/fu_type);
      } else if (nodeType == "vector port") {
        bool is_input = false;
        int in_vec_width = cgranode["num_input"].asInt();
        int out_vec_width = cgranode["num_output"].asInt();
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
        auto vp = new ssvport(cgranode["data_width"].asInt(),
                              cgranode["granularity"].asInt(),
                              cgranode["max_util"].asInt(),
                              /*dynamic timing=*/true,
                              /*fifo depth=*/2);
        vp->port(port_num);
        node = vp;
      } else {
        DSA_CHECK(false) << id << "has unknown type" << nodeType << "\n";
      }
      sf->add_node(node);
      DSA_CHECK(id == node->id());
      sym_tab[id] = node;
    }
    DSA_CHECK(num_outputs > 0);
    DSA_CHECK(num_inputs > 0);
  }

  {
    auto &jsonNodes = cgra->operator[]("links");
    auto sf = res;
    // Go over all links
    DSA_CHECK(jsonNodes.isArray());
    for (int i = 0; i < jsonNodes.size(); ++i) {
      auto &cgralink = jsonNodes[i];
      DSA_CHECK(cgralink.isObject());
      auto &source = cgralink["source"];
      DSA_CHECK(source.isArray());
      auto &sink = cgralink["sink"];
      DSA_CHECK(sink.isArray());
      int source_id = source[0].asInt();
      int sink_id = sink[0].asInt();

      ssnode* from_module = sym_tab[source_id];
      ssnode* to_module = sym_tab[sink_id];
      DSA_CHECK(from_module && to_module);
      // connect
      from_module->add_link(to_module);
    }
  }

  res->post_process();
  ifs.close();
  return res;
}

}  // namespace adg
}  // namespace dsa
