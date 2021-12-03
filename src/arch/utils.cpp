#include "dsa/arch/utils.h"

#include <string>

#include "../utils/string_utils.h"
#include "dsa/core/singleton.h"
#include "json/json.h"

namespace dsa {

namespace adg {

// This is the ADG JSON Parser
using string_utils::String;

SpatialFabric* Import(std::string filename) {
  Json::CharReaderBuilder crb;
  std::ifstream ifs(filename);
  std::string errs;
  Json::Value* cgra = new Json::Value();
  Json::parseFromStream(crb, ifs, cgra, &errs);
  FILE* fjson = fopen(filename.c_str(), "r");
  auto res = new SpatialFabric();
  DSA_CHECK(cgra->isObject());

  // Create Global Node Id -> Node Table (legacy)
  // Should only be used for Legacy JSON File
  std::vector<ssnode*> sym_tab;
  
  // Create Local Node Id -> Node Table
  std::vector<ssnode*> fu_tab;
  std::vector<ssnode*> sw_tab;
  std::vector<ssnode*> ivp_tab;
  std::vector<ssnode*> ovp_tab;

  // Check whether compatiable ADG Mode is selected
  bool newVersionADG = !ContextFlags::Global().adg_compat;
  if (newVersionADG) {
    // Parse all DSAGen Nodes
    {
      // Get the node set from JSON object
      auto& jsonNodes = cgra->operator[](ADGKEY_NAMES[DSANODES]);
      auto sf = res;
      int globalNodeId = 0;

      // Check whether the DSAGenNodes are Object (nodeType+nodeId -> Parameter)
      DSA_CHECK(jsonNodes.isObject());
      fu_tab.resize(jsonNodes.size(), nullptr);
      sw_tab.resize(jsonNodes.size(), nullptr);
      ivp_tab.resize(jsonNodes.size(), nullptr);
      ovp_tab.resize(jsonNodes.size(), nullptr);

      // Loop over all nodes
      for (Json::Value::iterator dsaNode = jsonNodes.begin(); dsaNode != jsonNodes.end(); ++dsaNode) {
        // Get the name of the DSANode
        String nodeName(dsaNode.key().asString());
        // Split the name to get the node type and node local id
        std::vector<String> nodeTypeLocalId = nodeName.Split('.');
        // Get the nodeType from first String
        std::string nodeType = nodeTypeLocalId[0];
        // Get the local node from the second string
        std::string localNodeIdStr = nodeTypeLocalId[1];
        int localNodeId = stoi(localNodeIdStr);
        // Create different ssnode based on the node Type
        ssnode* node;
        DSA_INFO << "Create DSA Node: " << nodeType << "." << localNodeId;
        if (nodeType == ADGKEY_NAMES[PE_TYPE]) {
          // Get the node parameter first
          Json::Value nodeParam = (*dsaNode)[ADGKEY_NAMES[COMP_NODE]];
          // Get the node type and node Id from node parameter
          std::string nodeTypeParam = nodeParam[ADGKEY_NAMES[NODETYPE]].asString();
          // Node Type in Node Definition is different from the one in Node Name
          DSA_CHECK(nodeTypeParam == nodeType)
              << "Internal Node Type is " << nodeTypeParam
              << ", but node type from the node name is" << nodeType;
          int nodeIdParam = nodeParam[ADGKEY_NAMES[NODEID]].asInt();
          // Node ID from node parameter should be same as the one from Node Name
          DSA_CHECK(nodeIdParam == localNodeId);
          // Get the predication support
          bool predication = nodeParam[ADGKEY_NAMES[COMP_PRED]].asBool();
          // Get the compute bits
          int compBits = nodeParam[ADGKEY_NAMES[COMP_BITS]].asInt();
          // Get the compute unit bits
          int compUnitBits = nodeParam[ADGKEY_NAMES[COMP_UNITBITS]].asInt();
          // Get the node active support
          bool nodeActive = nodeParam[ADGKEY_NAMES[COMP_NODEACTIVE]].asBool();
          DSA_CHECK(compBits > 0 && compUnitBits > 0)
              << "Compute Bits = " << compBits
              << ", Compute Unit Bits = " << compUnitBits;

          // Get the output buffer parameter
          Json::Value buffParam = (*dsaNode)[ADGKEY_NAMES[COMP_OUTBUFF]];
          // Get the output buffer type
          bool staticOutBuff = buffParam[ADGKEY_NAMES[COMP_OUTBUFF_STATIC]].asBool();
          // Get the output buffer depth
          int outBuffDepth = buffParam[ADGKEY_NAMES[COMP_OUTBUFF_DEPTH]].asInt();

          // Get the Register File Parameter
          Json::Value regParam = (*dsaNode)[ADGKEY_NAMES[PE_REG]];
          // Get the register file size
          int regSize = regParam[ADGKEY_NAMES[PE_REG_SIZE]].asInt();
          // Get the register file type
          bool asyncRegFile = regParam[ADGKEY_NAMES[PE_REG_TYPE]].asBool();
          // Get whether the register file support update from configuration
          bool updateRegFile = regParam[ADGKEY_NAMES[PE_REG_UPD]].asBool();
          // Get the resetable register index
          Json::Value& resetRegParam = regParam[ADGKEY_NAMES[PE_REG_RESET]];
          DSA_CHECK(resetRegParam.isArray())
              << "FU" << localNodeId << ", reg size = " << regSize
              << ", reset reg type is " << resetRegParam.type();
          std::vector<int> resetableRegIdx(resetRegParam.size());
          for (int i = 0; i < resetRegParam.size(); i++) {
            resetableRegIdx.push_back(resetRegParam[i].asInt());
          }

          // Get the Controller Parameter
          Json::Value ctrlParam = (*dsaNode)[ADGKEY_NAMES[PE_CTRL]];
          // Get the control type that controller supported
          bool inputCtrl = ctrlParam[ADGKEY_NAMES[PE_CTRL_INPUT]].asBool();
          bool outputCtrl = ctrlParam[ADGKEY_NAMES[PE_CTRL_OUTPUT]].asBool();
          // Get the size of control lookup table
          int sizeLUT = ctrlParam[ADGKEY_NAMES[PE_CTRL_SIZE]].asInt();
          // Whether the control support operand reuse
          bool operReuse = ctrlParam[ADGKEY_NAMES[PE_CTRL_REUSE]].asBool();
          // Whether the control support result discard
          bool resDiscard = ctrlParam[ADGKEY_NAMES[PE_CTRL_DISCARD]].asBool();
          // Whether the control support register reset
          bool regReset = ctrlParam[ADGKEY_NAMES[PE_CTRL_RESET]].asBool();
          // Whether the control support abstain
          bool abstain = ctrlParam[ADGKEY_NAMES[PE_CTRL_ABSTAIN]].asBool();

          // Get the Function Unit Parameter
          Json::Value aluParam = (*dsaNode)[ADGKEY_NAMES[PE_OP]];
          // Get the operation firing type
          bool fuOpDynamic = aluParam[ADGKEY_NAMES[PE_OP_DYNA]].asBool();
          // The firing of the fu should share the concept of dynamic above.
          DSA_CHECK(fuOpDynamic == !staticOutBuff);
          // Get the operation max repeat time
          int fuOpMaxRepeat = aluParam[ADGKEY_NAMES[PE_OP_REPEAT]].asInt();
          // Get the defined max instruction latency (this is the manual latency, not the
          // natural one)
          int fuDefinedLatency = aluParam[ADGKEY_NAMES[PE_OP_DEFLAT]].asInt();
          // Get the instruction slot size
          int fuInstSlotSize = aluParam[ADGKEY_NAMES[PE_OP_INSTSIZE]].asInt();
          // Get the max delay fifo depth
          int fuDelayFifoDepth = aluParam[ADGKEY_NAMES[PE_OP_FIFODEP]].asInt();
          // Get the operation type and encoding to build function unit type
          auto& insts = aluParam[ADGKEY_NAMES[PE_OP_TYPEENC]];
          DSA_CHECK(insts.isArray());
          // Construct the name of FU
          std::string fuName = nodeType + "." + localNodeIdStr + " (Function Unit)";
          // Create Function Unit Capability, name it by using PE's name as prefix
          Capability fu_type(fuName);
          for (int instIdx = 0; instIdx < insts.size(); instIdx++) {
            // Get the opcode for this instruction
            std::string opName = insts[instIdx].asString();
            DSA_INFO << "Add Operation [ " << opName << " ] to " << fuName;
            fu_type.Add(dsa::inst_from_string(opName.c_str()), 1 /*instruction count*/);
          }

          // Not all of the parameter above is used, I will add them later in the ssfu
          // definition after submission
          ssfu* fu = new ssfu(compBits, compUnitBits, fuInstSlotSize, fuOpDynamic,
                              fuDelayFifoDepth, fu_type);

          // Set operation related parameter above
          fu->maxOpRepeat(fuOpMaxRepeat);
          fu->definedLatency(fuDefinedLatency);
          fu->instSlotSize(fuInstSlotSize);
          // Set register file related parameter above
          fu->regFileSize(regSize);
          fu->asyncReg(asyncRegFile);
          fu->updReg(updateRegFile);
          fu->resettableRegIdx(resetableRegIdx);
          // Set control related parameter above
          fu->inputCtrl(inputCtrl);
          fu->outputCtrl(outputCtrl);
          fu->ctrlLUTSize(sizeLUT);
          fu->operandReuse(operReuse);
          fu->resultDiscard(resDiscard);
          fu->registerReset(regReset);
          fu->abstain(abstain);

          // Add the node to spatial fabric
          node = fu;
          node->localId(localNodeId);
          sf->add_node(node);
          DSA_CHECK(globalNodeId == node->id());
          globalNodeId++;
          fu_tab[localNodeId] = node;
        } else if (nodeType == ADGKEY_NAMES[SW_TYPE]) {  // Create ssswitch
          // Get node parameter
          Json::Value nodeParam = (*dsaNode)[ADGKEY_NAMES[COMP_NODE]];
          // Get the node type and node Id from node parameter
          std::string nodeTypeParam = nodeParam[ADGKEY_NAMES[NODETYPE]].asString();
          // Node Type in Node Definition is different from the one in Node Name
          DSA_CHECK(nodeTypeParam == nodeType);
          int nodeIdParam = nodeParam[ADGKEY_NAMES[NODEID]].asInt();
          // Node ID from node parameter should be same as the one from Node Name
          DSA_CHECK(nodeIdParam == localNodeId);
          // Get the predication support
          bool predication = nodeParam[ADGKEY_NAMES[COMP_PRED]].asBool();
          // Get the compute bits
          int compBits = nodeParam[ADGKEY_NAMES[COMP_BITS]].asInt();
          // Get the compute unit bits
          int compUnitBits = nodeParam[ADGKEY_NAMES[COMP_UNITBITS]].asInt();
          // Get the node active support
          bool nodeActive = nodeParam[ADGKEY_NAMES[COMP_NODEACTIVE]].asBool();

          // Get the output buffer parameter
          Json::Value buffParam = (*dsaNode)[ADGKEY_NAMES[COMP_OUTBUFF]];
          // Get the output buffer type
          bool staticOutBuff = buffParam[ADGKEY_NAMES[COMP_OUTBUFF_STATIC]].asBool();
          // Get the output buffer depth
          int outBuffDepth = buffParam[ADGKEY_NAMES[COMP_OUTBUFF_DEPTH]].asInt();

          // Get the routing parameter
          Json::Value routeParam = (*dsaNode)[ADGKEY_NAMES[SW_ROUTE]];
          // TODO: the fine grain subnet routing connectivity matrix is not used in
          // scheduler let me add it after the submission
          node =
              new ssswitch(compBits, compUnitBits, 1 /*Switch cannot be shared now*/,
                           !staticOutBuff /*if static switch, outbuffer will be ignore*/,
                           outBuffDepth);
          // Add the node to spatial fabric
          node->localId(localNodeId);
          sf->add_node(node);
          DSA_CHECK(globalNodeId == node->id());
          globalNodeId++;
          sw_tab[localNodeId] = node;
        } else if (nodeType == ADGKEY_NAMES[IVP_TYPE]) {
          // Get node parameter
          Json::Value nodeParam = (*dsaNode)[ADGKEY_NAMES[IVP_NODE]];
          // Get the node type and node Id from node parameter
          std::string nodeTypeParam = nodeParam[ADGKEY_NAMES[NODETYPE]].asString();
          // Node Type in Node Definition is different from the one in Node Name
          DSA_CHECK(nodeTypeParam == nodeType)
              << "NodeType from Name = " << nodeType
              << ", node type in parameter = " << nodeTypeParam;
          int nodeIdParam = nodeParam[ADGKEY_NAMES[NODEID]].asInt();
          // Node ID from node parameter should be same as the one from Node Name
          DSA_CHECK(nodeIdParam == localNodeId);
          // Get the compute bits
          int compBits =
              64;  // defined by connected compute node, for now I just assume 64
          // Get the compute unit bits
          int compUnitBits = 8;  // defined by connected compute node, for now just 8
          // Get the suggest depth for the vector port
          int suggestDepth = nodeParam[ADGKEY_NAMES[IVP_SUGGEST_DEPTH]].asInt();
          // Create the vector port
          auto vp = new ssvport(compBits, compUnitBits,
                                /* max_util */ 1,
                                /* dynamic timing */ true,
                                /* fifo depth=*/suggestDepth);
          // Add the node to spatial fabric
          vp->port(localNodeId);
          vp->localId(localNodeId);
          node = vp;
          sf->add_node(node);
          DSA_CHECK(globalNodeId == node->id())
              << "Global ID = " << globalNodeId << ", node->id() = " << node->id();
          globalNodeId++;
          ivp_tab[localNodeId] = node;
        } else if (nodeType == ADGKEY_NAMES[OVP_TYPE]) {
          // Get node parameter
          Json::Value nodeParam = (*dsaNode)[ADGKEY_NAMES[OVP_NODE]];
          // Get the node type and node Id from node parameter
          std::string nodeTypeParam = nodeParam[ADGKEY_NAMES[NODETYPE]].asString();
          // Node Type in Node Definition is different from the one in Node Name
          DSA_CHECK(nodeTypeParam == nodeType)
              << "NodeType from Name = " << nodeType
              << ", node type in parameter = " << nodeTypeParam;
          int nodeIdParam = nodeParam[ADGKEY_NAMES[NODEID]].asInt();
          // Node ID from node parameter should be same as the one from Node Name
          DSA_CHECK(nodeIdParam == localNodeId);
          // Get the compute bits
          int compBits =
              64;  // defined by connected compute node, for now I just assume 64
          // Get the compute unit bits
          int compUnitBits = 8;  // defined by connected compute node, for now just 8
          // Get the suggest depth for the vector port
          int suggestDepth = nodeParam[ADGKEY_NAMES[OVP_SUGGEST_DEPTH]].asInt();
          // Create the vector port
          auto vp = new ssvport(compBits, compUnitBits,
                                /* max_util */ 1,
                                /* dynamic timing */ true,
                                /* fifo depth=*/suggestDepth);
          // Add the node to spatial fabric
          vp->port(localNodeId);
          vp->localId(localNodeId);
          node = vp;
          sf->add_node(node);
          DSA_CHECK(globalNodeId == node->id());
          globalNodeId++;
          ovp_tab[localNodeId] = node;
        } else if (nodeType.compare(ADGKEY_NAMES[DMA_TYPE]) == 0) {
          // Create Direct Memory Access
        } else if (nodeType.compare(ADGKEY_NAMES[SPM_TYPE]) == 0) {
          // Create Scratchpad
        } else if (nodeType.compare(ADGKEY_NAMES[REC_TYPE]) == 0) {
          // Create Recurrence Engine
        } else if (nodeType.compare(ADGKEY_NAMES[DIS_TYPE]) == 0) {
          // Create Discard Engine
        } else if (nodeType.compare(ADGKEY_NAMES[GEN_TYPE]) == 0) {
          // Create Generate Engine
        } else {
          // If reach here, it must be register engine
          assert(nodeType.compare(ADGKEY_NAMES[REG_TYPE]) == 0);
        }
      }  // End of loop over all nodes
      DSA_CHECK(fu_tab.size() > 0);
      DSA_CHECK(sw_tab.size() > 0);
      DSA_CHECK(ivp_tab.size() > 0);
      DSA_CHECK(ovp_tab.size() > 0);
    }  // End of Parsing nodes

    // Parse all DSAGen Links
    {
      auto& jsonLinks = cgra->operator[](ADGKEY_NAMES[DSAEDGES]);
      auto sf = res;
      DSA_CHECK(jsonLinks.isArray());
      for (int i = 0; i < jsonLinks.size(); ++i) {
        auto& dsaLink = jsonLinks[i];
        DSA_CHECK(dsaLink.isObject());
        // Get the Source and Sink Node Type
        std::string sourceNodeType = dsaLink[ADGKEY_NAMES[SOURCENODETYPE]].asString();
        std::string sinkNodeType = dsaLink[ADGKEY_NAMES[SINKNODETYPE]].asString();
        // Get the Souce and Sink Node Local ID
        int sourceNodeId = dsaLink[ADGKEY_NAMES[SOURCENODEID]].asInt();
        int sinkNodeId = dsaLink[ADGKEY_NAMES[SINKNODEID]].asInt();
        // Get the source and sink edge index
        int sourceIndex = dsaLink[ADGKEY_NAMES[SOURCEINDEX]].asInt();
        int sinkIndex = dsaLink[ADGKEY_NAMES[SINKINDEX]].asInt();

        // Get the source Module from the table
        ssnode* sourceModule;
        bool sourceDefined = false;
        if (sourceNodeType.compare(ADGKEY_NAMES[PE_TYPE]) == 0) {
          sourceModule = fu_tab[sourceNodeId];
          sourceDefined = true;
        } else if (sourceNodeType.compare(ADGKEY_NAMES[SW_TYPE]) == 0) {
          sourceModule = sw_tab[sourceNodeId];
          sourceDefined = true;
        } else if (sourceNodeType.compare(ADGKEY_NAMES[IVP_TYPE]) == 0) {
          sourceModule = ivp_tab[sourceNodeId];
          sourceDefined = true;
        }

        // Get the sink Module from the table
        ssnode* sinkModule;
        bool sinkDefined = false;
        if (sinkNodeType.compare(ADGKEY_NAMES[PE_TYPE]) == 0) {
          sinkModule = fu_tab[sinkNodeId];
          sinkDefined = true;
        } else if (sinkNodeType.compare(ADGKEY_NAMES[SW_TYPE]) == 0) {
          sinkModule = sw_tab[sinkNodeId];
          sinkDefined = true;
        } else if (sinkNodeType.compare(ADGKEY_NAMES[OVP_TYPE]) == 0) {
          sinkModule = ovp_tab[sinkNodeId];
          sinkDefined = true;
        }

        // Connect between node
        // TODO: since memory node is not included, so we only connect between IVP -> PE
        // <-> SW -> OVP I don't know why sourceModule && sinkModule not work, should it
        // be nullptr if it is memory node?
        if (sourceDefined && sinkDefined) {
          DSA_INFO << "Connect: " << sourceNodeType << "." << sourceNodeId << "["
                   << sourceIndex << "]"
                   << " --> " << sinkNodeType << "." << sinkNodeId << "[" << sinkIndex
                   << "]";
          sourceModule->add_link(sinkModule);
        }
      }  // End of loop over all links
    }    // End of parsing all json links
  } else {

    /** 
     * Parsing ADG by using the legacy ADG Format
     */
    auto& jsonNodes = cgra->operator[]("nodes");
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
        node = new ssswitch(cgranode["data_width"].asInt(),
                            cgranode["granularity"].asInt(), cgranode["max_util"].asInt(),
                            /*dynamic timing=*/true, /*fifo depth=*/2);
      } else if (nodeType == "processing element" || nodeType == "function unit") {
        DSA_CHECK(cgranode["instructions"].isArray());
        bool fu_array = cgranode["fu count"].isArray();
        // DSA_CHECK(cgranode["fu count"].isArray());
        auto& insts = cgranode["instructions"];
        // auto &fucnt = cgranode["fu count"];
        // DSA_CHECK(insts.size() == fucnt.size());
        Capability fu_type("fu_" + std::to_string(id));
        for (int i = 0; i < insts.size(); ++i) {
          auto str = insts[i].asString();
          auto cnt = fu_array ? cgranode["fu count"][i].asInt() : 1;
          auto opcode = dsa::inst_from_string(str.c_str());
          DSA_CHECK(opcode != SS_ERR) << str;
          fu_type.Add(opcode, cnt);
        }
        node = new ssfu(cgranode["data_width"].asInt(), cgranode["granularity"].asInt(),
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
        auto vp =
            new ssvport(cgranode["data_width"].asInt(), cgranode["granularity"].asInt(),
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

    // Go over all links
    {
      auto& jsonNodes = cgra->operator[]("links");
      auto sf = res;
      DSA_CHECK(jsonNodes.isArray());
      for (int i = 0; i < jsonNodes.size(); ++i) {
        auto& cgralink = jsonNodes[i];
        DSA_CHECK(cgralink.isObject());
        auto& source = cgralink["source"];
        DSA_CHECK(source.isArray());
        auto& sink = cgralink["sink"];
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
  }

  // Post Process and Sanity Check of Parsed Architecture
  res->post_process();
  ifs.close();
  return res;
}

}  // namespace adg
}  // namespace dsa
