#pragma once

#include "dsa/arch/sub_model.h"
#include "dsa/arch/visitor.h"

#define log2ceil(x) (63U - __builtin_clzl(static_cast<uint64_t>(x-1)) + 1)

namespace dsa {
namespace adg {
namespace bitstream {

// Entry of Control Lookup Table
struct ControlEntry{
  bool valid{false};
  bool operand0Reuse{false};
  bool operand1Reuse{false};
  bool resultDiscard{false};
  bool registerReset{false};
  bool abstain{false};
};
struct NodeInfo {
  // ---------- Switch Only ----------
  // output[i] --> input[j]
  std::map<int, int> outputRoute{{-1, -1}};

  // ---------- Processing Element Only ----------
  // ----- Per instruction info -----
  // Operand Route and Delay: operandIdx -> input/delay
  std::map<int, int> operandRoute{{-1, -1}};
  std::map<int, int> operandDelay{{-1, -1}};
  // Control Configuration
  int ctrlMode{0};
  int inputCtrlRoute{0};
  int opcode{0};
  // Result Output Route: result[i] -> output[j]
  std::map<int, int> resultOutRoute{{-1, -1}};
  std::map<int, int> resultRegRoute{{-1, -1}};
  // ----- Per control entry info -----
  std::vector<ControlEntry> ctrlLUT;
};

struct BitstreamWriter : Visitor {
  int nodeTypeBits = 2;
  int nodeIdBits = 8;
  int cfgGroupBits = 2;
  int cfgIndexBits = 4;
  int cfgInfoBits = 48;
  int totalBits = nodeTypeBits + nodeIdBits + cfgGroupBits + cfgIndexBits + cfgInfoBits;
  int nodeTypeLow = totalBits - nodeTypeBits;
  int nodeIdLow = nodeTypeLow - nodeIdBits;
  int cfgGroupLow = nodeIdLow - cfgGroupBits;
  int cfgIndexLow = cfgGroupLow - cfgIndexBits;
  // 768 means 16 * 48 bit configuration per group
  std::bitset<768> cfgInfoBitset{0};
  std::bitset<768> cfgInfoMask{0xFFFFFFFFFFFF};

  // Print Configuration Bitstream for Switch
  void Visit(ssswitch* sw) {
    // Calculate the selection bit for output port
    int selBits = log2ceil(sw->in_links().size() + 1);
    // Loop from highest output port to lowest port
    for(int outIdx = (sw->out_links().size() - 1); outIdx >= 0; outIdx --){
      for(int subIdx = ((sw->datawidth() / sw->granularity()) - 1); subIdx >= 0; subIdx --){
        // DSA_INFO << "Output Index = " << outIdx << ", subnet index = " << subIdx;
        cfgInfoBitset <<= selBits;
        // DSA_INFO << "cfgInfoBitset = " << cfgInfoBitset;
        // Get the routing selection from nodeInfo
        if(ni.outputRoute.count(outIdx)){
          // DSA_INFO << "Route " << (ni.outputRoute[outIdx] - 1) << " input port to output port " << outIdx;
          cfgInfoBitset |= ni.outputRoute[outIdx];
        }
      }
    }
    // Set the lowest bit to turn on the node
    if(cfgInfoBitset.any()){
      cfgInfoBitset = cfgInfoBitset << 1;
      cfgInfoBitset.set(0);
    }

    // Group the cfgInfoBitset by [[cfgInfoBits]]
    int cfgIdx = 0;
    while(cfgInfoBitset.any()){
      // DSA_INFO << cfgIdx << "th Routing Bitstream for " << sw->name();
      // Get the first configuration by taking the lower [[cfgInfoBits]]
      uint64_t currCfgInfo = (cfgInfoBitset & cfgInfoMask).to_ulong();
      currCfgInfo = currCfgInfo | ((uint64_t)1 << nodeTypeLow); // Write the node type
      currCfgInfo = currCfgInfo | ((uint64_t)(sw->localId()) << nodeIdLow); // Write the node local Id
      currCfgInfo = currCfgInfo | ((uint64_t)0 << cfgGroupLow); // Switch only have one configuration group
      currCfgInfo = currCfgInfo | ((uint64_t)cfgIdx << cfgIndexLow); // Set configuration index
      // Push it into the configuration bit vector
      configBitsVec.push_back(currCfgInfo);
      // Right Shift the Bitset and update the currCfgInfo]
      cfgInfoBitset >>= cfgInfoBits;
      cfgIdx++;
    }
  }

  // Print Configuration Bitstream for Switch
  void Visit(ssfu* fu) {
    // Encode the control LUT table
    // DSA_INFO << fu->name() << " generate bitstream for LUT";
    for(int entryIdx = fu->ctrlLUTSize(); entryIdx >=0 ; entryIdx--){
      cfgInfoBitset |= ni.ctrlLUT[entryIdx].abstain;
      cfgInfoBitset <<= 1;
      cfgInfoBitset |= ni.ctrlLUT[entryIdx].registerReset;
      cfgInfoBitset <<= 1;
      cfgInfoBitset |= ni.ctrlLUT[entryIdx].resultDiscard;
      cfgInfoBitset <<= 1;
      cfgInfoBitset |= ni.ctrlLUT[entryIdx].operand1Reuse;
      cfgInfoBitset <<= 1;
      cfgInfoBitset |= ni.ctrlLUT[entryIdx].operand0Reuse;
    }
    
    // Encode the instruction slot
    // DSA_INFO << fu->name() << " generate bitstream for instruction";
    for(int instIdx = fu->instSlotSize(); instIdx >= 0; instIdx--){
      // Write register selection bits
      if(fu->regFileSize() > 0){
        // Calculate the register selection bit
        int regSelBits = log2ceil(1 + fu->regFileSize());
        cfgInfoBitset <<= regSelBits;
        cfgInfoBitset |= ni.resultRegRoute[0];
      }
      // Write output selection bits
      int outSelBits = log2ceil(fu->out_links().size() + 1);
      cfgInfoBitset <<= outSelBits;
      cfgInfoBitset |= ni.resultOutRoute[0];
      // Write Opcode Selection
      int opcodeSelBits = log2ceil(fu->fu_type_.capability.size());
      cfgInfoBitset <<= opcodeSelBits;
      cfgInfoBitset |= ni.opcode;
      // Write the delay fifo cycle
      if(!fu->flow_control()){
        int delayCycleBits = log2ceil(fu->delay_fifo_depth() + 1);
        for(int operIdx = 1; operIdx >= 0; operIdx --){
          cfgInfoBitset <<= delayCycleBits;
          if(ni.operandDelay.count(operIdx))
            cfgInfoBitset |= ni.operandDelay[operIdx];
        }
      }
      // Write Controlled input selection bit
      if(fu->inputCtrl()){
        int inputCtrlSelBit = log2ceil(fu->in_links().size() + 1);
        cfgInfoBitset <<= inputCtrlSelBit;
        cfgInfoBitset |= ni.inputCtrlRoute;
      }
      // Write Controlled mode
      if(fu->inputCtrl() || fu->outputCtrl()){
        cfgInfoBitset <<= 2;
        cfgInfoBitset |= ni.ctrlMode;
      }
      // Write the Operand
      int operInputSelBits = log2ceil(fu->in_links().size() + 1);
      for(int operIdx = 1; operIdx >= 0; operIdx --){
        cfgInfoBitset <<= operInputSelBits;
        if(ni.operandRoute.count(operIdx))
          cfgInfoBitset |= ni.operandRoute[operIdx];
      }
      // Write the instruction valid, since we only have one, it is always valid
      if(cfgInfoBitset.any()){
        cfgInfoBitset <<= 1;
        cfgInfoBitset.set(0);
      }
    }
    // Enable the node
    if(cfgInfoBitset.any()){
      cfgInfoBitset <<= 1;
      cfgInfoBitset.set(0);
    }
    // Group the cfgInfoBitset by [[cfgInfoBits]]
    int cfgIdx = 0;
    while(cfgInfoBitset.any()){
      // Get the first configuration by taking the lower [[cfgInfoBits]]
      uint64_t currCfgInfo = (cfgInfoBitset & cfgInfoMask).to_ulong();
      currCfgInfo = currCfgInfo | ((uint64_t)0 << nodeTypeLow); // Write the node type
      currCfgInfo = currCfgInfo | ((uint64_t)(fu->localId()) << nodeIdLow); // Write the node local Id
      currCfgInfo = currCfgInfo | ((uint64_t)0 << cfgGroupLow); // Switch only have one configuration group
      currCfgInfo = currCfgInfo | ((uint64_t)cfgIdx << cfgIndexLow); // Set configuration index
      // Push it into the configuration bit vector
      configBitsVec.push_back(currCfgInfo);
      // Right Shift the Bitset and update the currCfgInfo]
      cfgInfoBitset >>= cfgInfoBits;
      cfgIdx++;
    }
  }

  std::vector<uint64_t> configBitsVec;
  NodeInfo& ni;

  BitstreamWriter(NodeInfo& ni) : ni(ni){}
};

}  // namespace bitstream
}  // namespace adg
}  // namespace dsa
