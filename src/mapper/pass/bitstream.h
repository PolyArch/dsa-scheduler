#pragma once

#include "dsa/arch/sub_model.h"
#include "dsa/arch/visitor.h"

#define log2ceil(x) (63U - __builtin_clzl(static_cast<uint64_t>(x - 1)) + 1)

namespace dsa {
namespace adg {
namespace bitstream {

// Entry of Control Lookup Table
struct ControlEntry {
  bool valid{false};
  bool operand0Reuse{false};
  bool operand1Reuse{false};
  bool resultDiscard{false};
  bool registerReset{false};
  bool abstain{false};
};

// Struct that collects all node configuration
struct NodeInfo {
  
  /////////////////////////////
  /////////  Switch  //////////
  /////////////////////////////

  // Routing configuration per output port
  // TODO: subnet-level routing info is missing here
  std::map<int, int> outputRoute{{-1, -1}};

  //////////////////////////////
  //// Processing Element //////
  //////////////////////////////
  
  /*- Instruction: operation that applies to operands
    - Control lookup table: control behavior related to instruction
    - Control bitmask: select bit of interest to access control lookup table */
  
  // Input Ports / Registers -> Operands (zero means select null)
  std::map<int, int> operandRoute{{-1, -1}};
  // Delay of Operands for static PE (dynamic PE will not have this fields)
  std::map<int, int> operandDelay{{-1, -1}};
  // Control Mode: Self-controlled (self-produced result as control key); Input-controlled
  int ctrlMode{0};
  // Input Ports -> Input-controlled predicate (zero means select null)
  int inputCtrlRoute{0};
  // Opcode per instruction
  int opcode{0};
  // Results -> Output Ports
  std::map<int, int> resultOutRoute{{-1, -1}};
  // Results -> Registers
  std::map<int, int> resultRegRoute{{-1, -1}};
  
  // Vector of Control Lookup Table
  std::vector<ControlEntry> ctrlLUT;
  // Bitmask to filter out the bit of interests in predicate to access control LUT
  int bmss{0};

//////////////////////////////////////
/// Input/Output Vector Port  ////////
//////////////////////////////////////
/**
 * @brief Vector Port Configuration Info
 * - Mapping between physical port index and order in stream
 */

  // IVP : Physical Port Index -> Order in Stream; OVP : Order in Stream -> Physical Port Index
  std::map<int, int> vectorRoute{{-1, -1}};
};

// Enumeration for all node type that can be reconfigured
enum ReconfNodeType { PE_NODE_TYPE, SW_NODE_TYPE, IVP_NODE_TYPE, OVP_NODE_TYPE };

struct BitstreamWriter : Visitor {
  // The protocol of reconfiguration
  int nodeTypeBits  = 2;  // configBits[63:62]
  int nodeIdBits    = 8;  // configBits[61:54]
  int cfgGroupBits  = 2;  // configBits[53:52]
  int cfgIndexBits  = 4;  // configBits[51:48]
  int cfgInfoBits   = 48; // configBits[47: 0]

  // Total number of bits should be 64
  int totalBits = nodeTypeBits + nodeIdBits + cfgGroupBits + cfgIndexBits + cfgInfoBits;
  int nodeTypeLow = totalBits - nodeTypeBits;
  int nodeIdLow = nodeTypeLow - nodeIdBits;
  int cfgGroupLow = nodeIdLow - cfgGroupBits;
  int cfgIndexLow = cfgGroupLow - cfgIndexBits;
  // 768 means 16 * 48 bit configuration per group (which is the maximum bit for a reconfigurable node)
  std::bitset<768> cfgInfoBitset{0};
  std::bitset<768> cfgInfoMask{0xFFFFFFFFFFFF};

  // Print Configuration Bitstream for Switch
  void Visit(ssswitch* sw) {
    // Calculate the selection bit for output port
    int selBits = log2ceil(sw->in_links().size() + 1);
    // Loop from highest output port to lowest port
    for (int outIdx = (sw->out_links().size() - 1); outIdx >= 0; outIdx--) {
      // Loop from the highest subnet to the lowest subnet
      for (int subIdx = ((sw->datawidth() / sw->granularity()) - 1); subIdx >= 0;
           subIdx--) {
        // DSA_INFO << "Output Index = " << outIdx << ", subnet index = " << subIdx;
        cfgInfoBitset <<= selBits;
        // DSA_INFO << "cfgInfoBitset = " << cfgInfoBitset;
        // Get the routing selection from nodeInfo
        if (ni.outputRoute.count(outIdx)) {
          // DSA_INFO << "Route input port " << (ni.outputRoute[outIdx] - 1) << " to
          // output port " << outIdx;
          cfgInfoBitset |= ni.outputRoute[outIdx];
          // DSA_INFO << "cfgInfoBitset = " << cfgInfoBitset;
        }
      }
    }
    // Set the lowest bit to turn on the node
    if (cfgInfoBitset.any()) {
      cfgInfoBitset <<= 1;
      cfgInfoBitset.set(0);
    }

    // Group the cfgInfoBitset by [[cfgInfoBits]]
    int cfgIdx = 0;
    // Loop while there are still bits to be grouped
    while (cfgInfoBitset.any()) {
      // DSA_INFO << cfgIdx << "th Routing Bitstream for " << sw->name();
      // Get the first configuration by taking the lower [[cfgInfoBits]]
      uint64_t currCfgInfo = (cfgInfoBitset & cfgInfoMask).to_ulong();
      currCfgInfo |= ((uint64_t)SW_NODE_TYPE << nodeTypeLow);   // Write the node type
      currCfgInfo |= ((uint64_t)(sw->localId()) << nodeIdLow);  // Write the node local Id
      currCfgInfo |=
          ((uint64_t)0 << cfgGroupLow);  // Switch only have one configuration group
      currCfgInfo |= ((uint64_t)cfgIdx << cfgIndexLow);  // Set configuration index
      // Push it into the configuration bit vector
      configBitsVec.push_back(currCfgInfo);
      // Right Shift the Bitset and update the currCfgInfo]
      cfgInfoBitset >>= cfgInfoBits;
      cfgIdx++;
    }
  }

  // Print Configuration Bitstream for Vector Port
  void Visit(ssvport* vport) {
    // Generate bitstream for input vector port
    if (vport->isInputPort()) {
      // Calculate the selection bits for input vector port, +1 for selection of ground
      int selBits = log2ceil(vport->out_links().size() + 1);
      // DSA_INFO << vport->name() << " has " << vport->out_links().size() << " outputs";
      // DSA_INFO << vport->name() << " selection bits = " << selBits;
      // Loop from the highest physical port to lowest port
      for (int portIdx = (vport->out_links().size() - 1); portIdx >= 0; portIdx--) {
        // Left shift to make room for new selection bits
        cfgInfoBitset <<= selBits;
        if (ni.vectorRoute.count(portIdx)) {
          cfgInfoBitset |= ni.vectorRoute[portIdx];
        }
      }
    } else if (vport->isOutputPort()) {
      // Calculate the selection bits for output vector port, +1 for selection of ground
      int selBits = log2ceil(vport->in_links().size() + 1);
      // DSA_INFO << vport->name() << " has " << vport->in_links().size() << " inputs";
      // DSA_INFO << vport->name() << " selection bits = " << selBits;
      // Generate bitstream for output vector port
      for (int vecIdx = (vport->in_links().size() - 1); vecIdx >= 0; vecIdx--) {
        // Left shift to make room for new selection bits
        cfgInfoBitset <<= selBits;
        if (ni.vectorRoute.count(vecIdx)) {
          cfgInfoBitset |= ni.vectorRoute[vecIdx];
        }
      }
    } else {
      DSA_CHECK(false) << "The direction of vector port is problematic";
    }

    // Set the lower bit to enable the vector port
    if (cfgInfoBitset.any()) {
      cfgInfoBitset <<= 1;
      cfgInfoBitset.set(0);
    }

    // Group the configuration into by [[cfgInfoBits]]
    int cfgIdx = 0;
    // Loop while there are still bits to be grouped
    while (cfgInfoBitset.any()) {
      // Taking the lower bits of configuration info bitset
      uint64_t currCfgInfo = (cfgInfoBitset & cfgInfoMask).to_ulong();
      // Write the node type
      currCfgInfo |= ((uint64_t)(vport->isInputPort() ? IVP_NODE_TYPE : OVP_NODE_TYPE)
                      << nodeTypeLow);
      // Write the node local id
      currCfgInfo |= ((uint64_t)(vport->localId()) << nodeIdLow);
      // Set the configuration group index
      currCfgInfo |= ((uint64_t)0 << cfgGroupLow);
      // Set the configuration index
      currCfgInfo |= ((uint64_t)cfgIdx << cfgIndexLow);  // Set configuration index
      // Push it into the configuration bit vector
      configBitsVec.push_back(currCfgInfo);
      // Right shift the bitset to clear the grouped configuration info
      cfgInfoBitset >>= cfgInfoBits;
      cfgIdx++;
    }
  }

  // Print Configuration Bitstream for Function Unit
  void Visit(ssfu* fu) {

    // Encode the control LUT table
    if (ni.ctrlLUT.size() > 0) {
      // The Control LUT size in node info can be zero if this node is not used
      // If it is not zero, then it must be same as the one in parameter
      DSA_CHECK(ni.ctrlLUT.size() == fu->ctrlLUTSize())
          << "Size of LUT is " << ni.ctrlLUT.size() << ", but it is " << fu->ctrlLUTSize()
          << " in FU parameters";
      // Write BMSS to bitstream, the width of BMSS is fixed to 6 for now
      cfgInfoBitset <<= 6;
      cfgInfoBitset |= ni.bmss;
      for (int entryIdx = fu->ctrlLUTSize() - 1; entryIdx >= 0; entryIdx--) {
        // Abstain
        cfgInfoBitset <<= 1;
        cfgInfoBitset |= ni.ctrlLUT[entryIdx].abstain;
        // Register Reset
        cfgInfoBitset <<= 1;
        cfgInfoBitset |= ni.ctrlLUT[entryIdx].registerReset;
        // Result Discard
        cfgInfoBitset <<= 1;
        cfgInfoBitset |= ni.ctrlLUT[entryIdx].resultDiscard;
        // Operand Reuse
        cfgInfoBitset <<= 2;
        cfgInfoBitset |= ni.ctrlLUT[entryIdx].operand1Reuse;
        cfgInfoBitset |= ni.ctrlLUT[entryIdx].operand0Reuse;
        // Valid of control entry
        cfgInfoBitset <<= 1;
        cfgInfoBitset |= ni.ctrlLUT[entryIdx].valid;
        // Debug
        // DSA_INFO << fu->name() << "'s config bits for LUT's entry" << entryIdx;
      }
    }

    // Encode the instruction slot
    // DSA_INFO << fu->name() << " generate bitstream for instruction";
    for (int instIdx = fu->instSlotSize() - 1; instIdx >= 0; instIdx--) {
      // Write register selection bits
      if (fu->regFileSize() > 0) {
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
      if (!fu->flow_control()) {
        int delayCycleBits = log2ceil(fu->delay_fifo_depth() + 1);
        for (int operIdx = 1; operIdx >= 0; operIdx--) {
          cfgInfoBitset <<= delayCycleBits;
          if (ni.operandDelay.count(operIdx)) cfgInfoBitset |= ni.operandDelay[operIdx];
        }
      }
      // Write Controlled input selection bit
      if (fu->inputCtrl()) {
        int inputCtrlSelBit = log2ceil(fu->in_links().size() + 1);
        cfgInfoBitset <<= inputCtrlSelBit;
        cfgInfoBitset |= ni.inputCtrlRoute;
      }
      // Write Controlled mode
      if (fu->inputCtrl() || fu->outputCtrl()) {
        cfgInfoBitset <<= 2;
        cfgInfoBitset |= ni.ctrlMode;
      }
      // Write the Operand Selection (Input Ports / Register / Ground)
      int operInputSelBits = log2ceil(fu->in_links().size() + fu->regFileSize() + 1);
      for (int operIdx = 1; operIdx >= 0; operIdx--) {
        cfgInfoBitset <<= operInputSelBits;
        if (ni.operandRoute.count(operIdx)) cfgInfoBitset |= ni.operandRoute[operIdx];
      }
      // Write the instruction valid, since we only have one, it is always valid
      if (cfgInfoBitset.any()) {
        cfgInfoBitset <<= 1;
        cfgInfoBitset.set(0);
      }
      // Debug
      // DSA_INFO << fu->name() << "'s config bits for instSlot[" << instIdx << "]";
    }

    // Enable the node
    if (cfgInfoBitset.any()) {
      cfgInfoBitset <<= 1;
      cfgInfoBitset.set(0);
    }
    
    // Group the cfgInfoBitset by [[cfgInfoBits]]
    int cfgIdx = 0;
    while (cfgInfoBitset.any()) {
      // Debug
      // DSA_INFO << "Construct configuration bits for " << fu->name();
      // Get the first configuration by taking the lower [[cfgInfoBits]]
      uint64_t currCfgInfo = (cfgInfoBitset & cfgInfoMask).to_ulong();
      currCfgInfo |= ((uint64_t)PE_NODE_TYPE   << nodeTypeLow);  // Write the node type
      currCfgInfo |= ((uint64_t)(fu->localId())<< nodeIdLow);    // Write the node local Id
      currCfgInfo |= ((uint64_t)0              << cfgGroupLow);  // TODO: Reg Update Group (11) 2b add
      currCfgInfo |= ((uint64_t)cfgIdx         << cfgIndexLow);  // Set configuration index
      // Push it into the configuration bit vector
      configBitsVec.push_back(currCfgInfo);
      // Right Shift the Bitset and update the currCfgInfo]
      cfgInfoBitset >>= cfgInfoBits;
      cfgIdx++;
    }
  }

  std::vector<uint64_t> configBitsVec;
  NodeInfo& ni;

  BitstreamWriter(NodeInfo& ni) : ni(ni) {}
};

}  // namespace bitstream
}  // namespace adg
}  // namespace dsa
