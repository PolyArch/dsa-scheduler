#pragma once

#include "dsa/arch/sub_model.h"
#include "dsa/arch/visitor.h"

#define log2ceil(x) (63U - __builtin_clzl(static_cast<uint64_t>(x)) + 1)

namespace dsa {
namespace adg {
namespace bitstream {

struct NodeInfo {
  int route{-1};
  int operand{-1};
  int delay{-1};
};

struct BitstreamWriter : Visitor {
  void Visit(ssswitch* sw) {
    uint64_t left_bits = 64;
    // TODO: add shared config bits
    uint64_t num_config_index_bit = log2ceil(sw->max_util());
    // -- configure which configuration to write
    if (sw->max_util() > 1) {
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `config_index`;
    }
    // -- configure the current utility
    if (sw->max_util() > 1) {
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `curr_util`;
    }
    // -- config the ID
    DSA_CHECK(!sw->parent->node_list().empty()) << "number of node not parsed?";
    uint64_t num_id_bit = log2ceil((int)sw->parent->node_list().size());
    config_bits = config_bits << num_id_bit;
    left_bits -= num_id_bit;
    config_bits += sw->id();
    // -- config source select
    uint64_t num_source_sel_bit = log2ceil(sw->in_links().size() + 1);
    // + 1 is because zero means connect to ground
    for (uint output_idx = 0; output_idx < sw->out_links().size(); output_idx++) {
      config_bits = config_bits << num_source_sel_bit;
      left_bits -= num_source_sel_bit;
      if (ni.count(output_idx)) {
        config_bits += ni[output_idx].route + 1;
      }
    }
    // -- config offset for decomposability //TODO
    config_bits = config_bits << left_bits;
  }

  void Visit(ssfu* fu) {
    uint64_t left_bits = 64;
    // TODO: add shared config bits
    uint64_t num_config_index_bit = log2ceil(fu->max_util());
    // -- configure which configuration to write
    if (fu->max_util() > 1) {
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `config_index`;
    }
    // -- configure the current utility
    if (fu->max_util() > 1) {
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `curr_util`;
    }
    // -- config the ID
    DSA_CHECK(!fu->parent->node_list().empty()) << "number of node not parsed?";
    uint64_t num_id_bit = log2ceil((int)fu->parent->node_list().size());
    config_bits = config_bits << num_id_bit;
    left_bits -= num_id_bit;
    config_bits += fu->id();
    // -- config the operand source
    uint64_t num_operand_sel_bit = log2ceil(fu->in_links().size() + 1);
    // + 1 is because zero connect to ground
    for (uint operand_idx = 0; operand_idx < fu->fu_type_.max_num_operand;
         operand_idx++) {
      config_bits = config_bits << num_operand_sel_bit;
      left_bits -= num_operand_sel_bit;
      if (ni.count(operand_idx)) {
        config_bits += ni[operand_idx].operand + 1;
      }
    }
    // -- config the delay cycle
    uint64_t num_delay_sel_bit = log2ceil(fu->max_delay() + 1);
    // + 1 mean we allow zero cycle delay
    for (uint operand_idx = 0; operand_idx < fu->fu_type_.max_num_operand;
         operand_idx++) {
      config_bits = config_bits << num_delay_sel_bit;
      left_bits -= num_delay_sel_bit;
      if (ni.count(operand_idx)) {
        config_bits += ni[operand_idx].delay;
      }
    }
    // -- config opcode
    uint64_t num_opcode_bit =
        log2ceil(static_cast<uint64_t>(fu->fu_type_.capability.size()));
    config_bits = config_bits << num_opcode_bit;
    left_bits -= num_opcode_bit;
    config_bits += opcode;
    // -- config offset for decomposability //TODO
    // -- config output mode for power saving and shared mode
    config_bits = config_bits << left_bits;
  }

  uint64_t config_bits;
  std::map<int, NodeInfo>& ni;
  int opcode;

  BitstreamWriter(std::map<int, NodeInfo>& ni, int opcode) : ni(ni), opcode(opcode) {}
};

}  // namespace bitstream
}  // namespace adg
}  // namespace dsa