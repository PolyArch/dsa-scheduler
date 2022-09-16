#pragma once

#include <ostream>

#include "dsa/arch/sub_model.h"
#include "dsa/mapper/dse.h"
#include "dsa/mapper/schedule.h"
#include "dsa/arch/visitor.h"
#include "dsa/dfg/node.h"
#include "dsa/dfg/visitor.h"
#include "dsa/arch/utils.h"


namespace dsa {
namespace adg {
  

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

struct JsonWriter : dsa::adg::Visitor {  
  std::ostream& os;

  JsonWriter(std::ostream& os_) : os(os_) {}

  void Visit(ssswitch* sw) override {
    os << "\"" << ADGKEY_NAMES[SW_TYPE] << "." << sw->localId() << "\" : {" << std::endl;
    
    // Bitstream
    os << "\"" << ADGKEY_NAMES[COMP_BITENC] << "\" : {" << std::endl;
    os << "}," << std::endl;

    // Comp Node
    os << "\"" << ADGKEY_NAMES[COMP_NODE] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_PRED] << "\" : false," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_BITS] << "\" : " << sw->datawidth() << "," << std::endl;
    //os << "\"comment\": \"row3_col3\"," << std::endl;
    os << "\"parameterClassName\": \"dsagen2.comp.config.CompNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_UNITBITS] << "\" : " << sw->granularity() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[SW_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << sw->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_NODEACTIVE] << "\" : true" << std::endl;
    os << "}," << std::endl;
 
    // Output Buffer
    os << "\"" << ADGKEY_NAMES[COMP_OUTBUFF] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_OUTBUFF_DEPTH] << "\" : " << sw->delay_fifo_depth() << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.comp.config.common.CompNodeOutputBufferParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_OUTBUFF_STATIC] << "\" : " << !sw->flow_control() << std::endl;
    os << "}," << std::endl;

    // Switch Routing
    os << "\"" << ADGKEY_NAMES[SW_ROUTE] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[SW_ROUTE_FULLMAT] << "\" : ";
    for (int i = 0; i < sw->in_links().size(); i++) {
      if (i == 0)
              os << "[ ";
      for (int j = 0; j < sw->out_links().size(); j++) {
        if (j == 0)
              os << "[ ";
        auto connectivity_matrix = sw->printableRoutingTable(i, j);
        for (int k = 0; k < connectivity_matrix.size(); k++) {
          if (k == 0)
              os << "[ ";
          
          for (int l = 0; l < connectivity_matrix[k].size(); l++) {
            if (l == 0)
              os << "[ ";
            
            os << connectivity_matrix[k][l];
            if (l != connectivity_matrix[k].size() - 1) {
              os << ", ";
            } else {
              os << "] ";
            }
          }
          
          if (k != connectivity_matrix.size() - 1) {
            if (connectivity_matrix[k].size() != 0) {
              os << ", ";
            }
          } else {
            os << "] ";
          }
        }

        if (j < sw->out_links().size() - 1) {
          if (connectivity_matrix.size() != 0) {
            os << ", ";
          }
        } else {
          os << "] ";
        }

      }
      if (i < sw->in_links().size() - 1) {
        os << ",";
      } else {
        os << "] ";
      }
    }
    os << ", " << std::endl;
    
    os << "\"parameterClassName\" : \"dsagen2.comp.config.switch.SWRoutingParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[SW_ROUTE_INDIMAT] << "\" : [ ] " << std::endl;
   
    os << "}" << std::endl;

    os << "}";  

  }

  void Visit(ssivport* vport) override {
    os << "\"" << ADGKEY_NAMES[IVP_TYPE] << "." << vport->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[IVP_NODE] << "\" : {" << std::endl;

    // VectorPort Parameters
    os << "\"" << ADGKEY_NAMES[VP_IMPL] << "\" : " << 2 << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[VP_STATE] << "\" : " << BoolToString(vport->vp_stated()) << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.sync.config.IVPNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[DEPTH_BYTE] << "\" : " << vport->delay_fifo_depth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[IVP_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << vport->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[IVP_BROADCAST] << "\" : " << vport->broadcastIVP() << std::endl;

    os << "}" << std::endl;
    os << "}";
  }

  void Visit(ssovport* vport) override {
    os << "\"" << ADGKEY_NAMES[OVP_TYPE] << "." << vport->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[OVP_NODE] << "\" : {" << std::endl;

    // VectorPort Parameters
    os << "\"" << ADGKEY_NAMES[OVP_DISCARD] << "\" : " << vport->discardOVP() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[VP_IMPL] << "\" : " << 2 << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[VP_STATE] << "\" : " << BoolToString(vport->vp_stated()) << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.sync.config.OVPNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[OVP_TASKFLOW] << "\" : " << vport->taskOVP() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[DEPTH_BYTE] << "\" : " << vport->delay_fifo_depth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[OVP_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << vport->localId() << std::endl;

    os << "}" << std::endl;
    os << "}";
  }

  void Visit(ssfu* fu) override {
    os << "\"" << ADGKEY_NAMES[PE_TYPE] << "." << fu->localId() << "\" : {" << std::endl;
    
    // Bitstream
    os << "\"" << ADGKEY_NAMES[COMP_BITENC] << "\" : {" << std::endl;
    os << "}," << std::endl;
    
    // Comp Node
    os << "\"" << ADGKEY_NAMES[COMP_NODE] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_PRED] << "\" : false," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_BITS] << "\" : " << fu->datawidth() << "," << std::endl;
    //os << "\"comment\": \"row3_col3\"," << std::endl;
    os << "\"parameterClassName\": \"dsagen2.comp.config.CompNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_UNITBITS] << "\" : " << fu->granularity() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[PE_TYPE] << "\","<< std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << fu->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_NODEACTIVE] << "\" : true" << std::endl;
    os << "}," << std::endl;

    // Output Buffer
    os << "\"" << ADGKEY_NAMES[COMP_OUTBUFF] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_OUTBUFF_DEPTH] << "\" : 2," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.comp.config.common.CompNodeOutputBufferParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[COMP_OUTBUFF_STATIC] << "\" : " << BoolToString(!fu->flow_control()) << std::endl;
    os << "}," << std::endl;

    // Register Files
    os << "\"" << ADGKEY_NAMES[PE_REG] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_REG_SIZE] << "\" : " << fu->regFileSize() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_REG_TYPE] << "\" : " << BoolToString(fu->asyncReg()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_REG_UPD] << "\" : " << BoolToString(fu->updReg()) << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.comp.config.processing_element.PERegFileParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_REG_RESET] << "\" : [ 0 ]" << std::endl;
    os << "}," << std::endl;

    // Control Parameters
    os << "\"" << ADGKEY_NAMES[PE_CTRL] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_CTRL_OUTPUT] << "\" : " << BoolToString(fu->outputCtrl()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_CTRL_SIZE] << "\" : " << BoolToString(fu->ctrlLUTSize()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_CTRL_ABSTAIN] << "\" : " << BoolToString(fu->abstain()) << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.comp.config.processing_element.PEMetaCtrlParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_CTRL_INPUT] << "\" : " << BoolToString(fu->inputCtrl()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_CTRL_REUSE] << "\" : " << BoolToString(fu->operandReuse()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_CTRL_RESET] << "\" : " << BoolToString(fu->registerReset()) <<"," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_CTRL_DISCARD] << "\" : " << BoolToString(fu->resultDiscard())  << std::endl;
    os << "}," << std::endl;

    // Operations
    os << "\"" << ADGKEY_NAMES[PE_OP] << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_OP_DYNA] << "\" : " << BoolToString(fu->flow_control()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_OP_TYPEENC] << "\" : [ ";
    
    int idx_inst = 0;
    int num_inst = fu->fu_type().capability.size();
    for (auto& elem : fu->fu_type().capability) {
      os << "\"" << dsa::name_of_inst(elem.op) << "\"";
      if (idx_inst < num_inst - 1) {
        os << ", ";
        idx_inst++;
      }
    }
    os << " ]," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_OP_REPEAT] << "\" : " << fu->maxOpRepeat() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_OP_DEFLAT] << "\" : " << fu->definedLatency() << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.comp.config.processing_element.PEDsaOperationParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_OP_INSTSIZE] << "\" : " << fu->max_util() << "," << std::endl;
    //os << "\"" << ADGKEY_NAMES[PE_OP_INSTSIZE] << "\" : " << fu->instSlotSize() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[PE_OP_FIFODEP] << "\" : " << fu->delay_fifo_depth();
    os << "}" << std::endl;

    os << "}";
  }


  void Visit(ssdma* dma) override {
    os << "\"" << ADGKEY_NAMES[DMA_TYPE] << "." << dma->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NODE] << "\" : {" << std::endl;
    
    // Memory node parameters
    os << "\"" << ADGKEY_NAMES[NUM_MEM_WRITE] << "\" : " << dma->numWrite() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_UNITBITS] << "\" : " << dma->memUnitBits() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_READ] << "\" : " << dma->numRead() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L1D] << "\" : " << dma->maxLength1D() << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.mem.config.MemNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L3D] << "\" : " << dma->maxLength3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_CAP] << "\" : " << dma->capacity() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_L1D] << "\" : " << BoolToString( dma->linearLength1DStream()) << "," << std::endl;
    os << "\"" << "numGenDataType" << "\" : " << dma->numGenDataType() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_PADDING] << "\" : " << BoolToString(dma->linearPadding()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D2D] << "\" : " << dma->maxAbsStretch3D2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_PENDING_REQUEST] << "\" : " << dma->numPendingRequest() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_L1DUNITBITSEXP] << "\" : " << dma->numLength1DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE3D] << "\" : " << dma->maxAbsStride3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE1D] << "\" : " << dma->maxAbsStride1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_S2D] << "\" : " << BoolToString(dma->indirectStride2DStream()) << "," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_ATOMIC_OP] << "\" : [ ";
    for (int i = 0; i < dma->atomicOperations().size(); i++) {
      os << "\"" << dma->atomicOperations()[i] << "\"";
      if (i < dma->atomicOperations().size() - 1) {
        os << ", ";
      }
    }
    os << " ]," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_NUM_IDXUNITBITSEXP] << "\" : " << dma->numIdxUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRIDE2D] << "\" : " << dma->maxAbsDeltaStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_S2D] << "\" : " << BoolToString(dma->linearStride2DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L2D] << "\" : " << dma->maxLength2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH2D] << "\" : " << dma->maxAbsStretch2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[DMA_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_DATATYPE] << "\" : " << dma->numMemUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D1D] << "\" : " << dma->maxAbsStretch3D1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_INDEX] << "\" : " << BoolToString(dma->indirectIndexStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_STRIDE2D_UNIT_BITS_EXP] << "\" : " << dma->numStride2DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_WRITE_WIDTH] << "\" : " << dma->writeWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE2D] << "\" : " << dma->maxAbsStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_READ_WIDTH] << "\" : " << dma->readWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << dma->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_STR_STATE] << "\" : " << BoolToString(dma->streamStated()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_SPM_BANK] << "\" : " << dma->numSpmBank() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_L1D] << "\" : " << BoolToString(dma->indirectLength1DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRETCH2D] << "\" : " << dma->maxAbsDeltaStretch2D() << std::endl;


    os << "}" << std::endl;
    os << "}";

  }

  void Visit(ssscratchpad* spm) override {
    os << "\"" << ADGKEY_NAMES[SPM_TYPE] << "." << spm->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NODE] << "\" : {" << std::endl;
    
    // Memory node parameters
    os << "\"" << ADGKEY_NAMES[NUM_MEM_WRITE] << "\" : " << spm->numWrite() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_UNITBITS] << "\" : " << spm->memUnitBits() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_READ] << "\" : " << spm->numRead() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L1D] << "\" : " << spm->maxLength1D() << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.mem.config.MemNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L3D] << "\" : " << spm->maxLength3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_CAP] << "\" : " << spm->capacity() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_L1D] << "\" : " << BoolToString( spm->linearLength1DStream()) << "," << std::endl;
    os << "\"" << "numGenDataType" << "\" : " << spm->numGenDataType() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_PADDING] << "\" : " << BoolToString(spm->linearPadding()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D2D] << "\" : " << spm->maxAbsStretch3D2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_PENDING_REQUEST] << "\" : " << spm->numPendingRequest() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_L1DUNITBITSEXP] << "\" : " << spm->numLength1DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE3D] << "\" : " << spm->maxAbsStride3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE1D] << "\" : " << spm->maxAbsStride1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_S2D] << "\" : " << BoolToString(spm->indirectStride2DStream()) << "," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_ATOMIC_OP] << "\" : [ ";
    for (int i = 0; i < spm->atomicOperations().size(); i++) {
      os << "\"" << spm->atomicOperations()[i] << "\"";
      if (i < spm->atomicOperations().size() - 1) {
        os << ", ";
      }
    }
    os << " ]," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_NUM_IDXUNITBITSEXP] << "\" : " << spm->numIdxUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRIDE2D] << "\" : " << spm->maxAbsDeltaStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_S2D] << "\" : " << BoolToString(spm->linearStride2DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L2D] << "\" : " << spm->maxLength2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH2D] << "\" : " << spm->maxAbsStretch2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[SPM_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_DATATYPE] << "\" : " << spm->numMemUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D1D] << "\" : " << spm->maxAbsStretch3D1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_INDEX] << "\" : " << BoolToString(spm->indirectIndexStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_STRIDE2D_UNIT_BITS_EXP] << "\" : " << spm->numStride2DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_WRITE_WIDTH] << "\" : " << spm->writeWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE2D] << "\" : " << spm->maxAbsStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_READ_WIDTH] << "\" : " << spm->readWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << spm->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_STR_STATE] << "\" : " << BoolToString(spm->streamStated()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_SPM_BANK] << "\" : " << spm->numSpmBank() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_L1D] << "\" : " << BoolToString(spm->indirectLength1DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRETCH2D] << "\" : " << spm->maxAbsDeltaStretch2D() << std::endl;


    os << "}" << std::endl;
    os << "}";
  }

  void Visit(ssrecurrence* rec) override {
    os << "\"" << ADGKEY_NAMES[REC_TYPE] << "." << rec->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NODE] << "\" : {" << std::endl;
    
    // Memory node parameters
    os << "\"" << ADGKEY_NAMES[NUM_MEM_WRITE] << "\" : " << rec->numWrite() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_UNITBITS] << "\" : " << rec->memUnitBits() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_READ] << "\" : " << rec->numRead() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L1D] << "\" : " << rec->maxLength1D() << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.mem.config.MemNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L3D] << "\" : " << rec->maxLength3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_CAP] << "\" : " << rec->capacity() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_L1D] << "\" : " << BoolToString( rec->linearLength1DStream()) << "," << std::endl;
    os << "\"" << "numGenDataType" << "\" : " << rec->numGenDataType() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_PADDING] << "\" : " << BoolToString(rec->linearPadding()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D2D] << "\" : " << rec->maxAbsStretch3D2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_PENDING_REQUEST] << "\" : " << rec->numPendingRequest() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_L1DUNITBITSEXP] << "\" : " << rec->numLength1DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE3D] << "\" : " << rec->maxAbsStride3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE1D] << "\" : " << rec->maxAbsStride1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_S2D] << "\" : " << BoolToString(rec->indirectStride2DStream()) << "," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_ATOMIC_OP] << "\" : [ ";
    for (int i = 0; i < rec->atomicOperations().size(); i++) {
      os << "\"" << rec->atomicOperations()[i] << "\"";
      if (i < rec->atomicOperations().size() - 1) {
        os << ", ";
      }
    }
    os << " ]," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_NUM_IDXUNITBITSEXP] << "\" : " << rec->numIdxUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRIDE2D] << "\" : " << rec->maxAbsDeltaStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_S2D] << "\" : " << BoolToString(rec->linearStride2DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L2D] << "\" : " << rec->maxLength2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH2D] << "\" : " << rec->maxAbsStretch2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[REC_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_DATATYPE] << "\" : " << rec->numMemUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D1D] << "\" : " << rec->maxAbsStretch3D1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_INDEX] << "\" : " << BoolToString(rec->indirectIndexStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_STRIDE2D_UNIT_BITS_EXP] << "\" : " << rec->numStride2DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_WRITE_WIDTH] << "\" : " << rec->writeWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE2D] << "\" : " << rec->maxAbsStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_READ_WIDTH] << "\" : " << rec->readWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << rec->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_STR_STATE] << "\" : " << BoolToString(rec->streamStated()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_SPM_BANK] << "\" : " << rec->numSpmBank() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_L1D] << "\" : " << BoolToString(rec->indirectLength1DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRETCH2D] << "\" : " << rec->maxAbsDeltaStretch2D() << std::endl;


    os << "}" << std::endl;
    os << "}";
  }

  void Visit(ssgenerate* gen) override {
    os << "\"" << ADGKEY_NAMES[GEN_TYPE] << "." << gen->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NODE] << "\" : {" << std::endl;
    
    // Memory node parameters
    os << "\"" << ADGKEY_NAMES[NUM_MEM_WRITE] << "\" : " << gen->numWrite() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_UNITBITS] << "\" : " << gen->memUnitBits() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_READ] << "\" : " << gen->numRead() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L1D] << "\" : " << gen->maxLength1D() << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.mem.config.MemNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L3D] << "\" : " << gen->maxLength3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_CAP] << "\" : " << gen->capacity() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_L1D] << "\" : " << BoolToString( gen->linearLength1DStream()) << "," << std::endl;
    os << "\"" << "numGenDataType" << "\" : " << gen->numGenDataType() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_PADDING] << "\" : " << BoolToString(gen->linearPadding()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D2D] << "\" : " << gen->maxAbsStretch3D2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_PENDING_REQUEST] << "\" : " << gen->numPendingRequest() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_L1DUNITBITSEXP] << "\" : " << gen->numLength1DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE3D] << "\" : " << gen->maxAbsStride3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE1D] << "\" : " << gen->maxAbsStride1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_S2D] << "\" : " << BoolToString(gen->indirectStride2DStream()) << "," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_ATOMIC_OP] << "\" : [ ";
    for (int i = 0; i < gen->atomicOperations().size(); i++) {
      os << "\"" << gen->atomicOperations()[i] << "\"";
      if (i < gen->atomicOperations().size() - 1) {
        os << ", ";
      }
    }
    os << " ]," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_NUM_IDXUNITBITSEXP] << "\" : " << gen->numIdxUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRIDE2D] << "\" : " << gen->maxAbsDeltaStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_S2D] << "\" : " << BoolToString(gen->linearStride2DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L2D] << "\" : " << gen->maxLength2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH2D] << "\" : " << gen->maxAbsStretch2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[GEN_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_DATATYPE] << "\" : " << gen->numMemUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D1D] << "\" : " << gen->maxAbsStretch3D1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_INDEX] << "\" : " << BoolToString(gen->indirectIndexStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_STRIDE2D_UNIT_BITS_EXP] << "\" : " << gen->numStride2DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_WRITE_WIDTH] << "\" : " << gen->writeWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE2D] << "\" : " << gen->maxAbsStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_READ_WIDTH] << "\" : " << gen->readWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << gen->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_STR_STATE] << "\" : " << BoolToString(gen->streamStated()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_SPM_BANK] << "\" : " << gen->numSpmBank() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_L1D] << "\" : " << BoolToString(gen->indirectLength1DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRETCH2D] << "\" : " << gen->maxAbsDeltaStretch2D() << std::endl;


    os << "}" << std::endl;
    os << "}";
  }

  void Visit(ssregister* reg) override {
    os << "\"" << ADGKEY_NAMES[REG_TYPE] << "." << reg->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NODE] << "\" : {" << std::endl;
    
    // Memory node parameters
    os << "\"" << ADGKEY_NAMES[NUM_MEM_WRITE] << "\" : " << reg->numWrite() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_UNITBITS] << "\" : " << reg->memUnitBits() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_READ] << "\" : " << reg->numRead() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L1D] << "\" : " << reg->maxLength1D() << "," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.mem.config.MemNodeParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L3D] << "\" : " << reg->maxLength3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_CAP] << "\" : " << reg->capacity() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_L1D] << "\" : " << BoolToString( reg->linearLength1DStream()) << "," << std::endl;
    os << "\"" << "numGenDataType" << "\" : " << reg->numGenDataType() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_PADDING] << "\" : " << BoolToString(reg->linearPadding()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D2D] << "\" : " << reg->maxAbsStretch3D2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_PENDING_REQUEST] << "\" : " << reg->numPendingRequest() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_L1DUNITBITSEXP] << "\" : " << reg->numLength1DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE3D] << "\" : " << reg->maxAbsStride3D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE1D] << "\" : " << reg->maxAbsStride1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_S2D] << "\" : " << BoolToString(reg->indirectStride2DStream()) << "," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_ATOMIC_OP] << "\" : [ ";
    for (int i = 0; i < reg->atomicOperations().size(); i++) {
      os << "\"" << reg->atomicOperations()[i] << "\"";
      if (i < reg->atomicOperations().size() - 1) {
        os << ", ";
      }
    }
    os << " ]," << std::endl;

    os << "\"" << ADGKEY_NAMES[MEM_NUM_IDXUNITBITSEXP] << "\" : " << reg->numIdxUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRIDE2D] << "\" : " << reg->maxAbsDeltaStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_LINEAR_S2D] << "\" : " << BoolToString(reg->linearStride2DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_L2D] << "\" : " << reg->maxLength2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH2D] << "\" : " << reg->maxAbsStretch2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODETYPE] << "\" : \"" << ADGKEY_NAMES[REG_TYPE] << "\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[NUM_MEM_DATATYPE] << "\" : " << reg->numMemUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRETCH3D1D] << "\" : " << reg->maxAbsStretch3D1D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_INDEX] << "\" : " << BoolToString(reg->indirectIndexStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_STRIDE2D_UNIT_BITS_EXP] << "\" : " << reg->numStride2DUnitBitsExp() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_WRITE_WIDTH] << "\" : " << reg->writeWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_STRIDE2D] << "\" : " << reg->maxAbsStride2D() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_READ_WIDTH] << "\" : " << reg->readWidth() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[NODEID] << "\" : " << reg->localId() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_STR_STATE] << "\" : " << BoolToString(reg->streamStated()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_NUM_SPM_BANK] << "\" : " << reg->numSpmBank() << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_INDIRECT_L1D] << "\" : " << BoolToString(reg->indirectLength1DStream()) << "," << std::endl;
    os << "\"" << ADGKEY_NAMES[MEM_MAX_ABS_DELTA_STRETCH2D] << "\" : " << reg->maxAbsDeltaStretch2D() << std::endl;


    os << "}" << std::endl;
    os << "}";
  }

};

struct SchedWriter : dfg::Visitor {  
  std::ostream& os;
  Schedule* sched;

  SchedWriter(std::ostream& os_, Schedule* sched_) : os(os_), sched(sched_) {} 

  void Visit(dfg::Node* node) override {
    auto adg_node = sched->vex_prop()[node->id()].node();
    auto lane = sched->vex_prop()[node->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << node->id() << "," << std::endl;
    os << "\"VertexType\" : \"Unknown\"," << std::endl;
    os << "\"MappedNodeId\" : " << "\"node\"" << "," << std::endl;
    os << "\"MappedNodeLane\" : " << lane << std::endl; 
    os << "}" << std::endl;
  }

  void Visit(dfg::Instruction* inst) {
    auto adg_node = sched->vex_prop()[inst->id()].node();
    auto lane = sched->vex_prop()[inst->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << inst->id() << "," << std::endl;
    os << "\"VertexType\" : \"Instruction\"," << std::endl;
    os << "\"Instruction Name\" : \"" << dsa::name_of_inst(inst->inst()) << "\"," << std::endl;
    os << "\"Instruction OpCode\" : " << inst->inst() << "," << std::endl;

    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[PE_TYPE] << "." <<  adg_node->localId() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    }
  }
  void Visit(dfg::Operation* oper) {
    auto adg_node = sched->vex_prop()[oper->id()].node();
    auto lane = sched->vex_prop()[oper->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << oper->id() << "," << std::endl;
    os << "\"VertexType\" : \"Instruction\"," << std::endl;
    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[PE_TYPE] << "." <<  adg_node->localId() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    }
  }
  void Visit(dfg::InputPort* ivp) {
    auto adg_node = sched->vex_prop()[ivp->id()].node();
    auto lane = sched->vex_prop()[ivp->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << ivp->id() << "," << std::endl;
    os << "\"VertexType\" : \"InputPort\"," << std::endl;
    
    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[IVP_TYPE] << "." <<   adg_node->id() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl;
      os << "}" << std::endl;
    }
  }
  void Visit(dfg::OutputPort* ovp) {
    auto adg_node = sched->vex_prop()[ovp->id()].node();
    auto lane = sched->vex_prop()[ovp->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << ovp->id() << "," << std::endl;
    os << "\"VertexType\" : \"OutputPort\"," << std::endl;
    
    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[OVP_TYPE] << "." <<   adg_node->id() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl;
      os << "}" << std::endl;
    }
  }

  void Visit(dfg::DMA* dma) {
    auto adg_node = sched->vex_prop()[dma->id()].node();
    auto lane = sched->vex_prop()[dma->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << dma->id() << "," << std::endl;
    os << "\"VertexType\" : \"DirectMemoryAccess\"," << std::endl;
    
    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[DMA_TYPE] << "." <<   adg_node->localId() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl;
      os << "}" << std::endl;
    }
  }

  void Visit(dfg::Scratchpad* spad) {
    auto adg_node = sched->vex_prop()[spad->id()].node();
    auto lane = sched->vex_prop()[spad->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << spad->id() << "," << std::endl;
    os << "\"VertexType\" : \"ScratchpadMemory\"," << std::endl;

    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[SPM_TYPE] << "." << adg_node->localId() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl;
      os << "}" << std::endl;
    }
  }
  void Visit(dfg::Recurrance* rec) {
    auto adg_node = sched->vex_prop()[rec->id()].node();
    auto lane = sched->vex_prop()[rec->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << rec->id() << "," << std::endl;
    os << "\"VertexType\" : \"Recurrance\"," << std::endl;
    
    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[REC_TYPE] << "." << adg_node->localId() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl;
      os << "}" << std::endl;
    }
  }
  void Visit(dfg::Register* reg) {
    auto adg_node = sched->vex_prop()[reg->id()].node();
    auto lane = sched->vex_prop()[reg->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << reg->id() << "," << std::endl;
    os << "\"VertexType\" : \"Register\"," << std::endl;
    
    
    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[REG_TYPE] << "." << adg_node->localId() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl;
      os << "}" << std::endl;
    }
  }
  void Visit(dfg::Generate* gen) {
    auto adg_node = sched->vex_prop()[gen->id()].node();
    auto lane = sched->vex_prop()[gen->id()].lane();
    os << "{" << std::endl;
    os << "\"VertexId\" : " << gen->id() << "," << std::endl;
    os << "\"VertexType\" : \"Generate\"," << std::endl;
    
    if (adg_node) {
      os << "\"MappedNodeId\" : \"" << ADGKEY_NAMES[GEN_TYPE] << "." << adg_node->localId() << "\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl; 
      os << "}" << std::endl;
    } else {
      os << "\"MappedNodeId\" : " << "\"\"," << std::endl;
      os << "\"MappedNodeLane\" : " << lane << std::endl;
      os << "}" << std::endl;
    } 
  }

};

inline void print_spatial_json(std::ostream& os, SpatialFabric* fabric, bool end=true) {
  DSA_CHECK(os.good()) << "ADG (json) File has bad output stream";

  os << "{" << std::endl;
  os << "\"" << adg::ADGKEY_NAMES[adg::DSANODES] << "\" : { " << std::endl;
  dsa::adg::JsonWriter writer(os);
  
  // First Dump Processing Elements
  for (auto* node : fabric->fu_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump Switches
  for (auto* node : fabric->switch_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump Reccurrance
  for (auto* node : fabric->recur_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump Register
  for (auto* node : fabric->reg_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump Generate
  for (auto* node : fabric->gen_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump Scratchpad
  for (auto* node : fabric->scratch_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump DMA
  for (auto* node : fabric->dma_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump Input VectorPorts
  for (auto* node : fabric->input_list()) {
    node->Accept(&writer);
    os << "," << std::endl;
  }

  // Then Dump Output VectorPorts
  for (int i = 0; i < fabric->output_list().size(); i++) {
    fabric->output_list()[i]->Accept(&writer);
    if (i != fabric->output_list().size() - 1) {
      os << ",";
    }
    os << std::endl;
  }

  os << "}," << std::endl;
  os << "\"" << adg::ADGKEY_NAMES[adg::DSAEDGES] << "\" : [ ";
  // Print Edges
  for (int i = 0; i < fabric->link_list().size(); i++) {
    auto link = fabric->link_list()[i];
    os << "{" << std::endl;
    os << "\"" << adg::ADGKEY_NAMES[adg::SOURCENODETYPE] << "\" : \"";
    if (auto fu = dynamic_cast<ssfu*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::PE_TYPE];
    else if (auto sw = dynamic_cast<ssswitch*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::SW_TYPE];
    else if (auto ivp = dynamic_cast<ssivport*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::IVP_TYPE];
    else if (auto ovp = dynamic_cast<ssovport*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::OVP_TYPE];
    else if (auto dma = dynamic_cast<ssdma*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::DMA_TYPE];
    else if (auto spm = dynamic_cast<ssscratchpad*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::SPM_TYPE];
    else if (auto ovp = dynamic_cast<ssrecurrence*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::REC_TYPE];
    else if (auto ovp = dynamic_cast<ssgenerate*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::GEN_TYPE];
    else if (auto ovp = dynamic_cast<ssregister*>(link->source()))
      os << adg::ADGKEY_NAMES[adg::REG_TYPE];
    os << "\"," << std::endl;

    os << "\"" << adg::ADGKEY_NAMES[adg::SOURCENODEID] << "\" : " << link->source()->localId() << "," << std::endl;
    os << "\"" << adg::ADGKEY_NAMES[adg::SOURCEINDEX] << "\" : " << link->source()->link_index(link, false) << "," << std::endl;

    os << "\"" << adg::ADGKEY_NAMES[adg::SINKNODETYPE] << "\" : \"";
    if (auto fu = dynamic_cast<ssfu*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::PE_TYPE];
    else if (auto sw = dynamic_cast<ssswitch*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::SW_TYPE];
    else if (auto ivp = dynamic_cast<ssivport*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::IVP_TYPE];
    else if (auto ovp = dynamic_cast<ssovport*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::OVP_TYPE];
    else if (auto dma = dynamic_cast<ssdma*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::DMA_TYPE];
    else if (auto spm = dynamic_cast<ssscratchpad*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::SPM_TYPE];
    else if (auto ovp = dynamic_cast<ssrecurrence*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::REC_TYPE];
    else if (auto ovp = dynamic_cast<ssgenerate*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::GEN_TYPE];
    else if (auto ovp = dynamic_cast<ssregister*>(link->sink()))
      os << adg::ADGKEY_NAMES[adg::REG_TYPE];
    os << "\"," << std::endl;

    os << "\"" << adg::ADGKEY_NAMES[adg::SINKNODEID] << "\" : " << link->sink()->localId() << "," << std::endl;
    os << "\"" << adg::ADGKEY_NAMES[adg::SINKINDEX] << "\" : " << link->sink()->link_index(link, true) << std::endl;
    os << "}";
    if (i < fabric->link_list().size() - 1)
      os << ",";
    os << " ";
  }

  if (end) {
    os << "]" << std::endl;
    os << "}" << std::endl;
  } else {
    os << "] ," << std::endl;
  }
}

inline void print_schedule_json(ofstream& os, Schedule* sched, bool end=true, int number_of_cores=1, int number_of_banks=1) {
  DSA_CHECK(os.good()) << "ADG (json) File has bad output stream";
  dsa::adg::SchedWriter sched_writer(os, sched);

  std::string dram_performance = "";
  std::string l2_performance = "";
  std::string spm_performance = "";

  os << "{" << std::endl;
  os << "\"Name\" : \"" << sched->ssdfg()->filename << "\"," << std::endl;
  os << "\"Performance\" : " << sched->estimated_performance(spm_performance, l2_performance, dram_performance, number_of_cores, number_of_banks) << "," << std::endl;

  os << "\"SPM Performance\" : " << spm_performance << "," << std::endl;
  os << "\"L2 Performance\" : " << l2_performance << "," << std::endl;
  os << "\"DRAM Performance\" : " << dram_performance << "," << std::endl;

  os << "\"Vertices\" : [" << std::endl;
  for (auto elem : sched->ssdfg()->nodes) {
    elem->Accept(&sched_writer);
    if (elem != sched->ssdfg()->nodes.back())
      os << ",";
    os << std::endl;
  }
  os << "]," << std::endl;
  os << "\"Edges\" : [" << std::endl;
  for (auto edge : sched->ssdfg()->edges) {
    auto links = sched->edge_prop()[edge.id].links;
    os << "{" << std::endl;
    os << "\"EdgeId\" : " << edge.id << "," << std::endl; 
    os << "\"ValueNode\" : " << edge.val()->nid << "," << std::endl;
    os << "\"ValueIndex\" : " << edge.val()->index << "," << std::endl;
    os << "\"Bitwidth\" : " << edge.bitwidth() << "," << std::endl;
    os << "\"Startbit\" : " <<  sched->edge_prop()[edge.id].source_bit << "," << std::endl;
    os << "\"SourceId\" : " << edge.def()->id() << "," << std::endl;
    os << "\"SinkId\" : " << edge.use()->id() << "," << std::endl;
    os << "\"Links\" : [" << std::endl;
    for (auto link_slot : links) {
      auto link = link_slot.second;
      os << "{" << std::endl;
      os << "\"" << adg::ADGKEY_NAMES[adg::SOURCENODETYPE] << "\" : \"";
      if (auto fu = dynamic_cast<ssfu*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::PE_TYPE];
      else if (auto sw = dynamic_cast<ssswitch*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::SW_TYPE];
      else if (auto ivp = dynamic_cast<ssivport*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::IVP_TYPE];
      else if (auto ovp = dynamic_cast<ssovport*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::OVP_TYPE];
      else if (auto dma = dynamic_cast<ssdma*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::DMA_TYPE];
      else if (auto spm = dynamic_cast<ssscratchpad*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::SPM_TYPE];
      else if (auto ovp = dynamic_cast<ssrecurrence*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::REC_TYPE];
      else if (auto ovp = dynamic_cast<ssgenerate*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::GEN_TYPE];
      else if (auto ovp = dynamic_cast<ssregister*>(link->source()))
        os << adg::ADGKEY_NAMES[adg::REG_TYPE];
      os << "\"," << std::endl;

      os << "\"" << adg::ADGKEY_NAMES[adg::SOURCENODEID] << "\" : " << link->source()->localId() << "," << std::endl;

      os << "\"" << adg::ADGKEY_NAMES[adg::SINKNODETYPE] << "\" : \"";
      if (auto fu = dynamic_cast<ssfu*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::PE_TYPE];
      else if (auto sw = dynamic_cast<ssswitch*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::SW_TYPE];
      else if (auto ivp = dynamic_cast<ssivport*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::IVP_TYPE];
      else if (auto ovp = dynamic_cast<ssovport*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::OVP_TYPE];
      else if (auto dma = dynamic_cast<ssdma*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::DMA_TYPE];
      else if (auto spm = dynamic_cast<ssscratchpad*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::SPM_TYPE];
      else if (auto ovp = dynamic_cast<ssrecurrence*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::REC_TYPE];
      else if (auto ovp = dynamic_cast<ssgenerate*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::GEN_TYPE];
      else if (auto ovp = dynamic_cast<ssregister*>(link->sink()))
        os << adg::ADGKEY_NAMES[adg::REG_TYPE];
      os << "\"," << std::endl;

      os << "\"" << adg::ADGKEY_NAMES[adg::SINKNODEID] << "\" : " << link->sink()->localId() << "," << std::endl;
      os << "\"StartBit\"" << " : " << link_slot.first << "" << std::endl;
      os << "}";
      if (link->id() != links.back().second->id())
        os << ",";
      os << " ";
    }

    os << "]" << std::endl;
    os << "}";
    if (edge.id != sched->ssdfg()->edges.back().id)
      os << ",";
    os << std::endl;

  }
  os << "]" << std::endl;
  
  if (!end)
    os << "}," << std::endl;
  else
    os << "}" << std::endl;
}

inline void print_json(const std::string& name, SpatialFabric* fabric) {
  ofstream os(name);
  DSA_CHECK(os.good()) << "ADG (json) File" << name << "has bad output stream";
  print_spatial_json(os, fabric, true);
}

inline void print_sched_json(const std::string& name, SpatialFabric* fabric, std::vector<WorkloadSchedules> workloads, int num_cores=1, int num_banks=1, int system_bus_width=64) {
  ofstream os(name);
  DSA_CHECK(os.good()) << "ADG (json) File" << name << "has bad output stream";
  print_spatial_json(os, fabric, false);

  os << "\"SystemParameters\" : {" << std::endl;
  os << "\"NumberOfCores\" : " << num_cores << "," << std::endl;
  os << "\"NumberOfBanks\" : " << num_banks << "," << std::endl;
  os << "\"SystemBusWidth\" : " << system_bus_width << std::endl;
  os << "}," << std::endl;

  os << "\"Workloads\" : [ " << std::endl;
  for (auto schedules = workloads.begin(); schedules != workloads.end(); schedules++) {
    os << "{" << std::endl;
    os << "\"Schedules\" : [ " << std::endl;
    auto &sched_array = schedules->sched_array;
    for (auto sched = sched_array.begin(); sched!= sched_array.end(); sched++) {
      bool end = (std::next(sched) == sched_array.end());
      print_schedule_json(os, &(*sched), end);
    }
    os << "]" << std::endl;
    if (std::next(schedules) != workloads.end())
      os << "}," << std::endl;
    else 
      os << "}" << std::endl;
  }
  os << "]" << std::endl;
  os << "}" << std::endl;
}

inline void print_sched_json(const std::string& name, SpatialFabric* fabric, Schedule* sched) {
  ofstream os(name);
  DSA_CHECK(os.good()) << "ADG (json) File" << name << "has bad output stream";
  print_spatial_json(os, fabric, false);

  os << "\"Workloads\" : [ " << std::endl;
  os << "{" << std::endl;
  os << "\"Schedules\" : [ " << std::endl;
  print_schedule_json(os, sched, false);
  os << "]" << std::endl;
  os << "}" << std::endl;
  os << "]" << std::endl;
  os << "}" << std::endl;
}

}  // namespace adg
}  // namespace dsa