#pragma once

#include <ostream>

#include "dsa/arch/sub_model.h"
#include "dsa/arch/visitor.h"
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
    os << "\"" << ADGKEY_NAMES[SW_ROUTE_FULLMAT] << "\" : [ ]," << std::endl;
    os << "\"parameterClassName\" : \"dsagen2.comp.config.switch.SWRoutingParameters\"," << std::endl;
    os << "\"" << ADGKEY_NAMES[SW_ROUTE_INDIMAT] << "\" : [ ]" << std::endl;
    os << "}" << std::endl;

    os << "}";  

  }

  void Visit(ssivport* vport) override {
    os << "\"" << ADGKEY_NAMES[IVP_TYPE] << "." << vport->localId() << "\" : {" << std::endl;
    os << "\"" << ADGKEY_NAMES[IVP_NODE] << "\" : {" << std::endl;

    // VectorPort Parameters
    os << "\"" << ADGKEY_NAMES[VP_IMPL] << "\" : " << vport->vp_impl() << "," << std::endl;
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
    os << "\"" << ADGKEY_NAMES[VP_IMPL] << "\" : " << vport->vp_impl() << "," << std::endl;
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

}  // namespace adg
}  // namespace dsa
