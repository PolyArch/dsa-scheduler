#include "sbpdg.h"
#include "serialization.h"

//Boost Includes
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/bitset.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/export.hpp>


template<class Archive>
void SbPDG_Edge::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_NVP(_ID);
  ar & BOOST_SERIALIZATION_NVP(_sbpdg); 
  ar & BOOST_SERIALIZATION_NVP(_def);
  ar & BOOST_SERIALIZATION_NVP(_use);
  ar & BOOST_SERIALIZATION_NVP(_etype); 
  ar & BOOST_SERIALIZATION_NVP(_l);
  ar & BOOST_SERIALIZATION_NVP(_r);
}

template<class Archive> 
void SbPDG_Operand::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_NVP(edges);
  ar & BOOST_SERIALIZATION_NVP(imm);
}

template<class Archive>
void SbPDG_Node::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_NVP(_sbpdg);
  ar & BOOST_SERIALIZATION_NVP(_ID); 
  ar & BOOST_SERIALIZATION_NVP(_name); 
  ar & BOOST_SERIALIZATION_NVP(_ops);
  ar & BOOST_SERIALIZATION_NVP(_inc_edge_list);
  ar & BOOST_SERIALIZATION_NVP(_uses);
  ar & BOOST_SERIALIZATION_NVP(_min_lat);
  ar & BOOST_SERIALIZATION_NVP(_sched_lat); 
  ar & BOOST_SERIALIZATION_NVP(_max_thr); 
  ar & BOOST_SERIALIZATION_NVP(_group_id);
}

template<class Archive>
void CtrlBits::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_NVP(_bits);
}

template<class Archive>
void SbPDG_Inst::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SbPDG_Node);
  ar & BOOST_SERIALIZATION_NVP(_predInv);
  ar & BOOST_SERIALIZATION_NVP(_isDummy);
  ar & BOOST_SERIALIZATION_NVP(_imm_slot);
  ar & BOOST_SERIALIZATION_NVP(_subFunc);
  ar & BOOST_SERIALIZATION_NVP(_ctrl_bits);
  ar & BOOST_SERIALIZATION_NVP(_reg);
  ar & BOOST_SERIALIZATION_NVP(_imm);
  ar & BOOST_SERIALIZATION_NVP(_sbinst);
}

template<class Archive>
void SbPDG_IO::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SbPDG_Node);
  ar & BOOST_SERIALIZATION_NVP(vec_);
}

template<class Archive>
void SbPDG_Input::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SbPDG_IO);
}

template<class Archive>
void SbPDG_Output::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SbPDG_IO);
}

template<class Archive>
void SbPDG_Vec::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_NVP(_name); 
  ar & BOOST_SERIALIZATION_NVP(_locMap); 
  ar & BOOST_SERIALIZATION_NVP(_ID);
  ar & BOOST_SERIALIZATION_NVP(_sbpdg); 
  ar & BOOST_SERIALIZATION_NVP(_group_id);
}

template<class Archive>
void SbPDG_VecInput::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SbPDG_Vec);
  ar & BOOST_SERIALIZATION_NVP(_inputs);
}

template<class Archive>
void SbPDG_VecOutput::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SbPDG_Vec);
  ar & BOOST_SERIALIZATION_NVP(_outputs);
}

template<class Archive>
void GroupProp::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_NVP(is_temporal);
}

template<class Archive>
void SbPDG::serialize(Archive & ar, const unsigned version) {
  ar & BOOST_SERIALIZATION_NVP(_nodes); 
  ar & BOOST_SERIALIZATION_NVP(_insts);
  ar & BOOST_SERIALIZATION_NVP(_inputs);
  ar & BOOST_SERIALIZATION_NVP(_outputs);
  ar & BOOST_SERIALIZATION_NVP(_orderedInsts);
  ar & BOOST_SERIALIZATION_NVP(_orderedInstsGroup);
  ar & BOOST_SERIALIZATION_NVP(_vecInputs);
  ar & BOOST_SERIALIZATION_NVP(_vecOutputs);
  ar & BOOST_SERIALIZATION_NVP(_edges);
  ar & BOOST_SERIALIZATION_NVP(removed_edges);
  ar & BOOST_SERIALIZATION_NVP(_vecInputGroups);
  ar & BOOST_SERIALIZATION_NVP(_vecOutputGroups);
  ar & BOOST_SERIALIZATION_NVP(_groupProps);
  ar & BOOST_SERIALIZATION_NVP(dummy_map);
  ar & BOOST_SERIALIZATION_NVP(dummys_per_port);
  ar & BOOST_SERIALIZATION_NVP(dummies);
  ar & BOOST_SERIALIZATION_NVP(dummiesOutputs);
}

//Boost Stuff
BOOST_CLASS_EXPORT_GUID(SbPDG_Edge,     "SbPDG_Edge");
BOOST_CLASS_EXPORT_GUID(SbPDG_Node,     "SbPDG_Node")
BOOST_CLASS_EXPORT_GUID(SbPDG_Inst,     "SbPDG_Inst");
BOOST_CLASS_EXPORT_GUID(SbPDG_Input,    "SbPDG_Input")
BOOST_CLASS_EXPORT_GUID(SbPDG_Output,   "SbPDG_Output")
BOOST_CLASS_EXPORT_GUID(SbPDG_Vec,      "SbPDG_Vec")
BOOST_CLASS_EXPORT_GUID(SbPDG_VecInput, "SbPDG_VecInput")
BOOST_CLASS_EXPORT_GUID(SbPDG_VecOutput,"SbPDG_VecOutput")
BOOST_CLASS_EXPORT_GUID(SbPDG,          "SbPDG")

SERIALIZABLE(SbPDG_Edge);
SERIALIZABLE(SbPDG_Node);
SERIALIZABLE(SbPDG_Inst);
SERIALIZABLE(SbPDG_Input);
SERIALIZABLE(SbPDG_Output);
SERIALIZABLE(SbPDG_Vec);
SERIALIZABLE(SbPDG_VecInput);
SERIALIZABLE(SbPDG_VecOutput);
SERIALIZABLE(SbPDG);




