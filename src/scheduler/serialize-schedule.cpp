#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/vector.hpp>

#include "schedule.h"
#include "serialization.h"

template <class Archive>
void serialize(Archive& ar, std::pair<int, int>& pr, const unsigned int version) {
  ar& pr.first;
  ar& pr.second;
}

template <class Archive>
void Schedule::VertexProp::serialize(Archive& ar, const unsigned version) {
  ar& BOOST_SERIALIZATION_NVP(min_lat);
  ar& BOOST_SERIALIZATION_NVP(max_lat);
  ar& BOOST_SERIALIZATION_NVP(lat);
  ar& BOOST_SERIALIZATION_NVP(vio);
  ar& BOOST_SERIALIZATION_NVP(idx);
  ar& BOOST_SERIALIZATION_NVP(width);
}

template <class Archive>
void Schedule::EdgeProp::serialize(Archive& ar, const unsigned version) {
  ar& BOOST_SERIALIZATION_NVP(num_links);
  ar& BOOST_SERIALIZATION_NVP(extra_lat);
  // need to restore links & passthroughs
  ar& BOOST_SERIALIZATION_NVP(links_ser);
  ar& BOOST_SERIALIZATION_NVP(passthroughs_ser);
}

template <class Archive>
void Schedule::NodeProp::serialize(Archive& ar, const unsigned version) {
  for (int i = 0; i < 8; ++i) {
    ar& BOOST_SERIALIZATION_NVP(slots[i].num_passthroughs);
    ar& BOOST_SERIALIZATION_NVP(slots[i].vertices);
  }
}

template <class Archive>
void Schedule::LinkProp::serialize(Archive& ar, const unsigned version) {
  for (int i = 0; i < 8; ++i) {
    ar& BOOST_SERIALIZATION_NVP(slots[i].lat);
    ar& BOOST_SERIALIZATION_NVP(slots[i].order);
    ar& BOOST_SERIALIZATION_NVP(slots[i].edges);
  }
}

template <class Archive>
void Schedule::serialize(Archive& ar, const unsigned version) {
  // Note: ssmodel not serialized
  ar& BOOST_SERIALIZATION_NVP(_name);
  ar& BOOST_SERIALIZATION_NVP(_ssDFG);
  ar& BOOST_SERIALIZATION_NVP(_num_mapped);
  ar& BOOST_SERIALIZATION_NVP(_links_mapped);
  ar& BOOST_SERIALIZATION_NVP(_edge_links_mapped);
  ar& BOOST_SERIALIZATION_NVP(_groupMismatch);
  ar& BOOST_SERIALIZATION_NVP(_vertexProp);
  ar& BOOST_SERIALIZATION_NVP(_edgeProp);
  ar& BOOST_SERIALIZATION_NVP(_nodeProp);
  ar& BOOST_SERIALIZATION_NVP(_linkProp);
  ar& BOOST_SERIALIZATION_NVP(_decode_lat_mis);
}

// Boost Stuff
BOOST_CLASS_EXPORT_GUID(Schedule::VertexProp, "Schedule::VertexProp");
BOOST_CLASS_EXPORT_GUID(Schedule::NodeProp, "Schedule::NodeProp");
BOOST_CLASS_EXPORT_GUID(Schedule::EdgeProp, "Schedule::EdgeProp");
BOOST_CLASS_EXPORT_GUID(Schedule::LinkProp, "Schedule::LinkProp");
BOOST_CLASS_EXPORT_GUID(Schedule, "Schedule");
