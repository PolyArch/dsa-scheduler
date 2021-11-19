#pragma once

#include <iostream>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "dsa/arch/fabric.h"
#include "dsa/arch/fu_model.h"
#include "dsa/arch/sub_model.h"

namespace dsa {

class SSModel {
 public:
  SSModel(const char* filename);
  SSModel(SpatialFabric* sub);

  SpatialFabric* subModel() { return (_subModel); }

  void set_dispatch_inorder(bool d) { _dispatch_inorder = d; }
  bool dispatch_inorder() { return _dispatch_inorder; }

  void set_dispatch_width(int w) { _dispatch_width = w; }
  int dispatch_width() { return _dispatch_width; }

  void setMaxEdgeDelay(int d);

  void setCtrl(bool ctrl) {
    for (auto elem : this->subModel()->fu_list()) {
      elem->flow_control(ctrl);
    }
    for (auto elem : this->subModel()->switch_list()) {
      elem->flow_control(ctrl);
    }
  }

  int indirect(int v = -1) {
    if (v != -1) ind_memory = v;
    return ind_memory;
  }

  SSModel(const SSModel& m) {
    fu_types = m.fu_types;
    _subModel = m._subModel->copy();
    _dispatch_inorder = m._dispatch_inorder;
    _dispatch_width = m._dispatch_width;
    _maxEdgeDelay = m._maxEdgeDelay;
    ind_memory = m.ind_memory;
  }
  const std::string filename;

  ~SSModel() {
    // Don't delete _fuModel, just let it leak
    delete _subModel;
  }

  std::vector<Capability*> fu_types;

  int memory_size{4096};
  int io_ports{1};
  SpatialFabric* _subModel{nullptr};
  bool _dispatch_inorder{false};
  int _dispatch_width{2};
  int _maxEdgeDelay{15};
  int ind_memory{1};

  void parse_exec(std::istream& istream);
};

}  // namespace dsa