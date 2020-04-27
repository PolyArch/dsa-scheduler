#ifndef __SS_MODEL_H__
#define __SS_MODEL_H__

#include <iostream>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "fu_model.h"
#include "sub_model.h"

namespace dsa {

const std::map<std::pair<int, int>, std::pair<double, double>> memory_data = {
  {{4096, 1}, {34125, 2.84}},
  {{4096, 2}, {94423, 3.81}},
  {{4096, 3}, {181106, 4.97}},
  {{4096, 4}, {296494, 6.34}},
  {{8192, 1}, {39643, 5.33}},
  {{8192, 2}, {105418, 6.63}},
  {{8192, 3}, {199273, 8.12}},
  {{8192, 4}, {323736, 9.82}},
  {{16384, 1}, {50650, 10.31}},
  {{16384, 2}, {127253, 12.24}},
  {{16384, 3}, {235226, 14.34}},
  {{16384, 4}, {377642, 16.73}},
  {{32768, 1}, {72545, 20.16}},
  {{32768, 2}, {170684, 23.42}},
  {{32768, 3}, {306771, 26.87}},
  {{32768, 4}, {486511, 30.53}},
};

class SSModel {
 public:
  SSModel(bool multi = false);
  SSModel(const char* filename, bool multi = false);
  SSModel(SubModel* sub, bool multi = false);

  SubModel* subModel() { return (_subModel); }

  void set_dispatch_inorder(bool d) { _dispatch_inorder = d; }
  bool dispatch_inorder() { return _dispatch_inorder; }

  void set_dispatch_width(int w) { _dispatch_width = w; }
  int dispatch_width() { return _dispatch_width; }

  void setMaxEdgeDelay(int d);

  void setCtrl(bool ctrl) {
    for (auto elem : this->subModel()->fu_list()) {
      elem->flow_control() = ctrl;
    }
    for (auto elem : this->subModel()->switch_list()) {
      elem->flow_control() = ctrl;
    }
  }

  int indirect(int v = -1) {
    if (v != -1) ind_memory = v;
    return ind_memory;
  }

  //double host_area() { return (indirect() * 88800) + 5200 + 41000; }
  //double host_power() { return (indirect() * 18.1) + 9.3 + 10.1; }

  double memory_area() {
    auto iters = memory_data.find({memory_size, io_ports});
    assert(iters != memory_data.end());
    return iters->second.first + (indirect() == 2)* 88800 + 5200;
  }

  double memory_power() {
    auto iters = memory_data.find({memory_size, io_ports});
    assert(iters != memory_data.end());
    return iters->second.second + (indirect() == 2) * 18.1 + 9.3;
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
 private:
  // InstModel *instModel;
  SubModel* _subModel;

  bool _dispatch_inorder = false;
  int _dispatch_width = 2;
  int _maxEdgeDelay = 15;
  int ind_memory{1};

  void parse_exec(std::istream& istream);
  void parse_json(std::istream& istream);
};

}  // namespace dsa

#endif
