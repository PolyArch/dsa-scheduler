#ifndef __SS_MODEL_H__
#define __SS_MODEL_H__

#include <iostream>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

//#include "inst_model.h"
#include "fu_model.h"
#include "sub_model.h"

namespace SS_CONFIG {

class SSModel {
 public:
  SSModel(bool multi = false);
  SSModel(const char* filename, bool multi = false);
  SSModel(SubModel* sub, bool multi = false);

  FuModel* fuModel() { return (_fuModel); }
  SubModel* subModel() { return (_subModel); }

  void printGamsKinds(std::ostream& os);

  void set_dispatch_inorder(bool d) { _dispatch_inorder = d; }
  bool dispatch_inorder() { return _dispatch_inorder; }

  void set_dispatch_width(int w) { _dispatch_width = w; }
  int dispatch_width() { return _dispatch_width; }

  void setMaxEdgeDelay(int d) { _maxEdgeDelay = d; }
  int maxEdgeDelay() { return _maxEdgeDelay; }

  SSModel(const SSModel& m) {
     _fuModel = m._fuModel;
     _subModel = m._subModel->copy();
     _dispatch_inorder = m._dispatch_inorder;
     _dispatch_width = m._dispatch_width;
     _maxEdgeDelay = m._maxEdgeDelay;
  }
  const std::string filename;

 private:
  // InstModel *instModel;
  FuModel* _fuModel;
  SubModel* _subModel;

  bool _dispatch_inorder = false;
  int _dispatch_width = 2;
  int _maxEdgeDelay = 15;

  void parse_exec(std::istream& istream);
  void parse_json(std::istream& istream);
};

}  // namespace SS_CONFIG

#endif
