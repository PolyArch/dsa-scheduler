#include "sub_model.h"
#include "model_parsing.h"

#include <assert.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
namespace pt = boost::property_tree;

using namespace SS_CONFIG;
using namespace std;

// COMMA IF NOT FIRST
static void CINF(std::ostream& os, bool& first) {
  if (first) {
    first = false;
  } else {
    os << ", ";
  }
}

void ssio_interface::sort(std::vector<std::pair<int, int>>& portID2size,
                          std::map<int, ssvport*>& vports) {
  int index = 0;
  portID2size.resize(vports.size());
  for (auto i : vports) {
    int id = i.first;
    int size = i.second->size();
    portID2size[index++] = std::make_pair(id, size);
  }
  std::sort(portID2size.begin(), portID2size.end(),
            [](std::pair<int, int>& left, std::pair<int, int>& right) {
              return left.second < right.second;
            });
}

// ----------------------- sslink ---------------------------------------------

std::string sslink::name() const {
  std::stringstream ss;
  ss << _orig->name() << "_to_" << _dest->name();
  return ss.str();
}

std::string sslink::gams_name(int config) const {
  std::stringstream ss;
  ss << _orig->gams_name(config) << "_" << _dest->gams_name(config);
  return ss.str();
}

sslink* sslink::getCycleLink() {
  ssnode* n = this->dest();
  for (auto I = n->out_links().begin(), E = n->out_links().end(); I != E; ++I) {
    sslink* dlink = *I;
    if (dlink->dest() == this->orig()) {
      return dlink;
    }
  }
  return NULL;
}

std::string sslink::gams_name(int st, int end) const {
  std::stringstream ss;
  ss << _orig->gams_name(st) << "_" << _dest->gams_name(end);
  return ss.str();
}

// ---------------------- ssswitch --------------------------------------------

void parse_list_of_ints(std::istream& istream, std::vector<int>& int_vec) {
  string cur_cap;
  while (getline(istream, cur_cap, ' ')) {
    if (cur_cap.empty()) {
      continue;
    }
    int val;
    istringstream(cur_cap) >> val;
    int_vec.push_back(val);
  }
}

void SubModel::parse_io(std::istream& istream) {
  string param, value, portstring;

  while (istream.good()) {
    if (istream.peek() == '[') break;  // break out if done

    ModelParsing::ReadPair(istream, param, value);
    std::stringstream ss(param);
    getline(ss, param, ' ');

    getline(ss, portstring);  // port num
    int port_num;
    istringstream(portstring) >> port_num;

    std::vector<int> int_vec;

    std::stringstream ssv(value);

    int is_input = -1;
    if (ModelParsing::StartsWith(param, "VPORT")) {
      assert(0 &&
             "VPORT_IN/VPORT_OUT Depricated, switch to PORT_IN/PORT_OUT\n"
             "delete \":0\" \":1\" from port descriptions");
    }
    if (ModelParsing::StartsWith(param, "PORT_IN")) {
      parse_list_of_ints(ssv, int_vec);
      is_input = 1;
    } else if (ModelParsing::StartsWith(param, "PORT_OUT")) {
      parse_list_of_ints(ssv, int_vec);
      is_input = 0;
    }
    if (is_input != -1) {
      ssvport* nvp = add_vport(is_input, port_num);
      // Connect nodes to the vector, and also determine x/y
      nvp->set_port_vec(int_vec);
      int avgx = 0;
      int avgy = 0;
      for (int i : int_vec) {
        if (_io_map[is_input].count(i) == 0) {
          cout << "Error: " << (is_input ? "Input" : "Output") << " port " << i
               << " is not available!\n";
          assert(0);
        }
        ssnode* n = _io_map[is_input][i];
        if (is_input)
          nvp->add_link(n);
        else
          n->add_link(nvp);
        avgx += n->x();
        avgy += n->y();
      }
      avgx /= int_vec.size();
      avgy /= int_vec.size();

      if (avgx <= 1) avgx = -1;
      if (avgx >= sizex() - 1) avgx = sizex();

      if (avgy <= 1) avgy = -1;
      if (avgy >= sizey() - 1) avgy = sizey();

      bool changed = true;
      while (changed) {
        changed = false;
        for (auto alt_port : _vport_list) {
          if (avgx == alt_port->x() && avgy == alt_port->y()) {
            avgx += 1;
            changed = true;
            break;
          }
        }
      }
      nvp->setXY(avgx, avgy);
    }
  }
}

bool parseInt(std::string param, string value, const char* param_name, int& i) {
  if (ModelParsing::StartsWith(param, param_name)) {
    istringstream(value) >> i;
    return true;
  }
  return false;
}

// ------------------------ submodel impl -------------------------------------

SubModel::SubModel(std::istream& istream, FuModel* fuModel, bool multi_config) {
  string param, value;

  bool should_read = true;

  // parameters used here for initialization:
  int switch_outs = 2, switch_ins = 2, bwm = 1;
  double bwmfrac = 0.0;

  int temp_width = 0, temp_height = 0;  // size of temporal region
  int temp_x = 0, temp_y = 0;           // location of temporal region

  int skip_diag_dist = 0, skip_hv_dist = 0, skip_delay = 1;

  PortType portType = PortType::opensp;

  while (istream.good()) {
    if (istream.peek() == '[') break;  // break out if done

    if (should_read) ModelParsing::ReadPair(istream, param, value);
    should_read = true;

    parseInt(param, value, "width", _sizex);
    parseInt(param, value, "height", _sizey);
    parseInt(param, value, "outs_per_switch", switch_outs);
    parseInt(param, value, "ins_per_switch", switch_ins);
    parseInt(param, value, "bwm", bwm);
    parseInt(param, value, "temporal_x", temp_x);
    parseInt(param, value, "temporal_y", temp_y);
    parseInt(param, value, "temporal_width", temp_width);
    parseInt(param, value, "temporal_height", temp_height);

    parseInt(param, value, "skip_diag_dist", skip_diag_dist);
    parseInt(param, value, "skip_hv_dist", skip_hv_dist);
    parseInt(param, value, "skip_delay", skip_delay);

    if (ModelParsing::StartsWith(param, "io_layout")) {
      ModelParsing::trim(value);
      if (ModelParsing::StartsWith(value, "open_splyser")) {
        portType = PortType::opensp;
      } else if (ModelParsing::StartsWith(value, "every_switch")) {
        portType = PortType::everysw;
      } else if (ModelParsing::StartsWith(value, "three_sides_in")) {
        portType = PortType::threein;
      } else if (ModelParsing::StartsWith(value, "three_in_two_out")) {
        portType = PortType::threetwo;
      } else {
        cerr << "io_layout parameter: \"" << value << "\" not recognized\n";
        assert(0);
      }
    } else if (ModelParsing::StartsWith(param, "bw_extra")) {
      istringstream(value) >> bwmfrac;
    } else if (ModelParsing::StartsWith(param, "SS_LAYOUT")) {
      // defining switch capability

      ModelParsing::trim(value);

      // std::cout << "CGRA SIZE: " << _sizex << ", " << _sizey << "\n";
      build_substrate(_sizex, _sizey);

      if (value.compare("FULL") == 0) {
        for (int j = 0; j < _sizey; j++) {
          string line, fustring;
          getline(istream, line);

          stringstream ss(line);

          for (int i = 0; i < _sizex; i++) {
            getline(ss, fustring, ' ');

            if (fustring.length() == 0) {
              --i;
              continue;
            }

            for(auto* fu : _fu_list) {
              if(fu->x()==i && fu->y()==j) {
                fu->setFUDef(fuModel->GetFUDef(fustring));
              }
            }
          }
        }

      } else {
        cerr << "Unsupported FU Initialization Type\n";
      }
    }
  }

  connect_substrate(_sizex, _sizey, portType, switch_ins, switch_outs, multi_config,
                    temp_x, temp_y, temp_width, temp_height, skip_hv_dist, skip_diag_dist,
                    skip_delay);
}

// Graph of the configuration or substrate
void SubModel::PrintGraphviz(ostream& ofs) {
  ofs << "Digraph G { \n";

  // switchesnew_sched
  for(auto* sw : _switch_list) {
    // ofs << switches[i][j]->name() <<"[ label = \"Switch[" << i << "][" << j << "]\"
    // ];\n";

    // output links
    for (auto& elem : sw->out_links()) {
      const ssnode* dest_node = elem->dest();  // FUs and output nodes
      ofs << sw->name() << " -> " << dest_node->name() << ";\n";
    }
  }

  // fus
  for(auto* fu : _fu_list) {
    for (auto& elem : fu->out_links()) {
      const ssnode* dest_node = elem->dest();  // Output link of each FU
      ofs << fu->name() << " -> " << dest_node->name() << ";\n";
    }
  }

  // Input nodes
  // for (unsigned i = 0; i < _inputs.size(); ++i) {
  //  //ofs << _inputs[i]->name() <<"[ label = \"IPort[" << i << "]\" ];\n";
  //  for(auto &elem : _inputs[i]->out_links()) {
  //    const ssnode* dest_node = elem->dest();       //Dest nodes for input ndoes are
  //    switches ofs << _inputs[i]->name() << " -> " << dest_node->name() << ";\n";
  //  }
  //
  //}

  /*
  for (unsigned i = 0; i < outputs.size(); ++i) {
    ofs << outputs[i]->name() <<"[ label = \"OPort[" << i << "]\" ];\n";
    ssnode::const_iterator I = outputs[i]->ibegin(), E = outputs[i]->iend();
    for(;I!=E;++I) {
      const ssnode* orig_node = (*I)->orig();
      ofs << orig_node->name() << " -> " << outputs[i]->name() << ";\n";
    }
  }*/

  ofs << "}\n";
}

void SubModel::clear_all_runtime_vals() {
  for (ssnode* n : _node_list) {
    n->reset_runtime_vals();
  }
}

namespace SS_CONFIG {

template <>
std::vector<ssfu*>& SubModel::nodes() {
  return _fu_list;
}
template <>
std::vector<ssvport*>& SubModel::nodes() {
  return _vport_list;
}

template <int is_input, typename T>
void SubModel::PrintGamsIO(std::ostream& os) {
  bool first = true;
  auto& _ssio_interf = this->_ssio_interf;
  std::vector<T*>& nodes_ = this->nodes<T*>();
  for (auto& elem : _ssio_interf.vports_map[is_input]) {
    int i = 0;
    // for each elem in the vector
    for (auto port : elem.second->port_vec()) {
      assert((unsigned)port >= nodes_.size() && "TOO HIGH OP INDEX");
      if (first)
        first = false;
      else
        os << ", ";
      // os << "ip" << elem.first << "." << nodes_[port]->gams_name(0) << " " << i + 1;
      // //no config supp.
    }
  }
}

}  // namespace SS_CONFIG

int dist_grid(int x, int y) {
  int dist = abs(x) + abs(y) + 1;
  if (x < 0) dist += 1;
  if (y < 0) dist += 1;
  if (x > 0 && y > 0) dist -= 1;
  return dist;
}

int dist_switch(int x, int y) {
  int dist = abs(x) + abs(y) + 1;
  if (x < 0) dist -= 1;
  if (y < 0) dist -= 1;
  return dist;
}

void SubModel::PrintGamsModel(ostream& ofs,
                              unordered_map<string, pair<ssnode*, int>>& node_map,
                              unordered_map<string, pair<sslink*, int>>& link_map,
                              unordered_map<string, pair<ssswitch*, int>>& switch_map,
                              unordered_map<string, pair<bool, int>>& port_map,
                              int n_configs) {
  // int in each maps is the configuration num
  // string -- name of node and position

  // --------------------------- First, print the node sets ------------------------
  ofs << "$onempty\n";
  ofs << "Sets\n";
  ofs << "n \"Hardware Nodes\"\n /";
  bool first = true;

  for (int config = 0; config < n_configs; ++config) {
    // fus
    for(auto* fu : _fu_list) {
      CINF(ofs, first);
      ofs << fu->gams_name(config);
      node_map[fu->gams_name(config)] = make_pair(fu, config);
    }
    // inputs
    // TODO: Fix me or eliminate == no more inputs
    // ofs << "\n";
    // for (unsigned i = 0; i < _inputs.size(); ++i) {
    //  ofs << ", " << _inputs[i]->gams_name(config);
    //  node_map[_inputs[i]->gams_name(config)] = make_pair(_inputs[i], config);
    //}

    // outputs
    // TODO: Fix me or eliminate == no more inputs

    // ofs << "\n";
    // for (unsigned i = 0; i < _outputs.size(); ++i) {
    //  ofs << ", " << _outputs[i]->gams_name(config);
    //  node_map[_outputs[i]->gams_name(config)] = make_pair(_outputs[i], config);
    //}
  }
  ofs << "/\n";

  // --------------------------- next, print the capabilility sets
  // ------------------------ input nodes
  /// TODO: Fix me or eliminate == no more inputs

  // first = true;
  // ofs << "inN(n) \"Input Nodes\"\n /";

  // for (int config = 0; config < n_configs; ++config) {
  //  for (unsigned i = 0; i < _inputs.size(); ++i) {
  //    CINF(ofs, first);
  //    ofs << _inputs[i]->gams_name(config);
  //  }
  //}
  // ofs << "/\n";

  ////output nodes
  // first = true;
  // ofs << "outN(n) \"Output Nodes\"\n /";

  // for (int config = 0; config < n_configs; ++config) {
  //  for (unsigned i = 0; i < _outputs.size(); ++i) {
  //    CINF(ofs, first);
  //    ofs << _outputs[i]->gams_name(config);
  //  }
  //}
  // ofs << "/\n";

  // total capabilities
  for (int i = 2; i < SS_NUM_TYPES; ++i) {
    ss_inst_t ss_inst = (ss_inst_t)i;

    ofs << name_of_inst(ss_inst) << "N(n) /";

    first = true;
    for (int config = 0; config < n_configs; ++config) {
      for(auto* fu : _fu_list) {
        if (fu->fu_def() == nullptr || fu->fu_def()->is_cap(ss_inst)) {
          CINF(ofs, first);
          ofs << fu->gams_name(config);  // Each FU in the grid
        }
      }
    }

    ofs << "/\n";
  }

  // create the kindN Set
  ofs << "kindN(K,n) \"Capabilities of a Node\" \n";

  // -------------------------- print the ports ----------------------------
  ofs << "pn \"Port Interface Declarations\"\n /";

  //  int num_port_interfaces=_ssio_interf.in_vports.size() + _ssio_interf.out_

  // Declare Vector Ports
  first = true;

  for (int io = 0; io < 2; ++io) {
    std::string prfx[2] = {"op", "ip"};
    for (auto& elem : _ssio_interf.vports_map[io]) {
      CINF(ofs, first);
      ofs << "ip" << elem.first;
      port_map[prfx[io] + std::to_string(elem.first)] = make_pair(true, elem.first);
    }
  }

  if (first == true) {
    ofs << "pnXXX";
  }
  ofs << "/\n";

  // --------------------------- print the switches  ------------------------
  ofs << "r \"Routers (switches)\"\n /";
  first = true;

  for (int config = 0; config < n_configs; ++config) {
    for(auto* sw : _switch_list) {
      CINF(ofs, first);
      ofs << sw->gams_name(config);
    }

    /*
    //Inputs and outputs are also switches
    //inputs
    //TODO: FIXME or eleiminate, no more inputs
    ofs << "\n";
    for (unsigned i = 0; i < _inputs.size(); ++i) {
        ofs << ", " << _inputs[i]->gams_name(config);
        node_map[_inputs[i]->gams_name(config)]=make_pair(&_inputs[i],config);
    }
    //outputs
    ofs << "\n";
    for (unsigned i = 0; i < _outputs.size(); ++i) {
        ofs << ", " << _outputs[i]->gams_name(config);
        node_map[_outputs[i]->gams_name(config)]=make_pair(&_outputs[i],config);
    }
    */
  }

  ofs << "/\n";

  ofs << "l \"Links\"\n /";
  first = true;
  //_switches
  for (int config = 0; config < n_configs; ++config) {
    for(auto* sw : _switch_list) {
      for (auto& elem : sw->out_links()) {
        CINF(ofs, first);
        ofs << elem->gams_name(config);
        link_map[elem->gams_name(config)] = make_pair(elem, config);
      }
      switch_map[sw->gams_name(config)] =
          make_pair(sw, config);
    }

    // fus
    for(auto* fu : _fu_list) {
      for (auto& elem : fu->out_links()) {
        ofs << ", " << elem->gams_name(config);
        link_map[elem->gams_name(config)] = make_pair(elem, config);
      }
    }

    // inputs
    // TODO: FIxme or eliminate, no more inputs
    // for (unsigned i = 0; i < _inputs.size(); ++i) {
    //  for (auto &elem : _inputs[i]->out_links()) {
    //    ofs << ", " << elem->gams_name(config);
    //    link_map[elem->gams_name(config)] = make_pair(elem, config);
    //  }
    //}
  }
  ofs << "/;\n";

  // --------------------------- Enable the Sets ------------------------
  ofs << "kindN('Input', inN(n))=YES;\n";
  ofs << "kindN('Output', outN(n))=YES;\n";

  for (int i = 2; i < SS_NUM_TYPES; ++i) {
    ss_inst_t ss_inst = (ss_inst_t)i;
    ofs << "kindN(\'" << name_of_inst(ss_inst) << "\', " << name_of_inst(ss_inst)
        << "N(n))=YES;\n";
  }

  // Print Parameters
  ofs << "parameter\n";
  // Node Distances
  int config = 0;

  // ----------------------- Node Distances -----------------------------------
  first = true;
  ofs << "DIST(n,n) \"Node Distances\"\n /";
  for (int x1 = 0; x1 < _sizex; ++x1) {
    for (int y1 = 0; y1 < _sizey; ++y1) {
      // me to all func units
      for (int x2 = 0; x2 < _sizex; ++x2) {
        for (int y2 = 0; y2 < _sizey; ++y2) {
          if (x1 == x2 && y1 == y2) continue;
          CINF(ofs, first);

          //int d = dist_grid(x2 - x1, y2 - y1);
          //TODO: fixme or eliminate
          //ofs << _fus[x1][y1]->gams_name(config) << "." << _fus[x2][y2]->gams_name(config)
          //    << " " << d;
        }
      }
      // all inputs to me
      //    //TODO: FIXME or eleiminate, no more inputs

      // for (unsigned i = 0; i < _inputs.size(); ++i) {
      //  CINF(ofs, first);
      //  ssswitch *sw = static_cast<ssswitch *>(_inputs[i]->getFirstOutLink()->dest());
      //  int d = dist_switch(x1 - sw->x(), y1 - sw->y());
      //  ofs << _inputs[i]->gams_name(config) << "."
      //      << _fus[x1][y1]->gams_name(config) << " " << d;
      //}

      ////all outputs to me
      // for (unsigned i = 0; i < _outputs.size(); ++i) {
      //  CINF(ofs, first);
      //  ssswitch *sw = static_cast<ssswitch *>(_outputs[i]->getFirstInLink()->orig());
      //  int d = dist_switch(x1 - sw->x(), y1 - sw->y());
      //  ofs << _fus[x1][y1]->gams_name(config) << "."
      //      << _outputs[i]->gams_name(config) << " " << d;
      //}
      // ofs << "\n";
    }
  }
  ofs << "/\n";

  // ----------------------- Node Loc -------------------------------------
  ofs << "PXn(n) \" Position X \"\n /";
  first = true;
  for(auto* fu : _fu_list) {
    CINF(ofs, first);
    ofs << fu->gams_name(config) << " " << fu->x() * 2 + 1;
  }
  // TODO: Fixme or eliminate, no more inputs
  // for (unsigned i = 0; i < _inputs.size(); ++i) {
  //  CINF(ofs, first);
  //  ssswitch *sw = static_cast<ssswitch *>(_inputs[i]->getFirstOutLink()->dest());
  //  ofs << _inputs[i]->gams_name(config) << " " << sw->x() * 2;
  //}
  // for (unsigned i = 0; i < _outputs.size(); ++i) {
  //  CINF(ofs, first);
  //  ssswitch *sw = static_cast<ssswitch *>(_outputs[i]->getFirstInLink()->orig());
  //  ofs << _outputs[i]->gams_name(config) << " " << sw->x() * 2;
  //}
  // ofs << "/\n";

  ofs << "PYn(n) \" Position X \"\n /";
  first = true;
  for(auto* fu : _fu_list) {
    CINF(ofs, first);
    ofs << fu->gams_name(config) << " " << fu->y() * 2 + 1;
  }

  // TODO: Fixme or eliminate, no more inputs
  // for (unsigned i = 0; i < _inputs.size(); ++i) {
  //  CINF(ofs, first);
  //  ssswitch *sw = static_cast<ssswitch *>(_inputs[i]->getFirstOutLink()->dest());
  //  ofs << _inputs[i]->gams_name(config) << " " << sw->y() * 2;
  //}
  // for (unsigned i = 0; i < _outputs.size(); ++i) {
  //  CINF(ofs, first);
  //  ssswitch *sw = static_cast<ssswitch *>(_outputs[i]->getFirstInLink()->orig());
  //  ofs << _outputs[i]->gams_name(config) << " " << sw->y() * 2;
  //}
  // ofs << "/\n";

  // --------------------------- Print Port Interfaces --------------------
  ofs << "PI(pn,n) \"Port Interfaces\" /\n";
  // Declare Port to Node Mapping
  first = true;

  // TODO: fixme or eliminate
  // PrintGamsIO<1, ssinput>(ofs);
  // PrintGamsIO<0, ssoutput>(ofs);

  ofs << "/\n";

  // --------------------------- Now Print the Linkage ------------------------

  ofs << "Hnl(n,l) \"Node Outputs\" \n/";
  // fus
  first = true;

  for (int config = 0; config < n_configs; ++config) {
    for(auto* fu : _fu_list) {
      for (auto& elem : fu->out_links()) {
        CINF(ofs, first);
        ofs << fu->gams_name(config) << "." << elem->gams_name(config) << " 1";
      }
    }

    // inputs
    // TODO: fixme or eliminate, no more inputs
    // for (unsigned i = 0; i < _inputs.size(); ++i) {
    //  for (auto &elem: _inputs[i]->out_links()) {
    //    ofs << ", " << _inputs[i]->gams_name(config) << "." << elem->gams_name(config)
    //    << " 1";
    //  }
    //}

    // if (_multi_config) {
    //  //print loadslice and crosswitch links
    //  if (config != n_configs - 1) {
    //    //outputs
    //    for (unsigned i = 0; i < _outputs.size(); ++i) {
    //      for (auto &elem : _outputs[i]->out_links()) {
    //        ofs << ", " << _outputs[i]->gams_name(config) << "." <<
    //        elem->gams_name(config) << " 1";
    //      }
    //    }
    //  }

    //  //print final loadslice links
    //  //if (config == n_configs - 1) {
    //  //  for (auto &elem : _load_slice.in_links()) {
    //  //    ssnode *output = elem->orig();
    //  //    ofs << ", " << output->gams_name(config) << "." << elem->gams_name(config)
    //  << " 1";
    //  //  }
    //  //}
    //}
  }
  ofs << "/\n";

  ofs << "Hrl(r,l) \"Router Outputs\" \n/";
  first = true;

  for (int config = 0; config < n_configs; ++config) {
    for(auto* sw : _switch_list) {
      for (auto& elem : sw->out_links()) {
        CINF(ofs, first);
        ofs << sw->gams_name(config) << "." << elem->gams_name(config)
            << " 1";
      }
    }
  }
  ofs << "/\n";

  ofs << "Hln(l,n) \"Node Inputs\" \n/";
  // fus
  first = true;
  for (int config = 0; config < n_configs; ++config) {
    for(auto* fu : _fu_list) {
      for (auto& elem : fu->in_links()) {
        CINF(ofs, first);
        ofs << elem->gams_name(config) << "." << fu->gams_name(config) << " 1";
      }
    }
    // outputs
    // TODO: fixme or eliminate
    // for (unsigned i = 0; i < _outputs.size(); ++i) {
    //  for (auto &elem : _outputs[i]->in_links()) {
    //    ofs << ", " << elem->gams_name(config)
    //        << "." << _outputs[i]->gams_name(config) << " 1";
    //  }
    //}
    // if (_multi_config) {
    //  if (config != 0) {
    //    //outputs
    //    for (unsigned i = 0; i < _inputs.size(); ++i) {
    //      for (auto &elem: _inputs[i]->in_links()) {
    //        ofs << ", " << elem->gams_name(config - 1, config)
    //            << "." << _inputs[i]->gams_name(config) << " 1";
    //      }
    //    }
    //  }
    //  if (config == 0) {
    //    for (auto &elem : _load_slice.out_links()) {
    //      ssnode *input = elem->dest();
    //      ofs << ", " << elem->gams_name(config) << "." << input->gams_name(config) << "
    //      1";
    //    }
    //  }
    //}
  }

  ofs << "/\n";

  ofs << "Hlr(l,r) \"Router Inputs\" \n/";
  first = true;
  for (int config = 0; config < n_configs; ++config) {
    for(auto* sw : _switch_list) {
      for (auto& elem : sw->in_links()) {
        CINF(ofs, first);
        ofs << elem->gams_name(config) << "." << sw->gams_name(config) << " 1";
      }
    }
  }
  ofs << "/;\n";
}

SubModel::SubModel(int x, int y, PortType pt, int ips, int ops, bool multi_config) {
  build_substrate(x, y);
  connect_substrate(x, y, pt, ips, ops, multi_config, 0, 0, 0, 0, 0, 0, 1);
}

void SubModel::build_substrate(int sizex, int sizey) {
  _sizex = sizex;
  _sizey = sizey;

  // Create FU array
  _fus.resize(_sizex);

  // Iterate each x vector -- vector of ssfu objects
  for (unsigned x = 0; x < _fus.size(); x++) {
    _fus[x].resize(_sizey);
    for (unsigned y = 0; y < (unsigned)_sizey; ++y) {
      add_fu(x, y);
    }
  }

  // Create Switch array
  _switches.resize(_sizex + 1);
  for (unsigned x = 0; x < _switches.size(); x++) {
    _switches[x].resize(_sizey + 1);
    for (unsigned y = 0; y < (unsigned)_sizey + 1; ++y) {
      add_switch(x, y);
    }
  }
}

void ssnode::setup_routing_memo_node(int i, int in_off, int bitwidth_log) {
  if(_in_links.size()==0 && !is_input()) return;

  std::vector<std::pair<int, sslink*>>& links =
      _routing_memo.at(i * 4 * MAX_SUBNETS + in_off * 4 + bitwidth_log);
  int slots = 1;
  for (int b = 0; b < bitwidth_log; ++b) slots *= 2;

  int in_slots;
  int in_index;
  if (_in_links.size() > 0) {
    sslink* inlink = _in_links[i];
    in_slots = inlink->bitwidth() / 8;
    in_index = inlink->in_index();
    // cout << inlink->name() << ":" << in_off << ", size:" << slots << " -- ";

  } else {  // no inputs (like an input ssvport)
    in_slots = bitwidth() / 8;
    in_index = 0;
    // cout << "any for " << name() << ":" << in_off << ", size:" << slots << "--";
  }

  // if(_out_links.size()==0) {
  //  cout << "  NO B/C OUTLINKS IS 0 \n";
  //  return;
  //}

  for (sslink* outlink : _out_links) {
    // start at each output offset
    for (int out_off = 0; out_off < outlink->bitwidth() / 8; ++out_off) {
      bool failed = false;

      for (int i = 0; i < slots; ++i) {
        int out_slot = (out_off + i) % (outlink->bitwidth() / 8);
        int in_slot = (in_off + i) % (in_slots);
        if (!_subnet_table[outlink->out_index()][out_slot][in_index][in_slot]) {
          failed = true;
          break;
        }
      }

      if (!failed) {
        // cout << outlink->name() << ":" << out_off << " ";

        links.push_back(make_pair(out_off, outlink));
      }
    }
  }
  // cout << "\n";
}

void ssnode::setup_default_routing_table(sslink* inlink, sslink* outlink) {
  // By default we just want each corresponding output + one decomposable
  // bitwidth away

  int in_slots;
  int in_index;
  if (inlink) {
    in_slots = inlink->bitwidth() / 8;
    in_index = inlink->in_index();
  } else {
    in_slots = bitwidth() / 8;
    in_index = 0;
  }

  int min_slots = std::min(in_slots, outlink->bitwidth() / 8);

  for (int out_off = 0; out_off < outlink->bitwidth() / 8; out_off += min_slots) {
    for (int in_off = 0; in_off < in_slots; in_off += min_slots) {
      // Now we've created a square region.
      // first, avoid connecting multiple subnetworks to one output if the
      // switch is not decomposable at all
      if (outlink->decomp_bitwidth() == outlink->bitwidth() && in_off != 0) continue;

      // first connect same subnet
      for (int i = 0; i < min_slots; ++i) {
        _subnet_table.at(outlink->out_index())
            .at(out_off + i)
            .at(in_index)
            .at(in_off + i) = true;
      }

      // If its an FU, don't subnet hop
      if (dynamic_cast<ssfu*>(outlink->orig()) ||
          dynamic_cast<ssvport*>(outlink->orig())) {
        continue;
      }

      // If we aren't also changing bitwidths, lets allow some subnet connections
      if (in_slots != outlink->bitwidth() / 8) continue;

      for (int i = 0; i < min_slots; ++i) {
        int out = out_off + ((i + outlink->decomp_bitwidth() / 8) % min_slots);
        _subnet_table.at(outlink->out_index()).at(out).at(in_index).at(in_off + i) =
            true;
      }
    }
  }
}

// Add blank space for a given link input
void ssnode::create_blank_subnet_table_input(sslink* inlink) {
  for (sslink* outlink : _out_links) {
    for (int o_sub = 0; o_sub < MAX_SUBNETS; ++o_sub) {
      _subnet_table[outlink->out_index()][o_sub].resize(
          std::max(1, (int)_in_links.size()));

      create_blank_subnet_table(outlink,o_sub,inlink);    
    }
    setup_default_routing_table(inlink, outlink);
  }
}

// Subnet table layout
// [output][slot][input][slot]

void ssnode::create_blank_subnet_table(sslink* outlink, int o_sub, sslink* inlink) {
  _subnet_table[outlink->out_index()][o_sub][inlink->in_index()].resize(MAX_SUBNETS);
}

void ssnode::create_blank_subnet_table(sslink* outlink, int o_sub) {
  _subnet_table[outlink->out_index()][o_sub].resize(
      std::max(1, (int)_in_links.size()));
  for (sslink* inlink : _in_links) {
    create_blank_subnet_table(outlink,o_sub,inlink);
  }
  if (_in_links.size() == 0) {
    _subnet_table[outlink->out_index()][o_sub][0].resize(MAX_SUBNETS);
  }
}


void ssnode::create_blank_subnet_table(sslink* outlink) {
  _subnet_table[outlink->out_index()].resize(MAX_SUBNETS);
  for (int o_sub = 0; o_sub < MAX_SUBNETS; ++o_sub) {
    create_blank_subnet_table(outlink,o_sub);
  }

  for (sslink* inlink : _in_links) {
    setup_default_routing_table(inlink, outlink);
  }
  if (_in_links.size() == 0) {
    setup_default_routing_table(nullptr, outlink);
  }

}

void ssnode::setup_routing_memo() {

  //if (_subnet_table.size() == 0) {
  //  assert(0 && "depricated block");
  //  // Allocate the space for the table
  //  _subnet_table.resize(_out_links.size());
  //  for (sslink* outlink : _out_links) {
  //    create_blank_subnet_table(outlink);
  //  }

  //  for (sslink* outlink : _out_links) {
  //    for (sslink* inlink : _in_links) {
  //      setup_default_routing_table(inlink, outlink);
  //    }
  //    if (_in_links.size() == 0) {
  //      setup_default_routing_table(nullptr, outlink);
  //    }
  //  }
  //}

  // for(sslink* outlink : _out_links) {
  //  for(sslink* inlink : _in_links) {
  //    cout << inlink->name() << " -> " << outlink->name() << "\n";
  //    for(int o_sub = 0; o_sub < MAX_SUBNETS; ++o_sub) {
  //      for(int i_sub = 0; i_sub < MAX_SUBNETS; ++i_sub) {
  //        cout << _subnet_table[outlink->out_index()][o_sub][inlink->in_index()][i_sub]
  //        << " ";
  //      }
  //      cout << "\n";
  //    }
  //  }
  //}

  // Now, no matter what _subnet table will be filled
  // Now lets memo-ize it so that the scheduler isn't dog slow
  // allocate space for the memo table
  _routing_memo.clear();
  _routing_memo.resize(std::max(1, (int)_in_links.size()) * _bitwidth / 8 * 4);
  

  for (int i = 0; i < std::max(1, (int)_in_links.size()); ++i) {
    for (int slot = 0; slot < 8; ++slot) {
      for (int bitwidth = 0; bitwidth <= 3; ++bitwidth) {
        setup_routing_memo_node(i, slot, bitwidth);
      }
    }
  }
}

// Group Nodes/Links and Set IDs
// This should be done after all the links are added
void SubModel::post_process() {
  //_node_list.clear();
  _link_list.clear();

  for (auto elem : _vport_list)  elem->agg_elem(_node_list, _link_list);
  for (auto elem : _fu_list)     elem->agg_elem(_node_list, _link_list);
  for (auto elem : _switch_list) elem->agg_elem(_node_list, _link_list);

  // fix the ids since we reordered the nodes/links
  //fix_node_id(); 
  fix_link_id(); 

  // Setup the subnetwork topology lookup tables for each node if it has not been
  // setup already during configuration.
  for (auto node : _node_list) {
    node->setup_routing_memo();
  }

  _ssio_interf.fill_vec();
}

void SubModel::connect_substrate(int _sizex, int _sizey, PortType portType, int ips,
                                 int ops, bool multi_config, int temp_x, int temp_y,
                                 int temp_width, int temp_height, int skip_hv_dist,
                                 int skip_diag_dist, int skip_delay) {
  {
    const int di[] = {0, 1, 1, 0};
    const int dj[] = {0, 0, 1, 1};
    const SwitchDir::DIR dir[] = {SwitchDir::SE, SwitchDir::SW, SwitchDir::NW,
                                  SwitchDir::NE};

    const int t_di[] = {0, 1, 0};
    const int t_dj[] = {0, 0, 1};
    const SwitchDir::DIR t_dir[] = {SwitchDir::NW, SwitchDir::NE, SwitchDir::SW};
    // first connect switches to FUs
    for (int i = 0; i < _sizex; i++) {
      for (int j = 0; j < _sizey; j++) {
        for (int k = 0; k < 4; ++k)
          _switches[i + di[k]][j + dj[k]]->add_link(_fus[i][j])->setdir(dir[k]);

        // output from FU -- SE
        _fus[i][j]->add_link(_switches[i + 1][j + 1])->setdir(SwitchDir::SE);

        // For temporal region, lets add some extra outputs!
        if (i >= temp_x && i < temp_x + temp_width && j >= temp_y &&
            j < temp_y + temp_height) {
          for (int k = 0; k < 3; ++k)
            _fus[i][j]->add_link(_switches[i + t_di[k]][j + t_dj[k]])->setdir(t_dir[k]);
        }
      }
    }
  }

  // Now Switches to eachother
  {
    const int di[] = {-1, 0, 1, 0};
    const int dj[] = {0, -1, 0, 1};
    const SwitchDir::DIR dir[] = {SwitchDir::W, SwitchDir::N, SwitchDir::E, SwitchDir::S};
    for (int i = 0; i < _sizex + 1; i++) {
      for (int j = 0; j < _sizey + 1; j++) {
        for (int k = 0; k < 4; ++k) {
          int _i = i + di[k];
          int _j = j + dj[k];
          if (_i >= 0 && _i <= _sizex && _j >= 0 && _j <= _sizey)
            _switches[i][j]->add_link(_switches[_i][_j])->setdir(dir[k]);
        }

        // crazy diagonals
        //@Jian, feel free to condense if you want : )
        ssswitch* startItem = _switches[i][j];

        if (skip_diag_dist > 0) {
          int d = skip_diag_dist;
          int l = skip_delay;
          if (i - d >= 0 && j - d >= 0) {
            startItem->add_link(_switches[i - d][j - d])
                ->setdir(SwitchDir::NW2)
                ->set_lat(l);
          }
          if (i - d >= 0 && j + d < _sizey) {
            startItem->add_link(_switches[i - d][j + d])
                ->setdir(SwitchDir::SW2)
                ->set_lat(l);
          }
          if (i + d < _sizex && j - d >= 0) {
            startItem->add_link(_switches[i + d][j - d])
                ->setdir(SwitchDir::NE2)
                ->set_lat(l);
          }
          if (i + d < _sizex && j + d < _sizey) {
            startItem->add_link(_switches[i + d][j + d])
                ->setdir(SwitchDir::SE2)
                ->set_lat(l);
          }
        }

        // Crazy Jumps
        if (skip_hv_dist > 0) {
          int d = skip_hv_dist;
          int l = skip_delay;
          if (i - d >= 0) {
            startItem->add_link(_switches[i - d][j])->setdir(SwitchDir::W2)->set_lat(l);
          }
          if (j + d < _sizey) {
            startItem->add_link(_switches[i][j + d])->setdir(SwitchDir::S2)->set_lat(l);
          }
          if (j - d >= 0) {
            startItem->add_link(_switches[i][j - d])->setdir(SwitchDir::N2)->set_lat(l);
          }
          if (i + d < _sizex) {
            startItem->add_link(_switches[i + d][j])->setdir(SwitchDir::E2)->set_lat(l);
          }
        }
      }
    }
  }

  if (portType == PortType::threein || portType == PortType::threetwo) {
    // Three sides have inputs
    bool bonus_middle = true;

    int in_index = 0;
    for (int sw = 0; sw < _sizey; sw++) {
      for (int p = 0; p < ips; p++) {
        add_input(in_index++, _switches[0][_sizey - sw]);
      }
    }

    for (int sw = 0; sw < _sizex; sw++) {
      for (int p = 0; p < ips; p++) {
        add_input(in_index++, _switches[sw][0]);
      }
    }

    for (int sw = 0; sw < _sizey; sw++) {
      for (int p = 0; p < ips; p++) {
        add_input(in_index++, _switches[_sizex][sw]);
      }
    }

    if (bonus_middle) {  // TODO: make an option for this
      // cout << "bonus inputs: ";
      for (int sw = 0; sw < _sizex; sw++) {
        for (int p = 0; p < ips; p++) {
          // cout << in_index << " ";
          // assert((unsigned)in_index < _inputs.size());
          add_input(in_index++, _switches[sw + 1][_sizey]);
        }
      }
    }

    if (portType == PortType::threein) {
      // Switches to Outputs

      int out_index = 0;
      for (int sw = 0; sw < _sizex; sw++) {
        for (int p = 0; p < ops; p++) {
          add_output(out_index++, _switches[sw + 1][_sizey]);
        }
      }

    } else if (portType == PortType::threetwo) {
      int out_index = 0;
      for (int sw = 0; sw < _sizex; sw++) {
        for (int p = 0; p < ops; p++) {
          add_output(out_index++, _switches[sw + 1][_sizey]);
        }
      }
      for (int sw = 0; sw < _sizey; sw++) {
        for (int p = 0; p < ops; p++) {
          add_output(out_index++, _switches[_sizex][_sizey - sw - 1]);
        }
      }
    }

  } else if (portType == PortType::everysw) {  // all switches have inputs/outputs
    int inum = 0;
    int onum = 0;

    for (int i = 0; i < _sizex + 1; i++) {
      for (int j = 0; j < _sizey + 1; j++) {
        for (int p = 0; p < ips; p++) {
          add_input(inum++, _switches[i][_sizey - j]);
        }
        for (int p = 0; p < ops; p++) {
          add_output(onum++, _switches[i][_sizey - j]);
        }
      }
    }
  }

  if (multi_config) {
    printf("USING MULTI CONFIG (NOT REALLY SUPPORTED ANYMORE)\n");
  }

  _multi_config = multi_config;

  // The primitive temporal region that we are going to create just has local
  // connections to surrounding nodes.
  // TODO: FIXME: We need some way of specifying the max util in the config file

  for (int i = temp_x; i < temp_x + temp_width; i++) {
    for (int j = temp_y; j < temp_y + temp_height; j++) {
      _fus[i][j]->set_max_util(64);

      if (i + 1 < temp_x + temp_width) {
        sslink* link = _fus[i][j]->add_link(_fus[i + 1][j]);
        link->setdir(SwitchDir::E);
        link->set_max_util(100);
      }
      if (i > temp_x) {
        sslink* link = _fus[i][j]->add_link(_fus[i - 1][j]);
        link->setdir(SwitchDir::W);
        link->set_max_util(100);
      }
      if (j + 1 < temp_y + temp_height) {
        sslink* link = _fus[i][j]->add_link(_fus[i][j + 1]);
        link->setdir(SwitchDir::N);
        link->set_max_util(100);
      }
      if (j > temp_y) {
        sslink* link = _fus[i][j]->add_link(_fus[i][j - 1]);
        link->setdir(SwitchDir::S);
        link->set_max_util(100);
      }
    }
  }

  for (int i = 0; i < _sizex; ++i) {
    for (int j = 0; j < _sizey; ++j) {
      auto link = _fus[i][j]->add_link(_fus[i][j]);
      link->setdir(SwitchDir::IP0);

      if (i < temp_x && i >= temp_x + temp_width && j < temp_y &&
          j >= temp_y + temp_height) {
        link->set_max_util(64);
      }
    }
  }
}

void ssio_interface::fill_vec() {
  for (int i = 0; i < 2; ++i) {
    vports_vec[i].resize(vports_map[i].size());
    int j = 0;
    for (auto& elem : vports_map[i]) vports_vec[i][j++] = elem;
    std::sort(vports_vec[i].begin(), vports_vec[i].end(),
              [](const ssio_interface::EntryType& a, const ssio_interface::EntryType& b) {
                return a.second->size() < b.second->size();
              });
  }
}
