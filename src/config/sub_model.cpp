#include "sub_model.h"

#include <assert.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

#include "../utils/debug.h"
#include "model_parsing.h"

namespace pt = boost::property_tree;

using namespace SS_CONFIG;
using namespace std;

// ----------------------- sslink ---------------------------------------------

bool sslink::flow_control() { return _dest->flow_control(); }

std::string sslink::name() const {
  std::stringstream ss;
  ss << _orig->name() << "_to_" << _dest->name();
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

sslink::~sslink() {
  auto f = [this](std::vector<sslink*>& links) {
    auto iter = std::find(links.begin(), links.end(), this);
    assert(iter != links.end() && "Cannot find this link!");
    links.erase(iter);
  };
  f(orig()->links[0]);
  f(dest()->links[1]);
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
        for (auto alt_port : vport_list()) {
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

            for (auto elem : _node_list) {
              if (auto fu = dynamic_cast<ssfu*>(elem)) {
                if (fu->x() == i && fu->y() == j) {
                  fu->setFUDef(fuModel->GetFUDef(fustring));
                }
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

// Dump the Hardware Description in JSON -- void SubModel::DumpHwInJSON(const char* name)

// Graph of the configuration or substrate
void SubModel::PrintGraphviz(ostream& os) {
  os << "Digraph G { \n";

  // switchesnew_sched
  for (auto* sw : switch_list()) {
    // os << switches[i][j]->name() <<"[ label = \"Switch[" << i << "][" << j << "]\"
    // ];\n";

    // output links
    for (auto& elem : sw->out_links()) {
      const ssnode* dest_node = elem->dest();  // FUs and output nodes
      os << sw->name() << " -> " << dest_node->name() << ";\n";
    }
  }

  // fus
  for (auto* fu : fu_list()) {
    for (auto& elem : fu->out_links()) {
      const ssnode* dest_node = elem->dest();  // Output link of each FU
      os << fu->name() << " -> " << dest_node->name() << ";\n";
    }
  }

  os << "}\n";
}

void SubModel::clear_all_runtime_vals() {
  for (ssnode* n : _node_list) {
    n->reset_runtime_vals();
  }
}

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

SubModel::SubModel(int x, int y, PortType pt, int ips, int ops, bool multi_config) {
  build_substrate(x, y);
  connect_substrate(x, y, pt, ips, ops, multi_config, 0, 0, 0, 0, 0, 0, 1);
}

void SubModel::build_substrate(int sizex, int sizey) {
  _sizex = sizex;
  _sizey = sizey;

  // Iterate each x vector -- vector of ssfu objects
  for (int x = 0; x < sizex; x++) {
    for (int y = 0; y < _sizey; ++y) {
      auto fu = add_fu();
      fu->setXY(x, y);
    }
  }

  // Create Switch array
  for (int x = 0; x < sizex + 1; x++) {
    for (int y = 0; y < sizey + 1; ++y) {
      add_switch(x, y);
    }
  }
}

sslink* ssnode::add_link(ssnode* node) {
  sslink* link = new sslink(this, node);
  if (this == node) {
    if (_cycle_link) {
      DEBUG(CYCLELINK) << "two cycle links :\n"
                       << "source : " << link->orig()->nodeType() << "_"
                       << link->orig()->id() << ", "
                       << "sink : " << link->dest()->nodeType() << "_"
                       << link->dest()->id() << "\n"
                       << "[Warning] two cycle links, why?"
                       << "\n";
    }
    _cycle_link = link;
  }
  auto& olinks = links[0];
  olinks.push_back(link);

  link->subnet.resize(link->bitwidth() / 8);
  link->subnet[0] = ~0ull >> (64 - link->bitwidth());
  link->subnet[1] = ~0ull >> (64 - link->bitwidth());

  DEBUG(SUBNET) << link->subnet[0] << link->subnet[1] << "\n";

  node->links[1].push_back(link);
  return link;
}

void SubModel::connect_substrate(int _sizex, int _sizey, PortType portType, int ips,
                                 int ops, bool multi_config, int temp_x, int temp_y,
                                 int temp_width, int temp_height, int skip_hv_dist,
                                 int skip_diag_dist, int skip_delay) {
  auto fus = fu_list();
  auto sws = switch_list();

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
        for (int k = 0; k < 4; ++k) {
          int x = i + di[k], y = j + dj[k];
          sws[x * (_sizey + 1) + y]->add_link(fus[i * _sizey + j])->setdir(dir[k]);
        }

        // output from FU -- SE
        fus[i * _sizey + j]
            ->add_link(sws[(i + 1) * (_sizey + 1) + (j + 1)])
            ->setdir(SwitchDir::SE);

        // For temporal region, lets add some extra outputs!
        if (i >= temp_x && i < temp_x + temp_width && j >= temp_y &&
            j < temp_y + temp_height) {
          for (int k = 0; k < 3; ++k) {
            int x = i + t_di[k], y = j + t_dj[k];
            fus[i * _sizey + j]->add_link(sws[x * (_sizey + 1) + y])->setdir(t_dir[k]);
          }
        }
      }
    }
  }

  // Now Switches to eachother
  {
    const int di[] = {-1, 0, 1, 0};
    const int dj[] = {0, -1, 0, 1};
    const int cx[] = {-1, 1, 1, 1};
    const int cy[] = {-1, 1, -1, 1};
    const SwitchDir::DIR dir[] = {SwitchDir::W, SwitchDir::N, SwitchDir::E, SwitchDir::S};
    const SwitchDir::DIR dir2[] = {SwitchDir::NW2, SwitchDir::SW2, SwitchDir::NE2,
                                   SwitchDir::SE2};
    const SwitchDir::DIR dir3[] = {SwitchDir::W2, SwitchDir::S2, SwitchDir::N2,
                                   SwitchDir::E2};
    for (int i = 0; i < _sizex + 1; i++) {
      for (int j = 0; j < _sizey + 1; j++) {
        for (int k = 0; k < 4; ++k) {
          int _i = i + di[k];
          int _j = j + dj[k];
          if (_i >= 0 && _i <= _sizex && _j >= 0 && _j <= _sizey)
            sws[i * (_sizey + 1) + j]
                ->add_link(sws[_i * (_sizey + 1) + _j])
                ->setdir(dir[k]);
        }

        // crazy diagonals
        //@Jian, feel free to condense if you want : )
        ssswitch* startItem = sws[i * (_sizey + 1) + j];

        if (skip_diag_dist > 0) {
          int d = skip_diag_dist;
          int l = skip_delay;
          for (int k = 0; k < 4; ++k) {
            int x = i + cx[k] * d, y = j + cy[k] * d;
            int idx = x * (_sizey + 1) + y;
            if (idx >= 0 && idx < sws.size()) {
              startItem->add_link(sws[idx])->setdir(dir2[k])->set_lat(l);
            }
          }
        }

        // Crazy Jumps
        if (skip_hv_dist > 0) {
          int d = skip_hv_dist;
          int l = skip_delay;
          for (int k = 0; k < 4; ++k) {
            int x = i + di[k] * d, y = j + dj[k] * d;
            int idx = x * (_sizey + 1) + y;
            if (idx >= 0 && idx < sws.size()) {
              startItem->add_link(sws[idx])->setdir(dir3[k])->set_lat(l);
            }
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
        add_input(in_index++, sws[_sizey - sw]);
      }
    }

    for (int sw = 0; sw < _sizex; sw++) {
      for (int p = 0; p < ips; p++) {
        add_input(in_index++, sws[sw * (_sizey + 1)]);
      }
    }

    for (int sw = 0; sw < _sizey; sw++) {
      for (int p = 0; p < ips; p++) {
        add_input(in_index++, sws[_sizex * (_sizey + 1) + sw]);
      }
    }

    if (bonus_middle) {  // TODO: make an option for this
      // cout << "bonus inputs: ";
      for (int sw = 0; sw < _sizex; sw++) {
        for (int p = 0; p < ips; p++) {
          // cout << in_index << " ";
          // assert((unsigned)in_index < _inputs.size());
          add_input(in_index++, sws[(sw + 1) * (_sizey + 1) + _sizey]);
        }
      }
    }

    if (portType == PortType::threein) {
      // Switches to Outputs

      int out_index = 0;
      for (int sw = 0; sw < _sizex; sw++) {
        for (int p = 0; p < ops; p++) {
          add_output(out_index++, sws[sw + 1 * (_sizey + 1) + _sizey]);
        }
      }

    } else if (portType == PortType::threetwo) {
      int out_index = 0;
      for (int sw = 0; sw < _sizex; sw++) {
        for (int p = 0; p < ops; p++) {
          add_output(out_index++, sws[(sw + 1) * (_sizey + 1) + _sizey]);
        }
      }
      for (int sw = 0; sw < _sizey; sw++) {
        for (int p = 0; p < ops; p++) {
          add_output(out_index++, sws[_sizex * (_sizey + 1) + _sizey - sw - 1]);
        }
      }
    }

  } else if (portType == PortType::everysw) {  // all switches have inputs/outputs
    int inum = 0;
    int onum = 0;

    for (int i = 0; i < _sizex + 1; i++) {
      for (int j = 0; j < _sizey + 1; j++) {
        for (int p = 0; p < ips; p++) {
          add_input(inum++, sws[i * (_sizey + 1) + _sizey - j]);
        }
        for (int p = 0; p < ops; p++) {
          add_output(onum++, sws[i * (_sizey + 1) + _sizey - j]);
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
      fus[i * _sizey + j]->set_max_util(64);

      const int di[] = {-1, 1, 0, 0};
      const int dj[] = {0, 0, 1, -1};
      SwitchDir::DIR dir[] = {SwitchDir::E, SwitchDir::E, SwitchDir::N, SwitchDir::S};

      for (int k = 0; k < 4; ++k) {
        int x = i + di[k];
        int y = j + dj[k];
        if (temp_x <= x && x < temp_x + temp_width && temp_y <= y &&
            y < temp_y + temp_height) {
          sslink* link = fus[i * _sizey + j]->add_link(fus[x * _sizey + y]);
          link->set_max_util(1 << 7);
          link->setdir(dir[k]);
        }
      }
    }
  }

  for (int i = 0; i < _sizex; ++i) {
    for (int j = 0; j < _sizey; ++j) {
      auto link = fus[i * _sizey + j]->add_link(fus[i * _sizey + j]);
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

// Group Nodes/Links and Set IDs
// This should be done after all the links are added
void SubModel::post_process() {
  _link_list.clear();

  for (auto elem : _node_list) elem->agg_elem(_link_list);
  fix_id(_link_list);

  _ssio_interf.fill_vec();
}
