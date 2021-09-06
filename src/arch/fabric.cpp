#include "dsa/arch/fabric.h"

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

#include "../utils/model_parsing.h"
#include "dsa/arch/sub_model.h"
#include "dsa/arch/visitor.h"
#include "dsa/debug.h"

using namespace dsa;
using namespace std;

// ----------------------- sslink ---------------------------------------------

bool sslink::flow_control() { return sink_->flow_control(); }
int sslink::bitwidth() { return sink_->datawidth(); }

std::string sslink::name() const {
  std::stringstream ss;
  ss << source_->name() << "_to_" << sink_->name();
  return ss.str();
}

int sslink::slots(int slot, int width) {
  /* To check the connectivity in O(1), here we apply a tricky idea:
   * Originally, subnet[i][j] indicates subnet `i' is connected to subnet `j'.
   * This is unfriendly to check a bulk of connectivity: when the width is `w', and we
   * want to go from `i' to `j', we need to check subnet[i+k][j+k] where k = 0..(w-1),
   * which is O(w).
   *
   * Here we first transform the meaning of subnet[i][delta], which means
   * it is able to go from `i' to `i+delta' subnet. If we want to check connectivity
   * between `i' and `j', under width `w', where delta=j-i, the check becomes:
   * subnet[i+k][delta], where k=0..(w-1), which is still O(w).
   *
   * Then two tricks together enables O(1) check:
   * 1. Transposing the matrix makes the check subnet[delta][i+k], this makes access
   * continuous.
   * 2. Squeezing the inner dimension of subnet to bit representation, so that we can
   * check the one's in O(1) by bit operation.
   * */
  uint64_t res = 0;
  int n = subnet.size();
  auto f = [](int64_t a, int bits) {
    int full_mask = ~0ull >> (64 - bits);
    return (a & full_mask) == full_mask;
  };
  for (int i = 0; i < n; ++i) {
    if (slot + width <= n) {
      if (f(subnet[i] >> slot, width)) {
        res |= 1 << (i + slot) % n;
      }
    } else {
      int high = n - slot;
      int low = width - high;
      if (f(subnet[i] >> slot, high) && f(subnet[i], low)) {
        res |= 1 << (i + slot) % n;
      }
    }
  }
  return res;
}

sslink::~sslink() {
  auto f = [this](std::vector<sslink*>& links) {
    auto iter = std::find(links.begin(), links.end(), this);
    CHECK(iter != links.end()) << "Cannot find this link!";
    links.erase(iter);
  };
  f(source()->links_[0]);
  f(sink()->links_[1]);
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

void SpatialFabric::parse_io(std::istream& istream) {
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
      CHECK(0) << "VPORT_IN/VPORT_OUT Depricated, switch to PORT_IN/PORT_OUT\n"
               << "delete \":0\" \":1\" from port descriptions";
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
        CHECK(_io_map[is_input].count(i)) << "Error: " << (is_input ? "Input" : "Output")
                                          << " port " << i << " is not available!\n";
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
      nvp->x(avgx);
      nvp->y(avgy);
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

SpatialFabric::SpatialFabric(std::istream& istream,
                             const std::vector<Capability*>& fu_types) {
  string param, value;

  bool should_read = true;

  // parameters used here for initialization:
  int switch_outs = 2, switch_ins = 2, bwm = 1;
  double bwmfrac = 0.0;

  int temp_width = 0, temp_height = 0;  // size of temporal region
  int temp_x = 0, temp_y = 0;           // location of temporal region

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
        CHECK(false) << "io_layout parameter: \"" << value << "\" not recognized";
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
                  for (auto& type : fu_types) {
                    if (type->name == fustring) {
                      fu->fu_type_ = *type;
                    }
                  }
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

  connect_substrate(_sizex, _sizey, portType, switch_ins, switch_outs, temp_x, temp_y,
                    temp_width, temp_height);
}

// Graph of the configuration or substrate
void SpatialFabric::PrintGraphviz(ostream& os) {
  os << "Digraph G { \n";

  // switchesnew_sched
  for (auto* sw : switch_list()) {
    // os << switches[i][j]->name() <<"[ label = \"Switch[" << i << "][" << j << "]\"
    // ];\n";

    // output links
    for (auto& elem : sw->out_links()) {
      const ssnode* dest_node = elem->sink();  // FUs and output nodes
      os << sw->name() << " -> " << dest_node->name() << ";\n";
    }
  }

  // fus
  for (auto* fu : fu_list()) {
    for (auto& elem : fu->out_links()) {
      const ssnode* dest_node = elem->sink();  // Output link of each FU
      os << fu->name() << " -> " << dest_node->name() << ";\n";
    }
  }

  os << "}\n";
}

void SpatialFabric::Apply(adg::Visitor* visitor) {
  for (auto& elem : node_list()) {
    elem->Accept(visitor);
  }
}

void SpatialFabric::initialize_runtime_vals(std::map<int,  std::vector<int>>& node_dist, std::map<int,  std::vector<int>>& done, std::map<int,  std::vector<std::pair<int, sslink*>>> & came_from) {
  for (ssnode* n : _node_list) {
    node_dist[n->id()] = std::vector<int>(8, -1);
    came_from[n->id()] = std::vector<std::pair<int, sslink*>>(8);
    done[n->id()] = std::vector<int>(8, 0);
  }
}

SpatialFabric::SpatialFabric(int x, int y, PortType pt, int ips, int ops) {
  build_substrate(x, y);
  connect_substrate(x, y, pt, ips, ops, 0, 0, 0, 0);
}

void SpatialFabric::build_substrate(int sizex, int sizey) {
  _sizex = sizex;
  _sizey = sizey;

  // Iterate each x vector -- vector of ssfu objects
  for (int x = 0; x < sizex; x++) {
    for (int y = 0; y < _sizey; ++y) {
      auto fu = add_fu();
      fu->x(x);
      fu->y(y);
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
  CHECK(this != node) << "Cycle link is not allowed! " << id() << " " << node->id();
  auto& olinks = links_[0];
  olinks.push_back(link);

  link->subnet.resize(link->bitwidth() / link->source()->granularity());
  link->subnet[0] = ~0ull >> (64 - link->bitwidth());
  link->subnet[1] = ~0ull >> (64 - link->bitwidth());

  DSA_LOG(SUBNET) << link->subnet[0] << link->subnet[1] << "\n";

  node->links_[1].push_back(link);
  return link;
}

void SpatialFabric::connect_substrate(int _sizex, int _sizey, PortType portType, int ips,
                                      int ops, int temp_x, int temp_y, int temp_width,
                                      int temp_height) {
  auto fus = fu_list();
  auto sws = switch_list();

  {
    const int di[] = {0, 1, 1, 0};
    const int dj[] = {0, 0, 1, 1};

    const int t_di[] = {0, 1, 0};
    const int t_dj[] = {0, 0, 1};
    // first connect switches to FUs
    for (int i = 0; i < _sizex; i++) {
      for (int j = 0; j < _sizey; j++) {
        for (int k = 0; k < 4; ++k) {
          int x = i + di[k], y = j + dj[k];
          sws[x * (_sizey + 1) + y]->add_link(fus[i * _sizey + j]);
        }

        // output from FU -- SE
        fus[i * _sizey + j]->add_link(sws[(i + 1) * (_sizey + 1) + (j + 1)]);

        // For temporal region, lets add some extra outputs!
        if (i >= temp_x && i < temp_x + temp_width && j >= temp_y &&
            j < temp_y + temp_height) {
          for (int k = 0; k < 3; ++k) {
            int x = i + t_di[k], y = j + t_dj[k];
            fus[i * _sizey + j]->add_link(sws[x * (_sizey + 1) + y]);
          }
        }
      }
    }
  }

  // Now Switches to eachother
  {
    const int di[] = {-1, 0, 1, 0};
    const int dj[] = {0, -1, 0, 1};
    for (int i = 0; i < _sizex + 1; i++) {
      for (int j = 0; j < _sizey + 1; j++) {
        for (int k = 0; k < 4; ++k) {
          int _i = i + di[k];
          int _j = j + dj[k];
          if (_i >= 0 && _i <= _sizex && _j >= 0 && _j <= _sizey)
            sws[i * (_sizey + 1) + j]->add_link(sws[_i * (_sizey + 1) + _j]);
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

  // The primitive temporal region that we are going to create just has local
  // connections to surrounding nodes.
  // TODO: FIXME: We need some way of specifying the max util in the config file

  for (int i = temp_x; i < temp_x + temp_width; i++) {
    for (int j = temp_y; j < temp_y + temp_height; j++) {
      fus[i * _sizey + j]->max_util(64);

      const int di[] = {-1, 1, 0, 0};
      const int dj[] = {0, 0, 1, -1};

      for (int k = 0; k < 4; ++k) {
        int x = i + di[k];
        int y = j + dj[k];
        if (temp_x <= x && x < temp_x + temp_width && temp_y <= y &&
            y < temp_y + temp_height) {
          sslink* link = fus[i * _sizey + j]->add_link(fus[x * _sizey + y]);
          link->max_util(1 << 7);
        }
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
void SpatialFabric::post_process() {
  struct Aggreator : dsa::adg::Visitor {
    std::vector<sslink*>& links;
    Aggreator(std::vector<sslink*>& links_) : links(links_) { links.clear(); }
    void Visit(ssnode* node) override {
      for (auto& elem : node->out_links()) {
        int x = links.size();
        links.push_back(elem);
        links.back()->id(x);
      }
    }
  };

  Aggreator aggreator(_link_list);
  Apply(&aggreator);

  _ssio_interf.fill_vec();
}
