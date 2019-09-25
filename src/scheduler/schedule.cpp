#include "schedule.h"

#include <map>
#include <iostream>
#include <fstream>

#include "model_parsing.h"
#include "ssdfg.h"
#include "ssinst.h"
#include <assert.h>
#include <list>
#include <iomanip>
#include <unordered_set>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <set>
#include <exception>

using namespace std;
using namespace SS_CONFIG;
namespace pt = boost::property_tree;

//Scheduling Interface
extern "C" void libssscheduler_is_present() {}

void Schedule::clear_ssdfg() {
  if (_ssDFG) {
    delete _ssDFG;
    _ssDFG = nullptr;
  }
}

void Schedule::reset_simulation_state() {
  if(_ssDFG) {
    _ssDFG->reset_simulation_state();
  }
}

std::map<SS_CONFIG::ss_inst_t,int> Schedule::interpretConfigBits(int size,
    uint64_t* bits) {

  //Figure out if this configuration is real or not
  //NOTE: the first 9 characters of the configuration must spell filename
  //for this hack to work!
  if (strncmp((char *) bits, "filename:", 9) == 0) {
    char *c_bits = ((char *) bits) + 9;
    return interpretConfigBitsCheat(c_bits);
  } else {
    for (int i = 0; i < size; ++i) { //load in 64bit slices
      slices().write(i, bits[i]);
    }
    return map<SS_CONFIG::ss_inst_t, int>();
    //return interpretConfigBitsDedicated();
  }
}

std::map<SS_CONFIG::ss_inst_t,int> Schedule::interpretConfigBitsCheat(char* s) {
  std::map<SS_CONFIG::ss_inst_t,int> inst_histo;

  ifstream config_file;

  std::string filename = string("sched/")+string(s);
  config_file.open(filename.c_str());
  if(!config_file.good()) {
    filename = string(getenv("DFG_COMMON")) + string("/")+string(s);
    config_file.open(filename.c_str());
  } 

  if(!config_file.good()) {
    cout << "Could Not Open:" << s 
         << " at folder sched/ or $DFG_COMMON\n";
    assert(0);
  }

  static std::set<string> seen_sched;
  if(!seen_sched.count(filename)) {
    seen_sched.insert(filename);
    cout << "Using Schedule: \"" << filename << "\"\n";
  }

  boost::archive::text_iarchive ia(config_file);

  //I think this should work okay, its a little kludgey, but w/e
  Schedule sched2;
  ia >> BOOST_SERIALIZATION_NVP(sched2); //magic
  sched2._ssModel = _ssModel;
  *this = sched2;

  //Now lets patch up the schedule to get recover
  //vertex->node and edge->link mappings
  for(auto& ep : _edgeProp) {
    for(auto p : ep.links_ser) {
      int slot = p.first;
      int id = p.second;
      sslink* link = _ssModel->subModel()->link_list()[id]; 
      ep.links.emplace_back(slot,link);
    }
    for(auto p : ep.passthroughs_ser) {
      int slot = p.first;
      int id = p.second;
      ssnode* node = _ssModel->subModel()->node_list()[id]; 
      ep.passthroughs.emplace_back(slot,node);
    }
  } 


  for(int i = 0; i < (int)_nodeProp.size(); ++i) {
    auto& np = _nodeProp[i];
    ssnode* node = _ssModel->subModel()->node_list()[i];
    for(int slot = 0; slot < 8; ++slot) {
      for(auto elem : np.slots[slot].vertices) {
        _vertexProp[elem.first->id()].node = node;
      }
    }
  }

  for(auto dfg_inst : _ssDFG->nodes<SSDfgInst*>()) {
    auto inst=dfg_inst->inst();
    inst_histo[inst]+=1;
  }

  //Lets also just throw the node id at the dfg for now to make temporal
  //simulation work
  for(auto inst : _ssDFG->inst_vec()) {
    if(locationOf(inst)) {
      inst->set_node_id(locationOf(inst)->id());
    }
  }

  return inst_histo;
}

//TODO: Later we uncomment these bunch of codes to do real CGRA config.
//std::map<SS_CONFIG::ss_inst_t,int> Schedule::interpretConfigBitsDedicated() {
//  std::map<SS_CONFIG::ss_inst_t, int> inst_histo;
//
//  SwitchDir ssdir;
//  vector<vector<ssfu> > &fus = _ssModel->subModel()->fus();
//  SSDfgInst *dfg_inst;
//
//  map<ssnode *, map<SwitchDir::DIR, SwitchDir::DIR> > routeMap;
//  map<SSDfgNode *, vector<SwitchDir::DIR> > posMap;
//  map<ssnode *, SSDfgNode *> dfgnode_for;
//  _ssDFG = new SSDfg();
//
//  std::set<uint64_t> inputs_used;   //vector ports used
//  std::set<uint64_t> outputs_used;
//
//  _decode_lat_mis = slices().read_slice(IN_ACT_SLICE, 56, 63);
//
//  //Associating the DFG Nodes from the configuration bits, with the vector ports defined by the hardware
//  int start_bits_vp_mask = 0;
//  int total_bits_vp_mask = 0;
//  int slice = VP_MAP_SLICE_1;
//  for (auto &port_pair : _ssModel->subModel()->io_interf().in_vports) {
//    int i = port_pair.first; //index of port
//    std::vector<std::pair<int, std::vector<int> > > &port_m = port_pair.second;
//
//    //port mapping of 1 vector port - cgra_port_num: vector offset elements
//    total_bits_vp_mask += port_m.size();
//
//    if (start_bits_vp_mask < 64 && total_bits_vp_mask > 64) {
//      start_bits_vp_mask = port_m.size();
//      total_bits_vp_mask = port_m.size();
//      slice = VP_MAP_SLICE_2;
//    }
//
//    if (slices().read_slice(IN_ACT_SLICE, i, i)) {
//      SSDfgVecInput *vec_input = new SSDfgVecInput("I", _ssDFG->num_vec_input(),
//                                                     _ssDFG);
//      //vec_input->setLocMap(pm);
//      //_ssDFG->insert_vec_in(vec_input); -- don't need this, stored in group
//
//      //cout << "vp" << i << "  ";
//
//      vector<bool> mask;
//      mask.resize(port_m.size());
//      for (unsigned mi = 0; mi < port_m.size(); ++mi) {
//        mask[mi] = slices().read_slice(slice,
//                                       start_bits_vp_mask + mi, start_bits_vp_mask + mi);
//        if (mask[mi]) {
//          int ss_in_port = port_m[mi].first;
//          ssinput *in = _ssModel->subModel()->get_input(ss_in_port);
//          SSDfgInput *dfg_in = new SSDfgInput(_ssDFG);
//          dfg_in->setVPort(vec_input);
//          dfgnode_for[in] = dfg_in;
//          _ssDFG->addInput(dfg_in); //add input to dfg
//          vec_input->addInput(dfg_in); //add input to vector
//
//          //cout << mi << " (" << in->port() << ")";
//        }
//      }
//      //cout << "\n";
//      assign_vport(vec_input, make_pair(true/*input*/, i), mask);
//    }
//
//    start_bits_vp_mask = total_bits_vp_mask; //next
//  }
//
//  start_bits_vp_mask = 0;
//  total_bits_vp_mask = 0;
//
//  for (auto &port_pair : _ssModel->subModel()->io_interf().out_vports) {
//    int i = port_pair.first; //index of port
//    std::vector<std::pair<int, std::vector<int> > > &port_m = port_pair.second;
//
//    total_bits_vp_mask += port_m.size();
//
//    if (slices().read_slice(OUT_ACT_SLICE, i, i)) { //activate output port
//      SSDfgVecOutput *vec_output = new SSDfgVecOutput("O", _ssDFG->num_vec_output(),
//                                                        _ssDFG);
//
//      //_ssDFG->insert_vec_out(vec_output); don't do this, this is in group
//
//      vector<bool> mask;
//      mask.resize(port_m.size());
//      for (unsigned mi = 0; mi < port_m.size(); ++mi) {
//        mask[mi] = slices().read_slice(VP_MAP_SLICE_OUT,
//                                       start_bits_vp_mask + mi, start_bits_vp_mask + mi);
//        if (mask[mi]) {
//          ssoutput *out = _ssModel->subModel()->get_output(port_m[mi].first);
//          SSDfgOutput *dfg_out = new SSDfgOutput(_ssDFG);
//          dfg_out->setVPort(vec_output);
//          dfgnode_for[out] = dfg_out;
//          _ssDFG->addOutput(dfg_out);
//          vec_output->addOutput(dfg_out); //add output to vector
//        }
//      }
//      assign_vport(vec_output, make_pair(false/*input*/, i), mask);
//    }
//
//    start_bits_vp_mask = total_bits_vp_mask; //next
//  }
//
//
//  //TODO: NOT NEEDED
//  //Read input nodes
//  for (int i = 0; i < 32; ++i) {  //64 ports ?
//    uint64_t inact = _bitslices.read_slice(IN_ACT_SLICE, i, i);
//
//    if (inact) {
//      auto &vp = _ssModel->subModel()->io_interf().in_vports[i];
//      for (auto &p : vp) {     //iterate through ports of vector port i
//        inputs_used.insert(p.first);        //cgra port
//      }
//    }
//  }
//
//  //Read output nodes
//  for (int i = 0; i < 32; ++i) {
//    uint64_t outact = _bitslices.read_slice(OUT_ACT_SLICE, i, i);
//
//    //If the outport !=0
//    if (outact) {
//      auto &vp = _ssModel->subModel()->io_interf().out_vports[i];
//      for (auto &p : vp) {     //iterate through ports of vector port i
//        outputs_used.insert(p.first);
//      }
//    }
//  }
//
//  int cur_slice = SWITCH_SLICE;       //5th slice in bitslice
//
//  //In1, In2, In3, Opcode, S1, S2, ... S8, Row
//  vector<vector<ssswitch> > &switches = _ssModel->subModel()->switches();
//
//  for (int i = 0; i < _ssModel->subModel()->sizex() + 1; ++i) {
//    bool left = (i == 0);
//    bool right = (i == _ssModel->subModel()->sizex());
//    for (int j = 0; j < _ssModel->subModel()->sizey() + 1; ++j, ++cur_slice) {
//      bool top = (j == 0);
//      bool bottom = (j == _ssModel->subModel()->sizey());
//      ssswitch *sssw = &switches[i][j];
//
//      //cout << "Decode switch: " << i << "," << j << "\n";
//
//      //read the [Row]
//      int row = _bitslices.read_slice(cur_slice, ROW_LOC, ROW_LOC + ROW_BITS - 1);
//      assert(row == j);
//
//      int cur_bit_pos = SWITCH_LOC;
//
//      //---------------------------------DECODE SWITCHES ---------------------------
//      for (int o = 0; o < NUM_OUT_DIRS; ++o, cur_bit_pos += BITS_PER_DIR) {
//
//        uint64_t b = _bitslices.read_slice(cur_slice, cur_bit_pos,
//                                           cur_bit_pos + BITS_PER_DIR - 1);
//        SwitchDir::DIR out_dir = ssdir.dir_for_slot(o, top, bottom, left, right);
//        SwitchDir::DIR in_dir = ssdir.decode(b, top, bottom, left, right);
//        in_dir = SwitchDir::reverse(in_dir);
//
//        sslink *inlink = sssw->getInLink(in_dir);      //get the link object with that dir
//        if (!inlink) {
//          //cout << "no in_dir:" << SwitchDir::dirName(in_dir) << " bits:" << b << " (pos:" << o << ")\n";
//          continue; //no worries, this wasn't even a valid inlink
//        }
//
//        sslink *outlink = sssw->getOutLink(out_dir);
//        if (!outlink) {
//          //cout << "no out_dir:" << SwitchDir::dirNameDBG(out_dir) << " loc:" << o << "\n";
//          continue; //skip if no corresponding link
//        }
//
//        //cout << SwitchDir::dirName(in_dir) << "->" <<
//        //        SwitchDir::dirName(out_dir) << ":";
//
//        //cout << b << " @pos: " << o << " " << out_dir << "\n";
//
//        inst_histo[SS_CONFIG::SS_Switch] += 1;
//
//        assert(outlink->orig() == inlink->dest());
//        assign_switch(sssw, inlink, outlink);
//        //For better or worse, reconstruct takes in a route map, yes, this is redundant
//        //with assign_switch
//        routeMap[sssw][out_dir] = in_dir;
//
//        //if(ssfu* fu =dynamic_cast<ssfu*>(outlink->dest())) {
//        //  //create corresponding DFG Node
//        //  dfg_inst = new SSDfgInst();
//        //  dfgnode_for[fu]=dfg_inst;
//        //  _ssDFG->addInst(dfg_inst);
//        //} else
//
//        ////if the swithces out is an output node
//        //if(ssoutput* out = dynamic_cast<ssoutput*>(outlink->dest())) {
//        //  SSDfgOutput* dfg_out = new SSDfgOutput();
//        //  dfgnode_for[out]=dfg_out;
//        //  _ssDFG->addOutput(dfg_out);
//        //  dfg_out->setVPort(out->port());
//        //}
//
//        ////if the incoming node was from ssinput node
//        //if (ssinput* in=dynamic_cast<ssinput*>(inlink->orig())) {
//        //  SSDfgInput* dfg_in;
//        //  //Need to check if this is actually one of the useful inputs
//        //  if(inputs_used.count(in->port())) {
//        //    if(dfgnode_for.count(in)==0) {
//        //      cout << "Creating node for port " << in->port() << "\n";
//        //      dfg_in = new SSDfgInput();
//        //      dfgnode_for[in]=dfg_in;
//        //      _ssDFG->addInput(dfg_in);
//        //    } else {
//        //      dfg_in = dynamic_cast<SSDfgInput*>(dfgnode_for[in]);
//        //    }
//        //    dfg_in->setVPort(in->port());
//        //  }
//        //}//end if
//
//      }//end for
//
//    }//end for j
//  }//end for i
//
//
//  for (int g = 0; g < NUM_DFG_GROUPS; ++g) {
//    //Read input nodes
//    for (int i = 0; i < 32; ++i) {  //32 ports max
//      int i_adj = (g % 2) * 32 + i;
//      uint64_t inact = _bitslices.read_slice(IN_ACT_GROUP12 + g / 2, i_adj, i_adj);
//      if (inact) {
//        SSDfgVecInput *in_vec =
//                dynamic_cast<SSDfgVecInput *>(vportOf(make_pair(true, i)));
//        assert(in_vec);
//        _ssDFG->insert_vec_in_group(in_vec, g);
//      }
//    }
//    for (int i = 0; i < 32; ++i) {  //32 ports ?
//      int i_adj = (g % 2) * 32 + i;
//      uint64_t outact = _bitslices.read_slice(OUT_ACT_GROUP12 + g / 2, i_adj, i_adj);
//
//      if (outact) {
//        SSDfgVecOutput *out_vec =
//                dynamic_cast<SSDfgVecOutput *>(vportOf(make_pair(false, i)));
//        assert(out_vec);
//        _ssDFG->insert_vec_out_group(out_vec, g);
//      }
//    }
//  }
//
//  //---------------------------------DECODE FUNC UNITS ---------------------------
//  cur_slice = SWITCH_SLICE;
//  for (int i = 0; i < _ssModel->subModel()->sizex(); ++i) {
//    for (int j = 0; j < _ssModel->subModel()->sizey(); ++j, ++cur_slice) {
//      ssfu *ssfu_node = fus[i][j];
//
//      //opcode
//      uint64_t op = _bitslices.read_slice(cur_slice, OPCODE_LOC, OPCODE_LOC + OPCODE_BITS - 1);
//      if (op != 0) { //if O
//        auto inst = ssfu_node->fu_def()->inst_of_encoding(op);
//        dfg_inst = new SSDfgInst(_ssDFG, inst);
//        stringstream verif_name;
//        verif_name << i << "-" << j;
//        dfg_inst->set_verif_id(verif_name.str());
//
//        dfgnode_for[ssfu_node] = dfg_inst;
//        _ssDFG->addInst(dfg_inst);
//
//        inst_histo[inst] += 1;
//
//        //8-switch_dirs + 4bits_for_row
//        unsigned cur_bit_pos = FU_DIR_LOC;
//
//        for (int f = 0; f < NUM_IN_FU_DIRS; ++f, cur_bit_pos += BITS_PER_FU_DIR) {
//          uint64_t b = _bitslices.read_slice(cur_slice, cur_bit_pos,
//                                             cur_bit_pos + BITS_PER_FU_DIR - 1);
//          SwitchDir::DIR dir = ssdir.fu_dir_of(b);
//          assert(f != 0 || (f == 0 && dir != SwitchDir::END_DIR));
//          posMap[dfg_inst].push_back(dir);      //incoming FU dir
//          if (dir == SwitchDir::IM) {
//            dfg_inst->setImmSlot(f);
//          }
//        }//end for input fu dirs
//
//        //predictate inverse
//        uint64_t p = _bitslices.read_slice(cur_slice, FU_PRED_INV_LOC,
//                                           FU_PRED_INV_LOC + FU_PRED_INV_BITS - 1);
//        dfg_inst->setPredInv(p);
//      } else {
//        //cout << i << " " << j << " not mapped\n";
//      }
//    }
//    cur_slice += 1; // because we skipped the switch
//  }
//
//  cur_slice = SWITCH_SLICE +
//              (_ssModel->subModel()->sizex() + 1) * (_ssModel->subModel()->sizey() + 1);
//
//  //--------------------------------------- DECODE CONSTANTS ------------------------
//  while ((unsigned) cur_slice < _bitslices.size()) {
//    int row = _bitslices.read_slice(cur_slice, ROW_LOC, ROW_LOC + ROW_BITS - 1);
//    int col = _bitslices.read_slice(cur_slice, COL_LOC, COL_LOC + COL_BITS - 1);
//
//    assert(row < _ssModel->subModel()->sizey());
//    assert(col < _ssModel->subModel()->sizex());
//    ssfu *ssfu_node = fus[col][row];
//    assert(ssfu_node);

//    //cout << "row,col" << row << " " << col << "\n";
//    SSDfgNode *node = dfgnode_for[ssfu_node];
//    assert(node);
//    SSDfgInst *inst = dynamic_cast<SSDfgInst *>(node);
//
//
//    uint64_t ctrl_bits = _bitslices.read_slice(cur_slice, CTRL_LOC,
//                                               CTRL_LOC + CTRL_BITS - 1);
//
//    inst->set_ctrl_bits(CtrlBits(ctrl_bits));
//
//    bool has_imm = _bitslices.read_slice(cur_slice, IS_IMM_LOC, IS_IMM_LOC + IS_IMM_BITS - 1);
//
//
//    if (has_imm) {
//      uint64_t imm = _bitslices.read_slice(cur_slice, 0, 63);
//      assert(inst->immSlot() != -1);
//      inst->setImm(imm);
//    }
//
//    ++cur_slice;
//  }
//
//  //routemap -- for each ssnode - inlink and outlinks
//  //dfgnode_for -- ssnode to dfgnode mapping
//  //posMap -- for each dfgnode, vector of incoming dirs
//
//  reconstructSchedule(routeMap, dfgnode_for, posMap);
//
//  // Iterate over FUs, get the inc_edge assoc. with each FU_INPUT, set extra lat.
//  cur_slice = SWITCH_SLICE;
//  for (int i = 0; i < _ssModel->subModel()->sizex(); ++i) {
//    for (int j = 0; j < _ssModel->subModel()->sizey(); ++j, ++cur_slice) {
//      ssfu *ssfu_node = fus[i][j];
//      if (nodeAssigned(ssfu_node) != 0) {
//        SSDfgInst *dfg_node = dynamic_cast<SSDfgInst *>(dfgNodeOf(ssfu_node));
//
//        for (int n = 0; n < NUM_IN_FU_DIRS; ++n) {
//          if (dfg_node->immSlot() == n) {
//            //Do Nothing
//          } else if (n < (dfg_node->ops_end() - dfg_node->ops_begin())) {
//            SSDfgEdge *inc_edge = (dfg_node->ops_begin() + n)->get_first_edge();
//            if (!inc_edge) {
//              continue;
//            }
//
//            // delay for each input
//            int d1 = IN_DELAY_LOC + BITS_PER_DELAY * n;
//            int d2 = d1 + BITS_PER_DELAY - 1;
//            set_edge_delay(_bitslices.read_slice(cur_slice, d1, d2), inc_edge);
//
//            //_bitslices.write(cur_slice, d1, d2, _extraLatOfEdge[inc_edge]);
//            //  cout <<  i << " " << j << " slice: " << cur_slice
//            //     << ",delay:" << _extraLatOfEdge[inc_edge] << "\n";
//
//          } else {
//            assert(n != 0 && "can't be no slot for first input");
//          }
//        }
//      }
//    }
//    cur_slice += 1; // because we skipped the switch
//  }
//
//  calcLatency(_max_lat, _max_lat_mis, false);
///*  if(0 != _max_lat_mis) {
//    cerr << "The FU input latencies don't match, this may or may not be a problem!\n";
//  }*/
//
//  calc_out_lat();

//  return inst_histo;
//}

void Schedule::prepareForSaving() {
  for(auto& ep : _edgeProp) {
    ep.links_ser.clear();
    ep.passthroughs_ser.clear();
    for(auto& i : ep.links) {
      ep.links_ser.push_back(std::make_pair(i.first,i.second->id()));
    }
    for(auto& i : ep.passthroughs) {
      ep.passthroughs_ser.push_back(std::make_pair(i.first,i.second->id()));
    }
  }
}

//Write to a header file
void Schedule::printConfigHeader(ostream& os, std::string cfg_name,
                                 bool use_cheat) {

  //Step 1: Write the vector port mapping
  os << "#ifndef " << "__" << cfg_name << "_H__\n";
  os << "#define " << "__" << cfg_name << "_H__\n";

  for(auto& pv : _ssDFG->nodes<SSDfgVecInput*>()) {
    int pn = vecPortOf(pv);
    os << "#define P_" << cfg_name << "_" << pv->name() << " " << pn << "\n"; 
  }
  os<< "\n";
  for(auto& pv : _ssDFG->nodes<SSDfgVecOutput*>()) {
    int pn = vecPortOf(pv);
    os << "#define P_" << cfg_name << "_" << pv->name() << " " << pn << "\n"; 
  }
  os<< "\n";

  if(use_cheat) {
    printConfigCheat(os,cfg_name);
  } else {
    printConfigBits(os,cfg_name);
  }

  os << "#endif //" << cfg_name << "_H\n"; 
}

void Schedule::printConfigCheat(ostream& os, std::string cfg_name) {
  //First, print the config to the file
  std::string file_name = cfg_name + string(".sched");
  std::string full_file_name = string("sched/") + file_name;
  std::ofstream sched_file(full_file_name);
  boost::archive::text_oarchive oa(sched_file);

  oa << BOOST_SERIALIZATION_NVP(*this);

  os << "// CAUTION: This is a Boost::Serialization-based version\n"
     << "// of the schedule.  (ie. cheating)  It is for simulation only.\n"
     << "// corresponding dfg is in: " << full_file_name << "\n\n";

  //Approximate number of config words, good enough for now
  int config_words  = (_ssModel->subModel()->sizex()+1) *
                      (_ssModel->subModel()->sizey()+1)  + 16;

  //Negative size indicates funny thing
  os << "#define " << cfg_name << "_size " << config_words << "\n\n";

  //NOTE: Filename is necessary here! it is the indicator that we
  //are cheating and not giving the real config bits
  os << "char " << cfg_name << "_config[" << config_words << "] = \"";
  os << "filename:" << file_name.c_str() << "\";\n\n";
}

void Schedule::printConfigBits_Hw(std::string & hw_config_filename){
  /*
  pt::ptree root;
  pt::read_xml(hw_config_filename, root);

  // ------ Encode for Switches ------
  vector<ssswitch *> switches = _ssModel -> subModel() -> switch_list();
  for(ssswitch * sssw : switches){
    // Find Hardware Config
    string sw_name = sssw -> get_name();
    bool found = false;
    for (auto it : root.get_child("CGRA.Routers")){
      std::string router_name = it.second.get<std::string>("Module_Name");
      if (sw_name == router_name){// Switch Config Find
        found = true;
        int module_id = it.second.get<int>("Module_ID");
        //bool test = module_id > 0;
        //TODO: test is not used, fix        
      }else{
        continue; // Not this one, continue
      }
    }// End of Switch in Config File
    assert(found&&"Not found the config of this switch");
  }// End of Switch In Model
  */
}

void Schedule::printConfigBits(ostream& os, std::string cfg_name) {
  //print_bit_loc();

  //Active Input Ports
  for(auto& pv : _ssDFG->nodes<SSDfgVecOutput*>()) {
    int pn = vecPortOf(pv);    
    _bitslices.write(OUT_ACT_SLICE,pn,pn,1);
  }
  //Active Input Ports
  for(auto& pv : _ssDFG->nodes<SSDfgVecInput*>()) {
    int pn = vecPortOf(pv);    
    _bitslices.write(IN_ACT_SLICE,pn,pn,1);
  }

  int max_lat_mis=0, max_lat=0;
  cheapCalcLatency(max_lat,max_lat_mis);
  _bitslices.write(IN_ACT_SLICE, 56, 63, max_lat_mis);

  int num_vec_groups = _ssDFG->num_groups();
  assert(num_vec_groups <= NUM_DFG_GROUPS);
  for(int g = 0; g < num_vec_groups; ++g) {
    vector<SSDfgVecInput*>& vec = _ssDFG->vec_in_group(g);
    for(auto* vec_in : vec) {
      //TODO/FIXME: Encoding is broken here
      //int port_num = vecPortOf(vec_in).second;
      //int bit_pos=(g%2)*32+port_num;
      //_bitslices.write(IN_ACT_GROUP12+g/2, bit_pos, bit_pos, 1);
    }
  }

  for(int g = 0; g < num_vec_groups; ++g) {
    vector<SSDfgVecOutput*>& vec = _ssDFG->vec_out_group(g);
    for(auto* vec_out : vec) {
      //TODO/FIXME: Encoding is broken here
      //int port_num = vecPortOf(vec_out).second;
      //int bit_pos=(g%2)*32+port_num;
      //_bitslices.write(OUT_ACT_GROUP12+g/2, bit_pos, bit_pos, 1);
    }
  }



  // --------------------------- ENCODE VP MASK ------------------------------
  for (int io = 0; io < 2; ++io) {
    int start_bits_vp_mask=0;
    int total_bits_vp_mask=0;
    int slice=VP_MAP_SLICE_1;
    for(auto& port_pair : _ssModel->subModel()->io_interf().vports_map[io]) {
      const std::vector<int>& port_m = port_pair.second->port_vec();
      total_bits_vp_mask+=port_m.size();
   
      if(start_bits_vp_mask < 64 && total_bits_vp_mask >64) {
        start_bits_vp_mask=port_m.size();
        total_bits_vp_mask=port_m.size();
        slice=VP_MAP_SLICE_2;
      }
  
      //Is this port assigned?  if not can skip 
      //if(SSDfgVec* dfg_vec_in = vportOf(make_pair(true/*input*/,port_pair.first))) {
        //vector<bool> mask = maskOf(dfg_vec_in);
  
        //for(unsigned i = 0; i < port_m.size(); ++i) {
        //  _bitslices.write(io ? slice : VP_MAP_SLICE_OUT, start_bits_vp_mask+i,start_bits_vp_mask+i,mask[i]);
        //}
      //}
    
      start_bits_vp_mask=total_bits_vp_mask; //next
    }
  }

  xfer_link_to_switch(); // makes sure we have switch representation of routing
  int cur_slice=SWITCH_SLICE;

  vector< vector<ssfu*> >& fus = _ssModel->subModel()->fus();

  //In1, In2, In3, Opcode, S1, S2, ... S8, Row
  vector< vector<ssswitch*> >& switches = _ssModel->subModel()->switches();
  for(int i = 0; i < _ssModel->subModel()->sizex()+1; ++i) {

    bool left = (i==0);                 //left edge switch
    bool right = (i==_ssModel->subModel()->sizex());    //right edge switch
    
    for(int j = 0; j < _ssModel->subModel()->sizey()+1; ++j,++cur_slice) {
      
      //cout << "Encode switch: " << i << "," << j << "\n";

      bool top = (j==0);
      bool bottom = (j==_ssModel->subModel()->sizey());

      //Write the [Row] -- corresponding row of the siwtch based on the j 
      _bitslices.write(cur_slice, ROW_LOC, ROW_LOC + ROW_BITS - 1, j);

      //---------------------------------ENCODE SWITCHES -------------------------------
      ssswitch* sssw = switches[i][j];

      //after switch respresntation
      std::map<SS_CONFIG::sslink*,SS_CONFIG::sslink*>& link_map = _assignSwitch[sssw];
      if(link_map.size()!=0) {

        //Step 1: Encode all output switches with unused input

        //get used inputs
        std::set<int> used_in_enc;
        for(auto I=link_map.begin(), E=link_map.end();I!=E;++I) {
          sslink* inlink=I->second;
          auto in_port  = SwitchDir::reverse(inlink->dir());
          int in_encode = ssdir.encode(in_port,top,bottom,left,right);
          used_in_enc.insert(in_encode);
        }

        int unused_dir_enc=NUM_IN_DIRS; //num input dirs
        for(int i = 0; i < NUM_IN_DIRS; ++i) {
          if(used_in_enc.count(i)==0) {
            unused_dir_enc=i;           //unused input dir
            break;
          }
        }
            
        assert(unused_dir_enc!=NUM_IN_DIRS&&"no unused direction,does this ever happen?");
        //cout << "unused:" << unused_dir_enc << "\n";

        for(int i = 0; i < NUM_OUT_DIRS; ++i) {
          unsigned p1 = SWITCH_LOC + i*BITS_PER_DIR;
          unsigned p2 = p1 + BITS_PER_DIR-1; 
          _bitslices.write(cur_slice,p1,p2,unused_dir_enc);     //Why write unused input dir
        }

        //Step 2: Fill in correct switches
        for(auto I=link_map.begin(), E=link_map.end();I!=E;++I) {
          sslink* outlink=I->first;
          sslink* inlink=I->second;
         
          auto in_port  = SwitchDir::reverse(inlink->dir());
          int in_encode = ssdir.encode(in_port,top,bottom,left,right);
          int out_pos   = ssdir.slot_for_dir(outlink->dir(),top,bottom,left,right);
         
          //cout << SwitchDir::dirName(inlink->dir()) << "->" <<
          //        SwitchDir::dirName(outlink->dir()) << ":";

          //cout << in_encode << " @pos: " << out_pos << "\n";

          unsigned p1 = SWITCH_LOC + out_pos*BITS_PER_DIR;
          unsigned p2 = p1 + BITS_PER_DIR-1; 

          _bitslices.write(cur_slice,p1,p2,in_encode,false /*don't check*/);
        }

      }//end if


      //---------------------------------ENCODE FUNC UNITS ---------------------------
      //
      if(i < _ssModel->subModel()->sizex() && j < _ssModel->subModel()->sizey()) {
        ssfu* ssfu_node = fus[i][j];

        if(isPassthrough(0,ssfu_node)) { //TODO:wrong
          int cur_bit_pos=FU_DIR_LOC;
          int i = 0; //only one dir allowed
          unsigned p1 = cur_bit_pos+BITS_PER_FU_DIR*i;
          unsigned p2 = p1 + BITS_PER_FU_DIR-1;


          for(auto &inlink: ssfu_node->in_links()) {
            for (int slot = 0; slot < 8; ++slot) {
              if (linkAssigned(slot, inlink)) {
                int in_encode = ssdir.encode_fu_dir(inlink->dir()); //get encoding of dir
                _bitslices.write(cur_slice, p1, p2, in_encode);   //input dir for each FU in
                break;
              }
            }
          }

          //opcode encdoing
          unsigned op_encode = ssfu_node->fu_def()->encoding_of(SS_CONFIG::SS_Copy);
          _bitslices.write(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1,op_encode);
        }
       
        //get the dfg node assigned to that FU  
        if(dfg_nodes_of(0,ssfu_node).size()!=0) {
          //FIXME: obviously this should be fixed for slots...
          SSDfgInst* dfg_node = 
            dynamic_cast<SSDfgInst*>(dfgNodeOf(0,ssfu_node));
          
          int cur_bit_pos=FU_DIR_LOC;

          for(int n = 0; n < NUM_IN_FU_DIRS; ++n) {
            unsigned p1 = cur_bit_pos+BITS_PER_FU_DIR*n;
            unsigned p2 = p1 + BITS_PER_FU_DIR-1;

            if(dfg_node->immSlot()==n) {
              _bitslices.write(cur_slice,p1,p2,ssdir.encode_fu_dir(SwitchDir::IM));  //imm slot for FU
            } else if(n  < (dfg_node->ops().end()-dfg_node->ops().begin())) {
              SSDfgEdge* inc_edge = dfg_node->ops()[n].get_first_edge();
              if(!inc_edge) {continue;}
              SSDfgNode* inc_dfg_node = inc_edge->def();
              
              bool assigned=false;
              for(auto inlink: ssfu_node->in_links()) {
                for (int slot = 0; slot < 8; ++slot) {
                  if (linkAssigned(slot, inlink) && dfgNodeOf(slot, inlink) == inc_dfg_node) {
                    assert(inlink->dir() != SwitchDir::END_DIR);
                    int in_encode = ssdir.encode_fu_dir(inlink->dir()); //get the encoding of the dir
                    _bitslices.write(cur_slice, p1, p2, in_encode);      //input direction for each FU in
                    assigned = true;
                    break;
                  }
                }
              }
              if(!assigned) {
                cout << "Could not find mapped input link for mapped edge: " 
                     << inc_edge->name() << " at loc: " << ssfu_node->name() 
                     << "\n";
                printGraphviz("viz/sched-fail-connection.gv");
                assert(assigned);
              }
              //assert(assigned);

              //delay for each input
              int d1=IN_DELAY_LOC+BITS_PER_DELAY*n;
              int d2=d1+BITS_PER_DELAY-1;

              int ed = edge_delay(inc_edge);
              int max_representable = ((1 << BITS_PER_DELAY)-1);
              if(ed > max_representable) {
                cout << "FU " << i << "," << j << " delay too large (" 
                  << ed << ", but if that doesn't matter just ignore.\n";
                ed = max_representable;
              }

              _bitslices.write(cur_slice, d1, d2, ed);
              //cout <<  i << " " << j << " slice: " << cur_slice 
              //   << ",delay:" << _extraLatOfEdge[inc_edge] << "\n";

            } else {
              assert(n!=0 && "can't be no slot for first input");
            }
          }
          
          //print predicate
          _bitslices.write(cur_slice,FU_PRED_INV_LOC,
                                     FU_PRED_INV_LOC+FU_PRED_INV_BITS-1,
                                     dfg_node->predInv());
         
          //opcode encdoing
          unsigned op_encode = ssfu_node->fu_def()->encoding_of(dfg_node->inst());
          _bitslices.write(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1,op_encode);
        }

      } //end if for func encode
    
    }//end for switch x
  }//end for switch y 

  //--------------------------------------- ENCODE CONSTANTS ------------------------
  for(int i = 0; i < _ssModel->subModel()->sizex(); ++i) {    
    for(int j = 0; j < _ssModel->subModel()->sizey(); ++j) {
      ssfu* ssfu_node = fus[i][j];
      if(dfg_nodes_of(0,ssfu_node).size()!=0) {
        SSDfgInst* dfg_node = dynamic_cast<SSDfgInst*>(dfgNodeOf(0,ssfu_node));
        bool has_imm_slot = dfg_node->immSlot()!=-1;
        uint64_t ctrl_bits = dfg_node->ctrl_bits();
        if(has_imm_slot || ctrl_bits) {
           //cout << i << " " << j << " " << dfg_node->immSlot() << "\n";
           _bitslices.write(cur_slice,ROW_LOC,ROW_LOC+ROW_BITS-1,j);
           _bitslices.write(cur_slice,COL_LOC,COL_LOC+COL_BITS-1,i);
           _bitslices.write(cur_slice,IS_IMM_LOC,
                                      IS_IMM_LOC+IS_IMM_BITS-1,has_imm_slot);
           _bitslices.write(cur_slice,CTRL_LOC,CTRL_LOC+CTRL_BITS-1,ctrl_bits);
           ++cur_slice;
        }
        if(has_imm_slot) {
           _bitslices.write(cur_slice,0,63,dfg_node->imm());
           ++cur_slice;
        }
      }
    }
  } 
  os << "#define " << cfg_name << "_size " << _bitslices.size() << "\n\n";

  os << "unsigned long long " << cfg_name << "_config[" << _bitslices.size() << "] = {";
  for(unsigned i = 0; i < _bitslices.size(); ++i) {
    if(i!=0) {os <<", ";}
    if(i%8==0) {os <<"\n";}
    os << "0x" << std::setfill('0') << std::setw(2) << std::hex << _bitslices.read_slice(i);
  }
  os << "};\n";
  os << std::dec;
}

void Schedule::printConfigVerif(ostream& os) {
  for(unsigned i = 0; i < _bitslices.size(); ++i) {
    os << std::setfill('0') << std::setw(16) 
       << std::hex << _bitslices.read_slice(i) << "\n";
  }
}

void Schedule::printMvnGraphviz(std::ofstream& ofs, ssnode* node) {
  auto& np = _nodeProp[node->id()];

  for(int i = 0; i < 8; ++i) {
    std::vector<SSDfgNode*> vertices;
    for(auto elem : np.slots[i].vertices) {
      if(elem.second==i) {
        vertices.push_back(elem.first);
      }
    }

    if(vertices.size() == 0) {
        ofs << "<tr><td border=\"1\"> " << node->name() << " </td></tr>";
    } else {
      for(auto v : vertices) {
        ofs << "<tr><td port=\"" << v->name() 
            << "\" border=\"1\" bgcolor=\"#" 
            << std::hex << colorOf(v->values()[0]) << std::dec << "\">" 
  //          << ((node->max_util()!=1) ? "T!" : "")
            << v->name() << "</td></tr>";
      }
    }
  }
}

void Schedule::printMelGraphviz(std::ofstream& ofs, ssnode* node) {
  for(auto link: node->out_links()) {

    int ovr=0,agg_ovr=0,max_util=0;
    get_link_overprov(link,ovr,agg_ovr,max_util);

    std::vector<int> empty_slots;

    //show unique values and slices:  Value, l(), r()
    std::unordered_set<std::tuple<SSDfgValue*,int,int>,
             boost::hash<std::tuple<SSDfgValue*,int,int>> > seen_values;

    auto &lp = _linkProp[link->id()];
    for (int slot= 0; slot < 8; ++slot) {
      if(lp.slots[slot].edges.size()==0) empty_slots.push_back(slot);

      for (auto it : lp.slots[slot].edges) {
        SSDfgEdge* e = it.first;
        auto print_tuple = make_tuple(e->val(),e->l(),e->r());
        if(!seen_values.count(print_tuple)) {
          seen_values.insert(print_tuple);

          ofs << link->orig()->name() << "->" << link->dest()->name()
              << " [color=\"#" << std::hex << colorOf(e->val()) << std::dec << "\" "
              << (link->max_util()>1 ? " style=dotted " : "")
              << " label=\""; 
          if(slot!=0) {
            ofs << "s:" << slot << "-" << slot+e->bitwidth()/8-1;
          }
          if(agg_ovr!=0) {
            ofs<< "OVR:" << agg_ovr << " ";
          }
          ofs<< "\"];\n";
        }
      }
    }
    if(empty_slots.size()) {
      ofs << link->orig()->name() << "->" << link->dest()->name()
          << " [color=gray style=dotted, label=\"";
      if(empty_slots.size() != 8) {
        printCondensedVector(empty_slots,ofs);
      }
      ofs   <<"\" fontcolor=gray]" << "\n";
    }

  } 
 
}

void Schedule::printCondensedVector(std::vector<int>& vec, std::ostream& os) {
  int prev_i = -100;
  bool printed_dash=false;
  for(unsigned ind = 0; ind < vec.size(); ind++) {
    int i = vec[ind];
    if(ind==0) {
      os << i;
    } else if(ind==vec.size()-1) {
      if(printed_dash) os << i;
      else os << "," << i;
    } else if(prev_i+1==i) {
      if(!printed_dash) {
        os << "-";
        printed_dash=true;
      }
    } else {
      if(printed_dash) {
        os << prev_i << "," << i;
      } else {
        os << "," << i;
      }
      printed_dash=false;
    }
    prev_i=i;
  }
}

void Schedule::printNodeGraphviz(std::ofstream& ofs, ssnode* node) {
  int sy = _ssModel->subModel()->sizey();

  if(dynamic_cast<ssvport*>(node)) {
    auto& np = _nodeProp[node->id()];
    if(np.slots[0].vertices.size()==0) return;
  }

  ofs << node->name() << "[shape=plaintext, ";
  ofs << "label = <<table border=\"0\" cellspacing=\"0\">";

  printMvnGraphviz(ofs,node);

  ofs << "\n</table>>, pos = \"" << gvsf*node->x() + gvsf/2.0 << "," 
                                 << sy-gvsf*node->y()-1 - gvsf/2.0<< "!\"";
  ofs << ", pin=true];\n";  
}

void Schedule::printSwitchGraphviz(std::ofstream& ofs, ssswitch* sw) {
  int sy = _ssModel->subModel()->sizey();

  ofs << sw->name() << " [shape=diamond, ";
  ofs << "pos = \"" << gvsf*sw->x()  << "," << sy-gvsf*sw->y()-1 << "!\"";
  ofs << ", pin=true];\n";  
}

void Schedule::printGraphviz(const char* name) {
  ofstream ofs(name);
  assert(ofs.good());
  
  SS_CONFIG::SubModel* sub = _ssModel->subModel();

  ofs << "digraph sched {\n";

  for (auto elem : sub->input_list())
    printNodeGraphviz(ofs, elem);

  for (auto elem : sub->output_list())
    printNodeGraphviz(ofs, elem);

  for (auto &elem: sub->switch_list())
    printSwitchGraphviz(ofs, elem);

  for (auto elem : sub->fu_list())
    printNodeGraphviz(ofs, elem);

  for(ssnode* node : sub->node_list())
    printMelGraphviz(ofs,node);

  ofs << "}\n\n";
}

#if 0
// Lets keep this code around for inspiration, but basically we don't
// need to reconstruct the schedule for simulation because we have the
// option to keep the serialized version directly
//reconstruct the schedule
void Schedule::reconstructSchedule(
                  map<ssnode*, map<SwitchDir::DIR,SwitchDir::DIR> >& routeMap,
                  map<ssnode*, SSDfgNode* >& dfgnode_for, 
                  map<SSDfgNode*, vector<SwitchDir::DIR> >& posMap
                  ) {
  //iterate over inputs (ssinputs)
  for (auto elem : _ssModel->subModel()->inputs()) {
    ssinput* ssinput_node = (ssinput*) &elem;
   
    //get the dfg node for ssinput
    if(dfgnode_for.count(ssinput_node)!=0) {
        //cout << "reconstruction from input" << ssinput_node->name() << " " << ssinput_node->port() << "\n";
        SSDfgNode* dfg_node = dfgnode_for[ssinput_node];
        tracePath(ssinput_node, dfg_node, routeMap, dfgnode_for, posMap);
    }
  }

  //iterate over fus
  /*
  vector< vector<ssfu*> >& fus = _ssModel->subModel()->fus();
  for(int i = 0; i < _ssModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _ssModel->subModel()->sizey(); ++j) {
      ssfu* ssfu_node = fus[i][j];
      if(dfgnode_for.count(ssfu_node)!=0) {
        //cout << "reconstruct from fu " << i << " " << j << "\n";
        SSDfgNode* dfg_node = dfgnode_for[ssfu_node];
        tracePath(ssfu_node, dfg_node, routeMap, dfgnode_for, posMap);
      }
        
    }
  }
  */
  std::vector< ssfu* > fus = _ssModel->subModel()->fu_list();
  for(auto fu : fus) {
    ssfu* ssfu_node = dynamic_cast<ssfu*>(fu);
    if(dfgnode_for.count(ssfu_node)!=0) {
      //cout << "reconstruct from fu " << i << " " << j << "\n";
      SSDfgNode* dfg_node = dfgnode_for[ssfu_node];
      tracePath(ssfu_node, dfg_node, routeMap, dfgnode_for, posMap);
    }
  }
  allocate_space();
}
#endif

void Schedule::stat_printOutputLatency() {
  int n = _ssDFG->num_vec_output();
  cout << "** Output Vector Latencies **\n";
  for (int i = 0; i < n; i++) {
    SSDfgVecOutput *vec_out = _ssDFG->vec_out(i);
    auto loc = location_of(vec_out);
    ssvport* vport = dynamic_cast<ssvport*>(loc.second);
    cout << vec_out->gamsName() << " to " 
         << vport->name() << " sz" << vport->size() << ": ";
    for (auto inc_edge : vec_out->in_edges()) {
      int routing_latency = edge_latency(inc_edge);
      int edge_lat = edge_delay(inc_edge) + routing_latency -1;
      cout << latOf(inc_edge->def())  + edge_lat<< " ";
    }
    cout << endl;
  }
}

bool Schedule::fixLatency(int &max_lat, int &max_lat_mis) {
  for(auto& i : _edgeProp) {
    i.extra_lat=0;
  }

  iterativeFixLatency();

  max_lat=0;
  max_lat_mis=0;
  cheapCalcLatency(max_lat, max_lat_mis, false);

  //int max_lat2=0;
  //int max_lat_mis2=0;
  //calcLatency(max_lat2, max_lat_mis2);

  //if(max_lat != max_lat2|| max_lat_mis!=max_lat_mis2) {
  //  cout << "max_lat: " << max_lat << " mis:" << max_lat_mis << "\n";
  //  cout << "max_lat2: " << max_lat2 << " mis2:" << max_lat_mis2 << "\n";
  //}

  //_ssDFG->printGraphviz("viz/remap-fail.dot");
  //printGraphviz("viz/sched.gv");
  //cout << "-------------------------------------------------------------------\n";
  //cout << "-------------------------------------------------------------------\n"; 
  //exit(1);

  //printGraphviz("viz/sched.gv");

  return max_lat_mis==0;
}

std::vector<SSDfgNode*> Schedule::ordered_non_temporal() {
  std::vector<SSDfgNode*> res;
  for (SSDfgNode *i : _ssDFG->ordered_nodes()) {
    if (!i->is_temporal() && !dynamic_cast<SSDfgVecInput*>(i)) {
      res.push_back(i);
    }
  }
  return res;
}


void Schedule::iterativeFixLatency() {
  bool changed = true;
  reset_lat_bounds();

  int max_ed = _ssModel->maxEdgeDelay();
  int iters = 0;

  bool overflow = false;
  int max_mis = 0;

  int _max_expected_route_latency=8;

  std::vector<SSDfgNode *> ordered_non_temp = ordered_non_temporal();

  while (changed || overflow) {
    changed = false;

    iters++;
    if (overflow) {
      overflow = false;
      reset_lat_bounds();
      max_mis++;
    }

    //FORWARD PASS 
    for (SSDfgNode *node : ordered_non_temp) {
      auto &vp = _vertexProp[node->id()];
      int new_min = vp.min_lat;
      int new_max = vp.max_lat;

      for (auto edge : node->in_edges()) {
        SSDfgNode *origNode = edge->def();
        auto &orig_vp = _vertexProp[origNode->id()];
   
        int routing_latency =edge_latency(edge);
        int edge_lat = origNode->lat_of_inst() + routing_latency -1;

        //cout << " -----------------" <<  edge->name() << ": " << edge_lat << "\n";

        //This edge is routed
        if(routing_latency!=0) {
          new_min = std::max(new_min, orig_vp.min_lat + edge_lat);
          new_max = std::min(new_max, orig_vp.max_lat+edge_lat+max_ed+max_mis);
        } else {
          //This edge is not routed, so give worst case upper bound
          new_min = std::max(new_min, orig_vp.min_lat + edge_lat +
              _min_expected_route_latency);
          new_max = std::min(new_max, orig_vp.max_lat+edge_lat+max_ed+max_mis
              +_max_expected_route_latency);
        }
      }
      changed |= new_min != vp.min_lat;
      changed |= new_max != vp.max_lat;
      vp.min_lat = new_min;
      vp.max_lat = new_max;

      //cout << node->name() << "  min_lat:" << vp.min_lat 
      //                     << " max_lat:"<< vp.max_lat  
      //                     << " max_mis:" << max_mis << "\n";

      if (new_min > new_max) {
        overflow = true;
        break;
      }

    }

    if (overflow) continue;

    //BACKWARDS PASS
    for (int i = ordered_non_temp.size() - 1; i >= 0; i--) {
      SSDfgNode *node = ordered_non_temp[i];
      auto &vp = _vertexProp[node->id()];
      int new_min = vp.min_lat;
      int new_max = vp.max_lat;

      for (auto edge : node->uses()) {
        if (edge == nullptr) continue;
        SSDfgNode *useNode = edge->use();
        auto &use_vp = _vertexProp[useNode->id()];

        int routing_latency=edge_latency(edge);
        int edge_lat = routing_latency-1 + node->lat_of_inst();

        int my_max_ed = max_ed;
        if(dynamic_cast<SSDfgVecOutput*>(useNode)) {
          my_max_ed=0;
        }

        if(routing_latency!=0) {
          new_min = std::max(new_min, use_vp.min_lat - edge_lat -my_max_ed-max_mis);
          new_max = std::min(new_max, use_vp.max_lat - edge_lat);
        } else {
          new_min = std::max(new_min, use_vp.min_lat - edge_lat - my_max_ed - max_mis
              - _max_expected_route_latency);
          new_max = std::min(new_max, use_vp.max_lat - edge_lat
              - _min_expected_route_latency);
        }
      }
      changed |= new_min != vp.min_lat;
      changed |= new_max != vp.max_lat;
      vp.min_lat = new_min;
      vp.max_lat = new_max;

      //cout << node->name() << "  min_lat-b:" << vp.min_lat 
      //                     << " max_lat-b:"<< vp.max_lat << "\n";

      if (new_min > new_max) {
        overflow = true;
        break;
      }
    }
  }

  //cout << "iters until converge: " << iters << ", mismatch: " << max_mis << "\n";
  // NOW SET THE LATENCY!

  //TODO: need to check how this allows delays or not on vector outputs
  for (SSDfgNode *node : ordered_non_temp) {
    auto &vp = _vertexProp[node->id()];
    //cout << inst->name() << "  min_lat:" << vp.min_lat << " max_lat:" 
    //                                    << vp.max_lat << "\n";
    int target = vp.min_lat;;// < vp.max_lat ? vp.min_lat : 
    //              (vp.min_lat + vp.max_lat) / 2;
    //cout << "target : " << target << "\n";

    int max = 0;
    //int mis = 0;
    for (auto edge : node->in_edges()) {
      if (edge == nullptr) continue;
      SSDfgNode *origNode = edge->def();

      int routing_latency = edge_latency(edge);
      int max_edge_delay = _ssModel->maxEdgeDelay();

      if(routing_latency==0) { //if its not scheduled yet, be more liberal
        routing_latency=_min_expected_route_latency;
        max_edge_delay+=_max_expected_route_latency; 
      }

      int lat = latOf(origNode) + routing_latency -1; 

      int diff = std::max(std::min(max_edge_delay, target - lat),0);
      //mis = std::max(mis,(target- lat) - diff);
      set_edge_delay(diff,edge);

      int vio = std::max(0,(target-lat) - max_edge_delay);
      if(vio > 0) {
        record_violation(edge,vio);
      } 

      max=std::max(max,lat+diff);     
      //cout << " -- " << origNode->name() << "diff"  << diff 
      //                         << "links:" << link_count(edge)-1 << "\n";
    }
    //cout << " * mis: " << mis << "\n";
    assign_lat(node, node->lat_of_inst() + max);
  }

}

void Schedule::cheapCalcLatency(int &max_lat, int &max_lat_mis, bool set_delay) {
  _totalViolation=0;
  max_lat_mis=0;
  max_lat=0;
  _groupMismatch.clear();

  std::vector<SSDfgNode*> ordered_non_temp = ordered_non_temporal();

  for(SSDfgNode* node : ordered_non_temp) {
    calcNodeLatency(node,max_lat,max_lat_mis,set_delay);
  }
}

void Schedule::calcNodeLatency(SSDfgNode* node, int &max_lat, int &max_lat_mis, 
    bool set_delay) {
  int low_lat=MAX_SCHED_LAT, up_lat=0;

  for(auto edge : node->in_edges()) {

    SSDfgNode* origNode = edge->def();

    //If routing latency is 0, then its okay to assume minimum 
    int routing_latency = edge_latency(edge);
    if(routing_latency==0) {
      routing_latency=_min_expected_route_latency;
    }

    if (origNode != nullptr) {
      int edge_lat = edge_delay(edge) + routing_latency -1;
      assert(edge_lat >= 0);
      assert(edge_delay(edge)==0 || set_delay == 0);
      int lat = latOf(origNode) + edge_lat; 

      if(lat>up_lat) up_lat=lat;
      if(lat<low_lat) low_lat=lat;
    }
  }

  assign_lat_bounds(node,low_lat,up_lat); //FIXME: turn off, just for debug

  int diff = up_lat - low_lat; // - _ssModel->maxEdgeDelay();

  if(!node->is_temporal()) {
    if(diff>max_lat_mis) {
      max_lat_mis=diff;
    }
    if(diff>_groupMismatch[node->group_id()]) {
      _groupMismatch[node->group_id()] = diff;
    }
  }

  int new_lat = node->lat_of_inst() + up_lat;
  assign_lat(node,new_lat);

  //ssnode* n = locationOf(inst);
  //cout << "C " << inst->name() << " node: " << n->name() 
  //  << " low_lat: " << low_lat << " up_lat:" << up_lat << " latmis:" 
  //  << max_lat_mis << "diff: " << diff << "\n";
  
  if(max_lat < new_lat) max_lat=new_lat;

  if(diff > 0) {
    add_violation(diff);
    record_violation(node,diff);
  } else {
    record_violation(node,0);
  }

  assert(!set_delay); //set delay depricated

}

// Calculate the exact latency by traversing the schedule
// -- Note that this function is performance non-critical, as its
// purpose is to verify the schedule's latency calcuated cheaply
// TODO: This function should be udpated for link slots
void Schedule::calcLatency(int &max_lat, int &max_lat_mis, bool warnMismatch) {
  queue<sslink*> openset;

  unordered_map<sslink*,int> lat_edge;
  
  max_lat=0;  
  max_lat_mis=0;

  for(auto elem : _ssModel->subModel()->nodes<ssvport*>()) {
    for(auto link : elem->out_links()) {
      if (SSDfgNode *dfgnode = dfgNodeOf(0,link)) {
        openset.push(link);
        lat_edge[link] = latOf(dfgnode);
      }
    }
  }
    
 //Outlinks of all the inputs 
  while(!openset.empty()) {
    sslink* inc_link = openset.front(); 
    openset.pop();
    //cout << inc_link->name() << "\n";   

    //dest node
    ssnode* node = inc_link->dest();
    
    if(ssnode* next_node = dynamic_cast<ssfu*>(node)) {
      sslink* new_link = next_node->getFirstOutLink();
      if(lat_edge.count(new_link)) {
        continue; //skip if we've done it already
      }

      SSDfgNode* next_dfgnode = dfgNodeOf(0,node);
      if(!next_dfgnode && !isPassthrough(0,node)) {
        assert(false && "problem with latency calculation!\n");
        max_lat=-1;
        max_lat_mis=-1;
        return;
      }

      SSDfgInst* next_dfginst = dynamic_cast<SSDfgInst*>(next_dfgnode); 
      assert(next_dfginst || isPassthrough(0,node));

      bool everyone_is_here = true;

      for(auto &inlink: next_node->in_links()) {
        for (int slot = 0; slot < 8; ++slot) {
          if (dfgNodeOf(slot, inlink) != nullptr) {
            if (!lat_edge.count(inlink)) {
              everyone_is_here = false;
              break;
            }
          }
        }
      }

      int max_latency=0;
      int low_latency=100000000;  //magic number, forgive me

 
      if (everyone_is_here) {
        //cout << "----------------------------------- DONE WITH " 
        //     <<  next_fu->name() << "\n";
        // Latency should be the same across all the incoming edges
        for (auto &inlink: next_node->in_links()) {
          for (int slot = 0; slot < 8; ++slot) {
            SSDfgNode *origNode = dfgNodeOf(slot, inlink);
            if (origNode != nullptr) {
              int curLat = lat_edge[inlink];
              //cout << "reading: " << inlink->name() << "\n";

              if (!isPassthrough(0,node)) {
                SSDfgEdge *edge = origNode->getLinkTowards(next_dfgnode);
                if (!edge) {
                  continue;
                  cout << "Edge: " << origNode->name() << " has no edge towards "
                       << next_dfgnode->name() << ", for link:"
                       << inlink->name() << "\n";
                  if (next_dfginst->isDummy()) cout << "dummy!\n";

                  _ssDFG->printGraphviz("viz/remap-fail2.dot");
                  printGraphviz("viz/remap-fail2.gv");
                  assert(false);
                }
                if (edge_delay(edge)) {
                  curLat += edge_delay(edge);
                }
              }

              if (curLat > max_latency) {
                max_latency = curLat;
              }
              if (curLat < low_latency) {
                low_latency = curLat;
              }

              if (warnMismatch && max_latency != low_latency) {
                cout << "Mismatch, min_lat:" << low_latency << ", max_lat:" << max_latency
                     << ", link:" << inlink->name() << "\n";
                if (!isPassthrough(0,node)) {
                  SSDfgEdge *edge = origNode->getLinkTowards(next_dfgnode);
                  cout << "(calcLat) Edge " << edge->name() << "  lat_edge: " << lat_edge[inlink]
                       << "  extralat:" << edge_delay(edge) << "\n";
                } else {
                  cout << "passthrough\n";
                }

              }
            }
          }
        }
      }

      if (everyone_is_here) {
        // Update latency of outgoing edge
        if (isPassthrough(0,node)) {
          lat_edge[new_link] = max_latency + 1; //TODO: Check this
        } else { //regular inst
          int l = max_latency + inst_lat(next_dfginst->inst());
          lat_edge[new_link] = l;

          //if(next_dfginst) {
          //  cout << "L " << next_dfginst->name() << " lat:" << l
          //       << " low:" << low_latency << " up:" << max_latency << " - "
          //       << " lat:" << latOf(next_dfgnode)
          //       << " low:" << lat_bounds(next_dfgnode).first << " up:"
          //       << lat_bounds(next_dfgnode).second << "\n";
          //}

          assign_lat(next_dfgnode,l);
        }

        //if(next_dfginst) {
        //  cout << "L " << next_dfginst->name() << " " << latOf(next_dfgnode) 
        //       << " up:" << max_latency << " low:" << low_latency << "\n";
        //}

        openset.push(new_link);

        //cout << "lat of " << next_dfgnode->name() 
        //     << ", old:" << _latOf[next_dfgnode]
        //     << ", new:" << max_latency << " " << lat_edge[new_link] << "\n";

        int diff = max_latency-low_latency;
        if(diff>max_lat_mis) {
          max_lat_mis=diff;
        }
      }
    } else {
      for (auto &out_link : node->out_links()) {
        for (int slot = 0; slot < 8; ++slot) {

          //We'll need to check to make sure if there is ambiguity between links
          if (have_switch_links()) {
            ssswitch *sssw = dynamic_cast<ssswitch *>(node);
            assert(sssw);
            sslink *new_inc_link = get_switch_in_link(sssw, out_link);
            if (new_inc_link != inc_link) {
              continue;
            }
            SSDfgNode *n_out = dfgNodeOf(slot, out_link);
            SSDfgNode *n_inc = dfgNodeOf(0, inc_link);

            if (n_out != n_inc) {
              cout << "ERROR: Links don't match!  "
                   << out_link->name() << "is assigned "
                   << (n_out ? n_out->name() : "nothing, and ")
                   << inc_link->name() << "is assigned "
                   << (n_inc ? n_inc->name() : "nothing")
                   << "\n";
            }
            printGraphviz("viz/fail-link-match.gv");
          } else if (dfgNodeOf(slot, out_link) != dfgNodeOf(0, inc_link)) {
            continue;
          }

          lat_edge[out_link] = lat_edge[inc_link] + 1;
          openset.push(out_link);
        }
      }
    }
  }

  for(auto& i : lat_edge) {
    sslink *link = i.first;
    int lat = i.second;
    set_link_order(0, link, lat);
  }

  _max_lat=max_lat;
  _max_lat_mis=max_lat_mis; 

}

#if 0
// I am decomissioning this for now because its unclear if we need it.  I
// will leave it for posterity + inspiration. -- Tony
//
//Trace the path of a schedule to help re-create the DFG
//This is not a high-performance implementation, but shouldn't have to be
//because its not called often
void Schedule::tracePath(ssnode* ssspot, SSDfgNode* dfgnode, 
    map<ssnode*, map<SwitchDir::DIR,SwitchDir::DIR> >& routeMap,
    map<ssnode*, SSDfgNode* >& dfgnode_for, 
    map<SSDfgNode*, vector<SwitchDir::DIR> >& posMap) {

  //FIXME: later put the value makes more sense
  assign_node(dfgnode, make_pair(0, ssspot));
  
  vector<tuple<ssnode*, SwitchDir::DIR, std::vector<sslink*>> > worklist;

  sslink* firstLink = ssspot->getFirstOutLink();

  //FIXME: assign_link(dfgnode,firstLink); //TODO: Check if broke anything
  std::vector<sslink*> lvec;
  lvec.push_back(firstLink);

  ssnode* startItem = firstLink->dest();
  SwitchDir::DIR initialDir = firstLink->dir();
  worklist.push_back(make_tuple(startItem,initialDir,lvec));

  //cerr << "---   tracing " << ssspot->name() << "   ---\n"; 
  
  while(!worklist.empty()) {
    
    //cerr << "worklist: ";
    //for(unsigned i = 0; i < worklist.size(); ++i) {
    //  ssnode* item = worklist[i].first;
    //  SwitchDir::DIR dir = worklist[i].second;
    //  cerr << SwitchDir::dirName(dir) << ", " << item->name() << "";
    //  cerr << " | ";
    //}
    //cerr << "\n";
   
    auto& item = worklist.back();
    ssnode* curItem = std::get<0>(item);
    SwitchDir::DIR inDir = std::get<1>(item);

    auto item_links = std::get<2>(item);
    worklist.pop_back();
    
    map<SwitchDir::DIR,SwitchDir::DIR>::iterator I,E;
    for(I=routeMap[curItem].begin(), E=routeMap[curItem].end(); I!=E; ++I) {
      SwitchDir::DIR newOutDir = I->first;
      SwitchDir::DIR newInDir = I->second;
      
      if(inDir == newInDir) { //match!

        //sslink* inLink = curItem->getInLink(newInDir);
        
        sslink* outLink = curItem->getOutLink(newOutDir);
        //FIXME: assign_link(dfgnode,outLink);

        auto links = item_links;
        links.push_back(outLink);
        
        if(outLink==nullptr) {
          cerr << "outlink is null: ";
          cerr << curItem->name() << " (dir:" << SwitchDir::dirName(newOutDir) << "\n";
          assert(0);
        }

        ssnode* nextItem = outLink->dest();

        //cerr << "match" << curItem->name() << " (dir:" << SwitchDir::dirName(newOutDir)
        //     << " " << nextItem->name() << "\n";


       //Output node 
        if(ssoutput* ssout = dynamic_cast<ssoutput*>(nextItem)) {
          //cerr << SwitchDir::dirName(newInDir) << " -> " << SwitchDir::dirName(newOutDir)
          //     << "    out port" << ssout->port() << "\n";

          SSDfgNode* dest_dfgnode = dfgnode_for[ssout];
          assert(dest_dfgnode);
          
          //_assignNode[ssout]=dest_dfgnode;  //perform the assignment
          //_ssnodeOf[dest_dfgnode]=ssout;
          assign_node(dest_dfgnode, make_pair(0, ssout));

          SSDfgEdge* edge = _ssDFG->connect(dfgnode->values()[0],dest_dfgnode,0, SSDfgEdge::data); 
          for(auto& i : links) {
            //FIXME: for now, we do not use this function to rebuild the schedule, so I just put some random value for the slot
            assign_edgelink(edge, 0, i);
          }

        } else if(ssfu* fu_node = dynamic_cast<ssfu*>(nextItem)) {

          //cerr << SwitchDir::dirName(newInDir) << " -> " << SwitchDir::dirName(newOutDir)
          //     << "    (" << fu_node->x() << " " << fu_node->y() << " .. FU)" << "\n";

          SSDfgInst* dest_dfgnode = dynamic_cast<SSDfgInst*>(dfgnode_for[fu_node]);
          assert(dest_dfgnode);

          int n_ops = SS_CONFIG::num_ops[dest_dfgnode->inst()];

          for(int slot = 0; slot < NUM_IN_FU_DIRS; ++slot) { 
            if(posMap[dest_dfgnode][slot] == outLink->dir()) {
              auto edge_type = SSDfgEdge::data;
              if(slot==n_ops) edge_type = dest_dfgnode->predInv()? 
                             SSDfgEdge::ctrl_false : SSDfgEdge::ctrl_true;

              SSDfgEdge * edge = _ssDFG->connect(dfgnode->values()[0],dest_dfgnode, slot, edge_type);
              //FIXME: put a placeholder for the slot to pass compilation first. Later we will use this function to reconstruct the scheudle
              for(auto& i : links) {
                assign_edgelink(edge, 0, i);
              }
            }
          }
        
        } else if(dynamic_cast<ssswitch*>(nextItem)){ //must be switch
          worklist.emplace_back(make_tuple(nextItem,newOutDir,links));
        } else {
          assert(0);
        }
      }
    }
  }
}
#endif

template<typename T>
int count_unique(std::vector<T> &vec) {
  sort(vec.begin(), vec.end());
  return unique(vec.begin(), vec.end()) - vec.begin();
}

void Schedule::get_overprov(int& ovr, int& agg_ovr, int& max_util) {
  ovr = 0;
  agg_ovr = 0;
  max_util = 0;

  for (auto v: _vertexProp) {
    if (v.node) {
      const auto &np = _nodeProp[v.node->id()];

      //Calculate aggregate overage
      for(int i = 0; i<8; ++i) {
        auto& slot = np.slots[i];
        int cnt=0;

        vector<SSDfgNode *> io;
        for (auto elem: slot.vertices) {
          SSDfgNode* v = elem.first;
          if (v->is_temporal()) {
            if (v->type() == SSDfgNode::V_INPUT)
              io.push_back(v);
            if (v->type() == SSDfgNode::V_OUTPUT)
              io.push_back(v);
          } else {
            cnt++; 
          }
        }
        int unique_io = count_unique(io);

        int cur_util = cnt + slot.num_passthroughs + unique_io;
        int cur_ovr = cur_util - v.node->max_util();
        agg_ovr += std::max(cur_ovr, 0);
        ovr = max(ovr, cur_ovr);
        max_util = std::max(cur_util, max_util);
      }
    }
  }

  for (auto &n : _ssModel->subModel()->node_list()) {
    for (auto &elem: n->out_links()) {
      get_link_overprov(elem,ovr,agg_ovr,max_util);
    }
  }

}

void Schedule::get_link_overprov(sslink* link, 
                                 int& ovr, int& agg_ovr, int& max_util) {

  for (int slot = 0; slot < 8; ++slot) {
    auto &lp = _linkProp[link->id()];
    int util = 0;

    std::vector<SSDfgVec *> vecs;
    std::vector<std::pair<SSDfgValue*,int>> values;

    for (auto it : lp.slots[slot].edges) {
      SSDfgEdge* edge = it.first;
      //std::cout << edge->name() << "\n";
      auto v = edge->def();
      if (v->is_temporal()) {
        if (auto input = dynamic_cast<SSDfgVecInput *>(v)) {
          vecs.push_back(input);
          continue;
        }
        for (auto use : v->uses()) {
          if (auto *out = dynamic_cast<SSDfgVecOutput *>(use->use())) {
            vecs.push_back(out);
            continue;
          }
        }
      } else {
        values.push_back(make_pair(edge->val(),edge->l()));
        //cout << edge->name() << " " << edge->val()->index() << "\n";
      }
    }
    //if(values.size() > 0) {
    //  cout << link->name() << " " << slot 
    //    << " has " << lp.slots[slot].edges.size() << " edges and " 
    //    << count_unique(values) << " values " << "\n";
    //}
    util = count_unique(values) + count_unique(vecs);
    int cur_ovr = util - link->max_util();
    ovr = std::max(cur_ovr, ovr);
    agg_ovr += std::max(cur_ovr, 0);
    max_util = std::max(util, max_util);
  }
}
