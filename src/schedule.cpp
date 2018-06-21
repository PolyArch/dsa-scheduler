#include "schedule.h"

#include <map>
#include <regex>
#include <iostream>
#include <fstream>

#include "model_parsing.h"
#include "sbpdg.h"
#include "sbinst.h"
#include <assert.h>
#include <regex>
#include <list>
#include  <iomanip>

using namespace std;
using namespace SB_CONFIG;

//Scheduling Interface

extern "C" void libsbscheduler_is_present() {}

void Schedule::clear_sbpdg() {
  if(_sbPDG) {
    delete _sbPDG;
    _sbPDG=NULL;
  } 
}

//For a given pdgnode
//return the input or ouput port num if the pdfgnode is a
//sbinput ot sboutput
int Schedule::getPortFor(SbPDG_Node* sbpdg_in) 
{ 
  if (sbnode* n = locationOf(sbpdg_in)) {
    if (sbinput *assigned_sbinput = dynamic_cast<sbinput*>(n)) {
      return assigned_sbinput->port();
    }

    if (sboutput *assigned_sboutput = dynamic_cast<sboutput*>(n)) {
      return assigned_sboutput->port();
    }
  }
  return -1; 
}

std::map<SB_CONFIG::sb_inst_t,int> Schedule::interpretConfigBits() {
  std::map<SB_CONFIG::sb_inst_t,int> inst_histo;

  SbDIR sbdir;
  vector< vector<sbfu> >& fus = _sbModel->subModel()->fus();
  SbPDG_Inst  *pdg_inst;

  map<sbnode*, map<SbDIR::DIR,SbDIR::DIR> > routeMap;
  map<SbPDG_Node*, vector<SbDIR::DIR> > posMap;
  map<sbnode*, SbPDG_Node*> pdgnode_for;
  _sbPDG = new SbPDG();

  std::set<uint64_t> inputs_used;   //vector ports used
  std::set<uint64_t> outputs_used;

   //Associating the PDG Nodes from the configuration bits, with the vector ports defined by the hardware
  int start_bits_vp_mask=0;
  int total_bits_vp_mask=0;
  int slice=VP_MAP_SLICE_1;
  for(auto& port_pair : _sbModel->subModel()->io_interf().in_vports) {
    int i = port_pair.first; //index of port
    std::vector<std::pair<int, std::vector<int> > >& port_m = port_pair.second;

    //port mapping of 1 vector port - cgra_port_num: vector offset elements
    total_bits_vp_mask+=port_m.size();
  
    if(start_bits_vp_mask < 64 && total_bits_vp_mask >64) {
      start_bits_vp_mask=port_m.size();
      total_bits_vp_mask=port_m.size();
      slice=VP_MAP_SLICE_2;
    }

    if(slices().read_slice(IN_ACT_SLICE,i,i)) {
      SbPDG_VecInput* vec_input = new SbPDG_VecInput("I",_sbPDG->num_vec_input()); 
      //vec_input->setLocMap(pm);
      //_sbPDG->insert_vec_in(vec_input);

      //cout << "vp" << i << "  ";

      vector<bool> mask;
      mask.resize(port_m.size());
      for(unsigned mi = 0; mi < port_m.size(); ++mi) {
        mask[mi]=slices().read_slice(slice, 
                                     start_bits_vp_mask+mi,start_bits_vp_mask+mi);
        if(mask[mi]) {
          int sb_in_port = port_m[mi].first;
          sbinput* in = _sbModel->subModel()->get_input(sb_in_port);
          SbPDG_Input* pdg_in = new SbPDG_Input(_sbPDG);
          pdg_in->setVPort(_sbPDG->num_vec_input());
          pdgnode_for[in]=pdg_in;
          _sbPDG->addInput(pdg_in); //add input to pdg
          vec_input->addInput(pdg_in); //add input to vector

          //cout << mi << " (" << in->port() << ")";
        }
      }
      //cout << "\n";
      assign_vport(vec_input,make_pair(true/*input*/,i),mask);
    }
    
    start_bits_vp_mask=total_bits_vp_mask; //next
  }

  start_bits_vp_mask=0;
  total_bits_vp_mask=0;

  for(auto& port_pair : _sbModel->subModel()->io_interf().out_vports) {
    int i = port_pair.first; //index of port
    std::vector<std::pair<int, std::vector<int> > >& port_m = port_pair.second;

    total_bits_vp_mask+=port_m.size();

    if(slices().read_slice(OUT_ACT_SLICE,i,i)) { //activate output port
      SbPDG_VecOutput* vec_output = new SbPDG_VecOutput("O",_sbPDG->num_vec_output()); 
      //_sbPDG->insert_vec_out(vec_output);

      vector<bool> mask;
      mask.resize(port_m.size());
      for(unsigned mi = 0; mi < port_m.size(); ++mi) {
        mask[mi]=slices().read_slice(VP_MAP_SLICE_OUT, 
                                     start_bits_vp_mask+mi,start_bits_vp_mask+mi);
        if(mask[mi]) {
          sboutput* out = _sbModel->subModel()->get_output(port_m[mi].first);
          SbPDG_Output* pdg_out = new SbPDG_Output(_sbPDG);
          pdg_out->setVPort(_sbPDG->num_vec_output());
          pdgnode_for[out]=pdg_out;
          _sbPDG->addOutput(pdg_out);
          vec_output->addOutput(pdg_out); //add output to vector
        }
      }
      assign_vport(vec_output,make_pair(false/*input*/,i),mask);
    }

    start_bits_vp_mask=total_bits_vp_mask; //next
  }


  //TODO: NOT NEEDED
  //Read input nodes
  for(int i = 0; i < 64; ++i) {  //64 ports ? 
    uint64_t inact = _bitslices.read_slice(IN_ACT_SLICE ,i,i);
    
    if(inact) {
      auto& vp = _sbModel->subModel()->io_interf().in_vports[i];
      for(auto& p : vp) {     //iterate through ports of vector port i
        inputs_used.insert(p.first);        //cgra port
      }
    }
  }

  //Read output nodes
  for(int i = 0; i < 64; ++i) {
   uint64_t outact= _bitslices.read_slice(OUT_ACT_SLICE,i,i);

    //If the outport !=0
    if(outact) {
      auto& vp = _sbModel->subModel()->io_interf().out_vports[i];
      for(auto& p : vp) {     //iterate through ports of vector port i
        outputs_used.insert(p.first);
      }
    }
  }

  int cur_slice=SWITCH_SLICE;       //5th slice in bitslice

  //In1, In2, In3, Opcode, S1, S2, ... S8, Row
  vector< vector<sbswitch> >& switches = _sbModel->subModel()->switches();
  
  for(int i = 0; i < _sbModel->subModel()->sizex()+1; ++i) {
    bool left = (i==0);
    bool right = (i==_sbModel->subModel()->sizex());
    for(int j = 0; j < _sbModel->subModel()->sizey()+1; ++j,++cur_slice) {
      bool top = (j==0);
      bool bottom = (j==_sbModel->subModel()->sizey());
      sbswitch* sbsw = &switches[i][j];

     // cout << i << "," << j << "\n";

      //read the [Row]
      int row = _bitslices.read_slice(cur_slice, ROW_LOC, ROW_LOC+ROW_BITS-1);
      assert(row==j);

      int cur_bit_pos=SWITCH_LOC;

      //---------------------------------DECODE SWITCHES ---------------------------
      for(int o=0; o < NUM_OUT_DIRS; ++ o, cur_bit_pos += BITS_PER_DIR) {
        
        uint64_t b =_bitslices.read_slice(cur_slice,cur_bit_pos,
                                                   cur_bit_pos+BITS_PER_DIR-1);
        SbDIR::DIR out_dir = sbdir.dir_for_slot(o,top,bottom,left,right);
        SbDIR::DIR in_dir  = sbdir.decode(b,      top,bottom,left,right);
        in_dir = SbDIR::reverse(in_dir);

        sblink* inlink  = sbsw->getInLink(in_dir);      //get the link object with that dir
        if(!inlink) {
          //cout << "no in_dir:" << SbDIR::dirName(in_dir) << " bits:" << b << " (pos:" << o << ")\n";
          continue; //no worries, this wasn't even a valid inlink
        }

        sblink* outlink = sbsw->getOutLink(out_dir);
        if(!outlink) {
          //cout << "no out_dir:" << SbDIR::dirNameDBG(out_dir) << " loc:" << o << "\n";
          continue; //skip if no corresponding link
        }

        //cout << SbDIR::dirName(in_dir) << "->" <<
        //        SbDIR::dirName(out_dir) << ":";

        //cout << b << " @pos: " << o << "\n";

        inst_histo[SB_CONFIG::SB_Switch]+=1;

        assert(outlink->orig() == inlink->dest());
        assign_switch(sbsw,inlink,outlink);
        //For better or worse, reconstruct takes in a route map, yes, this is redundant
        //with assign_switch
        routeMap[sbsw][out_dir]=in_dir;

        //if(sbfu* fu =dynamic_cast<sbfu*>(outlink->dest())) {
        //  //create corresponding PDG Node
        //  pdg_inst = new SbPDG_Inst();
        //  pdgnode_for[fu]=pdg_inst;
        //  _sbPDG->addInst(pdg_inst);
        //} else 

        ////if the swithces out is an output node
        //if(sboutput* out = dynamic_cast<sboutput*>(outlink->dest())) {
        //  SbPDG_Output* pdg_out = new SbPDG_Output();         
        //  pdgnode_for[out]=pdg_out;             
        //  _sbPDG->addOutput(pdg_out);
        //  pdg_out->setVPort(out->port());
        //}
       
        ////if the incoming node was from sbinput node
        //if (sbinput* in=dynamic_cast<sbinput*>(inlink->orig())) {
        //  SbPDG_Input* pdg_in;
        //  //Need to check if this is actually one of the useful inputs
        //  if(inputs_used.count(in->port())) {
        //    if(pdgnode_for.count(in)==0) {
        //      cout << "Creating node for port " << in->port() << "\n";
        //      pdg_in = new SbPDG_Input();
        //      pdgnode_for[in]=pdg_in;
        //      _sbPDG->addInput(pdg_in);
        //    } else {
        //      pdg_in = dynamic_cast<SbPDG_Input*>(pdgnode_for[in]);
        //    }
        //    pdg_in->setVPort(in->port());
        //  }
        //}//end if

      }//end for
    
    }//end for j
  }//end for i


  for(int g=0; g < NUM_DFG_GROUPS; ++g) {
    //Read input nodes
    for(int i = 0; i < 32; ++i) {  //32 ports max
      int i_adj=(g%2)*32+i;
      uint64_t inact = _bitslices.read_slice(IN_ACT_GROUP12+g/2, i_adj, i_adj);
      if(inact) {
        SbPDG_VecInput* in_vec = 
          dynamic_cast<SbPDG_VecInput*>(vportOf(make_pair(true,i)));
        assert(in_vec);
        _sbPDG->insert_vec_in_group(in_vec,g);
      }
    }
    for(int i = 0; i < 32; ++i) {  //32 ports ? 
      int i_adj=(g%2)*32+i;
      uint64_t outact = _bitslices.read_slice(OUT_ACT_GROUP12+g/2, i_adj, i_adj);
      
      if(outact) {
        SbPDG_VecOutput* out_vec = 
          dynamic_cast<SbPDG_VecOutput*>(vportOf(make_pair(false,i)));
        assert(out_vec);
        _sbPDG->insert_vec_out_group(out_vec,g);
      }
    }
  }

  //---------------------------------DECODE FUNC UNITS ---------------------------
  cur_slice=SWITCH_SLICE;
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j,++cur_slice) {
      sbfu* sbfu_node = &fus[i][j];
        
      //opcode
      uint64_t op=_bitslices.read_slice(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1);
      if(op!=0) { //if O
        pdg_inst = new SbPDG_Inst(_sbPDG);
        stringstream verif_name;
        verif_name << i << "-" << j;
        pdg_inst->set_verif_id(verif_name.str());

        pdgnode_for[sbfu_node]=pdg_inst;
        _sbPDG->addInst(pdg_inst);

        auto inst=sbfu_node->fu_def()->inst_of_encoding(op);
        pdg_inst->setInst(inst);

        inst_histo[inst]+=1;

        //8-switch_dirs + 4bits_for_row
        unsigned cur_bit_pos=FU_DIR_LOC;
    
        for(int f=0; f < NUM_IN_FU_DIRS; ++f, cur_bit_pos += BITS_PER_FU_DIR) {
          uint64_t b=_bitslices.read_slice(cur_slice,cur_bit_pos,
                                                     cur_bit_pos+BITS_PER_FU_DIR-1);
          SbDIR::DIR dir = sbdir.fu_dir_of(b);
          assert(f!=0 || (f==0 && dir != SbDIR::END_DIR)); 
          posMap[pdg_inst].push_back(dir);      //incoming FU dir
          if(dir == SbDIR::IM) {
            pdg_inst->setImmSlot(f);
          }
        }//end for input fu dirs
  
        //predictate inverse
        uint64_t p=_bitslices.read_slice(cur_slice,FU_PRED_INV_LOC,
                                                   FU_PRED_INV_LOC+FU_PRED_INV_BITS-1);
        pdg_inst->setPredInv(p);
      } else {
        //cout << i << " " << j << " not mapped\n";
      }
    }
    cur_slice+=1; // because we skipped the switch
  }

  cur_slice=SWITCH_SLICE +
       (_sbModel->subModel()->sizex()+1) * (_sbModel->subModel()->sizey()+1);

  //--------------------------------------- DECODE CONSTANTS ------------------------
  while((unsigned)cur_slice < _bitslices.size()) {
    int row = _bitslices.read_slice(cur_slice,ROW_LOC,ROW_LOC+ROW_BITS-1);
    int col = _bitslices.read_slice(cur_slice,COL_LOC,COL_LOC+COL_BITS-1);

    assert(row <  _sbModel->subModel()->sizey());
    assert(col <  _sbModel->subModel()->sizex());
    sbfu* sbfu_node = &fus[col][row];
    assert(sbfu_node);
    
    //cout << "row,col" << row << " " << col << "\n";
    SbPDG_Node* node = pdgnode_for[sbfu_node];
    assert(node);
    SbPDG_Inst* inst = dynamic_cast<SbPDG_Inst*>(node);


    uint64_t ctrl_bits=_bitslices.read_slice(cur_slice,CTRL_LOC,
                                                       CTRL_LOC+CTRL_BITS-1);

    inst->set_ctrl_bits(CtrlBits(ctrl_bits));

    bool has_imm = _bitslices.read_slice(cur_slice,IS_IMM_LOC,
                                                   IS_IMM_LOC+IS_IMM_BITS-1);

    ++cur_slice;

    if(has_imm) {
      uint64_t imm = _bitslices.read_slice(cur_slice,0,63);
      assert(inst->immSlot() != -1);
      inst->setImm(imm);

      ++cur_slice;
    }
  }

  //routemap -- for each sbnode - inlink and outlinks
  //pdgnode_for -- sbnode to pdgnode mapping
  //posMap -- for each pdgnode, vector of incoming dirs

  reconstructSchedule(routeMap, pdgnode_for, posMap);

  // Iterate over FUs, get the inc_edge assoc. with each FU_INPUT, set extra lat.
  cur_slice=SWITCH_SLICE;
  for (int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for (int j = 0; j < _sbModel->subModel()->sizey(); ++j, ++cur_slice) {
      sbfu* sbfu_node = &fus[i][j];
      if (nodeAssigned(sbfu_node) != 0) {
        SbPDG_Inst* pdg_node = dynamic_cast<SbPDG_Inst*>(pdgNodeOf(sbfu_node));
 
        for (int n = 0; n < NUM_IN_FU_DIRS; ++n) {
          if (pdg_node->immSlot() == n) {
            //Do Nothing 
          } else if (n < (pdg_node->ops_end() - pdg_node->ops_begin())) {
            SbPDG_Edge* inc_edge = *(pdg_node->ops_begin() + n);
            if (!inc_edge) {
              continue;
            }
 
            // delay for each input
            int d1 = IN_DELAY_LOC + BITS_PER_DELAY * n;
            int d2 = d1 + BITS_PER_DELAY - 1;
            set_edge_delay(_bitslices.read_slice(cur_slice, d1, d2),inc_edge);
 
            //_bitslices.write(cur_slice, d1, d2, _extraLatOfEdge[inc_edge]);
            //  cout <<  i << " " << j << " slice: " << cur_slice 
            //     << ",delay:" << _extraLatOfEdge[inc_edge] << "\n";
 
          } else {
            assert(n != 0 && "can't be no slot for first input");
          }
        }
      }
    }
    cur_slice += 1; // because we skipped the switch
  }

  calcLatency(_max_lat, _max_lat_mis,true);
/*  if(0 != _max_lat_mis) {
    cerr << "The FU input latencies don't match, this may or may not be a problem!\n";
  }*/

  calc_out_lat();

  return inst_histo;
}


//Write to a header file
void Schedule::printConfigBits(ostream& os, std::string cfg_name) {
  //print_bit_loc();

  //Step 1: Place bits into fields

  //Active Input Ports
  for(auto& i : _assignVPort) {
    std::pair<bool,int> pn = i.first;
    //SbPDG_Vec* pv = i.second;
    
    if(pn.first) { //INPUT
      _bitslices.write(IN_ACT_SLICE, pn.second,pn.second,1);
    } else { //OUTPUT
      _bitslices.write(OUT_ACT_SLICE,pn.second,pn.second,1);
    }
  }

  int num_vec_groups = _sbPDG->num_groups();
  assert(num_vec_groups <= NUM_DFG_GROUPS);
  for(int g = 0; g < num_vec_groups; ++g) {
    vector<SbPDG_VecInput*>& vec = _sbPDG->vec_in_group(g);
    for(auto* vec_in : vec) {
      int port_num = vecPortOf(vec_in).second;
      int bit_pos=(g%2)*32+port_num;
      _bitslices.write(IN_ACT_GROUP12+g/2, bit_pos, bit_pos, 1);
    }
  }

  for(int g = 0; g < num_vec_groups; ++g) {
    vector<SbPDG_VecOutput*>& vec = _sbPDG->vec_out_group(g);
    for(auto* vec_out : vec) {
      int port_num = vecPortOf(vec_out).second;
      int bit_pos=(g%2)*32+port_num;
      _bitslices.write(OUT_ACT_GROUP12+g/2, bit_pos, bit_pos, 1);
    }
  }



  // --------------------------- ENCODE VP MASK ------------------------------
  int start_bits_vp_mask=0;
  int total_bits_vp_mask=0;
  int slice=VP_MAP_SLICE_1;
  for(auto& port_pair : _sbModel->subModel()->io_interf().in_vports) {
    // pair<cgra_port_location, vector<indicies_into_vec>>
    std::vector<std::pair<int, std::vector<int> > >& port_m = port_pair.second;
    total_bits_vp_mask+=port_m.size();
 
    if(start_bits_vp_mask < 64 && total_bits_vp_mask >64) {
      start_bits_vp_mask=port_m.size();
      total_bits_vp_mask=port_m.size();
      slice=VP_MAP_SLICE_2;
    }

    //Is this port assigned?  if not can skip 
    if(SbPDG_Vec* pdg_vec_in = vportOf(make_pair(true/*input*/,port_pair.first))) {
      vector<bool> mask = maskOf(pdg_vec_in);

      for(unsigned i = 0; i < port_m.size(); ++i) {
        _bitslices.write(slice, start_bits_vp_mask+i,start_bits_vp_mask+i,mask[i]);
      }
    }
  
    start_bits_vp_mask=total_bits_vp_mask; //next
  }

  start_bits_vp_mask=0;
  total_bits_vp_mask=0;
  for(auto& port_pair : _sbModel->subModel()->io_interf().out_vports) {
    // pair<cgra_port_location, vector<indicies_into_vec>>
    std::vector<std::pair<int, std::vector<int> > >& port_m = port_pair.second;
    total_bits_vp_mask+=port_m.size();
 
    //Is this port assigned?  if not can skip 
    if(SbPDG_Vec* pdg_vec_out = vportOf(make_pair(false/*output*/,port_pair.first))) {
      vector<bool> mask = maskOf(pdg_vec_out);

      for(unsigned i = 0; i < port_m.size(); ++i) {
        _bitslices.write(VP_MAP_SLICE_OUT, start_bits_vp_mask+i,start_bits_vp_mask+i,mask[i]);
      }
    }
  
    start_bits_vp_mask=total_bits_vp_mask; //next
  }

  xfer_link_to_switch(); // makes sure we have switch representation of routing
  int cur_slice=SWITCH_SLICE;

  vector< vector<sbfu> >& fus = _sbModel->subModel()->fus();

  //In1, In2, In3, Opcode, S1, S2, ... S8, Row
  vector< vector<sbswitch> >& switches = _sbModel->subModel()->switches();
  for(int i = 0; i < _sbModel->subModel()->sizex()+1; ++i) {
    
    bool left = (i==0);                 //left edge switch
    bool right = (i==_sbModel->subModel()->sizex());    //right edge switch
    
    for(int j = 0; j < _sbModel->subModel()->sizey()+1; ++j,++cur_slice) {
      
      bool top = (j==0);
      bool bottom = (j==_sbModel->subModel()->sizey());

      //Write the [Row] -- corresponding row of the siwtch based on the j 
      _bitslices.write(cur_slice, ROW_LOC, ROW_LOC + ROW_BITS - 1, j);

      //---------------------------------ENCODE SWITCHES -------------------------------
      sbswitch* sbsw = &switches[i][j];

      //after switch respresntation
      std::map<SB_CONFIG::sblink*,SB_CONFIG::sblink*>& link_map = _assignSwitch[sbsw]; 
      if(link_map.size()!=0) {

        //Step 1: Encode all output switches with unused input

        //get used inputs
        std::set<int> used_in_enc;
        for(auto I=link_map.begin(), E=link_map.end();I!=E;++I) {
          sblink* inlink=I->second;
          int in_encode = sbdir.encode(inlink->dir(),top,bottom,left,right);
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
          sblink* outlink=I->first;
          sblink* inlink=I->second;
         
          auto in_port  = SbDIR::reverse(inlink->dir());
          int in_encode = sbdir.encode(in_port,top,bottom,left,right);
          int out_pos   = sbdir.slot_for_dir(outlink->dir(),top,bottom,left,right);
         
//          cout << SbDIR::dirName(inlink->dir()) << "->" <<
//                  SbDIR::dirName(outlink->dir()) << ":";
//
//          cout << in_encode << " @pos: " << out_pos << "\n";

          unsigned p1 = SWITCH_LOC + out_pos*BITS_PER_DIR;
          unsigned p2 = p1 + BITS_PER_DIR-1; 

          _bitslices.write(cur_slice,p1,p2,in_encode,false /*don't check*/);
        }

      }//end if


      //---------------------------------ENCODE FUNC UNITS ---------------------------
      //
      if(i < _sbModel->subModel()->sizex() && j < _sbModel->subModel()->sizey()) {
        sbfu* sbfu_node = &fus[i][j];

        if(isPassthrough(sbfu_node)) {
          int cur_bit_pos=FU_DIR_LOC;
          int i = 0; //only one dir allowed
          unsigned p1 = cur_bit_pos+BITS_PER_FU_DIR*i;
          unsigned p2 = p1 + BITS_PER_FU_DIR-1;


          for(auto Ie=sbfu_node->ibegin(), Ee=sbfu_node->iend(); Ie!=Ee; ++Ie) {
            sblink* inlink=*Ie;
            if(linkAssigned(inlink)) {
              int in_encode = sbdir.encode_fu_dir(inlink->dir()); //get encoding of dir
              _bitslices.write(cur_slice,p1,p2,in_encode);   //input dir for each FU in
              break;
            }
          }

          //opcode encdoing
          unsigned op_encode = sbfu_node->fu_def()->encoding_of(SB_CONFIG::SB_Copy);
          _bitslices.write(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1,op_encode);
        }
       
        //get the pdg node assigned to that FU  
        if(nodeAssigned(sbfu_node)!=0) {
          SbPDG_Inst* pdg_node = 
            dynamic_cast<SbPDG_Inst*>(pdgNodeOf(sbfu_node));
          
          int cur_bit_pos=FU_DIR_LOC;

          for(int n = 0; n < NUM_IN_FU_DIRS; ++n) {
            unsigned p1 = cur_bit_pos+BITS_PER_FU_DIR*n;
            unsigned p2 = p1 + BITS_PER_FU_DIR-1;

            if(pdg_node->immSlot()==n) {
              _bitslices.write(cur_slice,p1,p2,sbdir.encode_fu_dir(SbDIR::IM));  //imm slot for FU
            } else if(n  < (pdg_node->ops_end()-pdg_node->ops_begin())) {
              SbPDG_Edge* inc_edge = *(pdg_node->ops_begin()+n);
              if(!inc_edge) {continue;}
              SbPDG_Node* inc_pdg_node = inc_edge->def();
              
              bool assigned=false;
              for(auto Ie=sbfu_node->ibegin(), Ee=sbfu_node->iend(); Ie!=Ee; ++Ie) {
                sblink* inlink=*Ie;
                if(linkAssigned(inlink) && pdgNodeOf(inlink)==inc_pdg_node) {
                  assert(inlink->dir() != SbDIR::END_DIR);
                  int in_encode = sbdir.encode_fu_dir(inlink->dir()); //get the encoding of the dir
                  _bitslices.write(cur_slice,p1,p2,in_encode);      //input direction for each FU in
                  assigned=true;
                  break;
                }
              }
              assert(assigned);

              //delay for each input
              int d1=IN_DELAY_LOC+BITS_PER_DELAY*n;
              int d2=d1+BITS_PER_DELAY-1;
              _bitslices.write(cur_slice, d1, d2, edge_delay(inc_edge));
              //cout <<  i << " " << j << " slice: " << cur_slice 
              //   << ",delay:" << _extraLatOfEdge[inc_edge] << "\n";

            } else {
              assert(n!=0 && "can't be no slot for first input");
            }
          }
          
          //print predicate
          _bitslices.write(cur_slice,FU_PRED_INV_LOC,
                                     FU_PRED_INV_LOC+FU_PRED_INV_BITS-1,
                                     pdg_node->predInv());
         
          //opcode encdoing
          unsigned op_encode = sbfu_node->fu_def()->encoding_of(pdg_node->inst());
          _bitslices.write(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1,op_encode);
        }

      } //end if for func encode
    
    }//end for switch x
  }//end for switch y 

  //--------------------------------------- ENCODE CONSTANTS ------------------------
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {    
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* sbfu_node = &fus[i][j];
      if(nodeAssigned(sbfu_node)!=0) {
        SbPDG_Inst* pdg_node = dynamic_cast<SbPDG_Inst*>(pdgNodeOf(sbfu_node));
        bool has_imm_slot = pdg_node->immSlot()!=-1;
        uint64_t ctrl_bits = pdg_node->ctrl_bits();
        if(has_imm_slot || ctrl_bits) {
           //cout << i << " " << j << " " << pdg_node->immSlot() << "\n";
           _bitslices.write(cur_slice,ROW_LOC,ROW_LOC+ROW_BITS-1,j);
           _bitslices.write(cur_slice,COL_LOC,COL_LOC+COL_BITS-1,i);
           _bitslices.write(cur_slice,IS_IMM_LOC,
                                      IS_IMM_LOC+IS_IMM_BITS-1,has_imm_slot);
           _bitslices.write(cur_slice,CTRL_LOC,CTRL_LOC+CTRL_BITS-1,ctrl_bits);
           ++cur_slice;
        }
        if(has_imm_slot) {
           _bitslices.write(cur_slice,0,63,pdg_node->imm());
           ++cur_slice;
        }
      }
    }
  } 

  //Step 2: Write to output stream
  os << "#ifndef " << "__" << cfg_name << "_H__\n";
  os << "#define " << "__" << cfg_name << "_H__\n";

  os << "#define " << cfg_name << "_size " << _bitslices.size() << "\n\n";

  for(auto& i : _assignVPort) {
    std::pair<bool,int> pn = i.first;
    SbPDG_Vec* pv = i.second;
    os << "#define P_" << cfg_name << "_" << pv->name() << " " << pn.second << "\n";    
  }
  os<< "\n";

  os << "unsigned long long " << cfg_name << "_config[" << _bitslices.size() << "] = {";
  for(unsigned i = 0; i < _bitslices.size(); ++i) {
    if(i!=0) {os <<", ";}
    if(i%8==0) {os <<"\n";}
    os << "0x" << std::setfill('0') << std::setw(2) << std::hex << _bitslices.read_slice(i);
  }
  os << "};\n";
  os << std::dec;

  os << "#endif //" << cfg_name << "_H\n"; 
}

void Schedule::printConfigVerif(ostream& os) {
  for(unsigned i = 0; i < _bitslices.size(); ++i) {
    os << std::setfill('0') << std::setw(16) 
       << std::hex << _bitslices.read_slice(i) << "\n";
  }
}


//Print to a program config file (.cfg) -- text format for gui
void Schedule::printConfigText(ostream& os) 
{
  //print dimension
  os << "[dimension]\n";
  os << "height = " << _sbModel->subModel()->sizey() << "\n";
  os << "width = " << _sbModel->subModel()->sizex() << "\n";
  os << "\n";
  
  xfer_link_to_switch(); // makes sure we have switch representation of link

  //for each switch -- print routing if there is some
  os << "[switch]\n";
  vector< vector<sbswitch> >& switches = _sbModel->subModel()->switches();
  
  for(int i = 0; i < _sbModel->subModel()->sizex()+1; ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey()+1; ++j) {
    
      stringstream ss;
      sbswitch* sbsw = &switches[i][j];
    
      //get the out to in link map for each switch
      std::map<SB_CONFIG::sblink*,SB_CONFIG::sblink*>& link_map = _assignSwitch[sbsw]; 
      
      if(link_map.size()!=0) {
        ss << i << "," << j << ":\t";

        for(auto I=link_map.begin(), E=link_map.end();I!=E;++I) {
          //os << i << "," << j << ": ";
          //os << "\t";
        
        // print inputs and ordering 
          //Get the sbnode for pdgnode
          sblink* outlink=I->first;
          sblink* inlink=I->second;
          ss << SbDIR::dirName(inlink->dir(),true) << "->" << SbDIR::dirName(outlink->dir(),false) << "\t";
        }

      ss << "\n";
      os << ss.str();
      }

    }
  }

  os << "\n";
  
  os << "[funcunit]\n";
  //for each fu -- print assignment if there is some
  
  vector< vector<sbfu> >& fus = _sbModel->subModel()->fus();

  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* sbfu_node = &fus[i][j];

      if(isPassthrough(sbfu_node)) {
         os << i << "," << j << ": ";
         os << "Copy ";

         //Where did it come from?
         for(auto Ie=sbfu_node->ibegin(), Ee=sbfu_node->iend(); Ie!=Ee; ++Ie) {
           sblink* inlink=*Ie;
           if(linkAssigned(inlink)) {
              os << SbDIR::dirName(inlink->dir(),true);   //reverse
              os << " -  -  \n";
           }
         }
      }

      if(nodeAssigned(sbfu_node)!=0) {
        SbPDG_Inst* pdg_node = dynamic_cast<SbPDG_Inst*>(pdgNodeOf(sbfu_node));
        os << i << "," << j << ": ";
        os << config_name_of_inst(pdg_node->inst()); //returns sb_inst
        os << "\t";
        
        //Parse the inc-edges for each FU dir
        for(int i = 0; i < NUM_IN_FU_DIRS; ++i) {
          
          if(pdg_node->immSlot()==i) {
            os << "IM ";
          } else if(i < (pdg_node->ops_end()-pdg_node->ops_begin())) {
            
            SbPDG_Edge* inc_edge = *(pdg_node->ops_begin()+i);
            
            if(!inc_edge) {
              os << "-  ";
              continue;
            }
            
            SbPDG_Node* inc_pdg_node = inc_edge->def();
            
            for(auto Ie=sbfu_node->ibegin(), Ee=sbfu_node->iend(); Ie!=Ee; ++Ie) {
              sblink* inlink=*Ie;
              if(linkAssigned(inlink) && pdgNodeOf(inlink)==inc_pdg_node) {
                os << SbDIR::dirName(inlink->dir(),true);           //reverse the direction of inlink
                break;
              }
            }

            os << " ";
          } else {
            os << "-  ";
          }

        }//end for FU dir
        
        //print predicate
        if(pdg_node->predInv()) {
          os << "1 ";
        } else {
          os << "0 ";
        }
        
        if(pdg_node->subFunc()!=0) {
          os << "," << pdg_node->subFunc() << " ";
        }
        
        //print constant
        if(pdg_node->immSlot()!=-1) {
          os << pdg_node->getImmInt();
        }
        
        //more stuff?
        os << "\n";
      
      }//end if to check if sbfu ias assigned to each PDG_INST
      
    }
  }
  
  
  
  if(_wide_ports.size()>0) {
    os << "\n[wide-port]\n";
    for(unsigned i=0; i <_wide_ports.size(); ++i) {
      os << i << ": ";
      for(unsigned j = 0; j < _wide_ports[i].size(); ++j) {
        
        if(_wide_ports[i][j]==0xff) {
          os << "0xff ";
        } else {
          os << _wide_ports[i][j] << " ";
        }

      }//end for j loop
      os << "\n"; 
    }//end for i loop

  }
  
  os << "\n";

}

vector<string> getCaptureList(regex& rx, string str, string::const_iterator& start, bool only_once=false) {
    vector<string> list;
    //match_results<std::string::const_iterator> capturedTexts; 
    smatch capturedTexts;
    
    std::string::const_iterator end; 
    end = str.end(); 
    regex_constants::match_flag_type flags = regex_constants::match_default | regex_constants::match_continuous; 
    
    while(regex_search(start, end, capturedTexts, rx,flags)) 
    {
        start+=capturedTexts[0].length();
        
        for(unsigned i = 1; i < capturedTexts.size(); ++i) {
            list.push_back(capturedTexts[i].str());
        }
      //flags |= match_prev_avail; 
      //flags |= match_not_bob; 
      if(only_once) break;
    }
    return list;
}


//Reads Text format of the schedule
Schedule::Schedule(string filename) {
  enum loadstate{NoSchedState, Dimension, Switch,FuncUnit, WidePort} state;

  ifstream ifs(filename.c_str(), ios::in);
  
  if(ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  map<sbnode*, map<SbDIR::DIR,SbDIR::DIR> > routeMap;
  map<SbPDG_Node*, vector<SbDIR::DIR> > posMap;     //input dirs
  map<sbnode*, SbPDG_Node* > pdgnode_for;
  _sbPDG = new SbPDG();
  
  int newSizeX=-1;
  int newSizeY=-1;
  
  regex posRE("\\s*(\\d+),(\\d+):");
  regex arrowRE("\\s*(NE|SW|NW|SE|N|E|S|W|P0|P1)->(NE|SW|NW|SE|N|E|S|W|P0|P1)\\s*");
  regex dirRE("\\s*(NE|SW|NW|SE|N|E|S|W|P0|P1|IM|-)\\s*");
  regex numsRE("\\s*(\\d+)\\s*");
  regex fuRE("\\s*((?:\\w|\\-)*)\\s*");
  regex constRE("\\s*(-?(?:0x)?(?:\\d|[a-f]|[A-F])+)\\s*");
  regex portRE("\\s*(\\d+):");
  regex xnumsRE("\\s*(\\w+)\\s*");
  string line;

  state = NoSchedState;
  while (ifs.good()) {
      getline(ifs, line);
      ModelParsing::trim(line);
      
      if(line.empty() || ModelParsing::StartsWith(line,"#")) continue;

      if(ModelParsing::StartsWith(line,"[dimension]")) {
          state = Dimension;
          continue;
      } else if(ModelParsing::StartsWith(line,"[switch]")) {
          state = Switch;
          continue;
      } else if(ModelParsing::StartsWith(line,"[funcunit]")) {
          state = FuncUnit;
          continue;
      } else if(ModelParsing::StartsWith(line,"[wide-port]")) {
          state = WidePort;
          continue;
      }
      
      vector<string> lineElements;
      vector<string> caps;
      smatch captures;
      int posX,posY;
      string::const_iterator strIter;
      SbPDG_Output* pdg_out;
      SbPDG_Input *pdg_in;
      SbPDG_Inst  *pdg_inst;
      int pred;
      sbinput* d_input;
      sboutput* d_output;
      int portNum;
      
      switch (state) {
      case Dimension:
          lineElements.clear();
          ModelParsing::split(line,'=',lineElements);
          if(lineElements.size()!=2) {
              continue;
          }
          ModelParsing::trim(lineElements[0]); 
          ModelParsing::trim(lineElements[1]);
          
          if(ModelParsing::StartsWith(lineElements[0],"width")) {
              newSizeX=atoi(lineElements[1].c_str());
          } else if(ModelParsing::StartsWith(lineElements[0],"height")) {
              newSizeY=atoi(lineElements[1].c_str());
          } else {
              cerr << "Unrecognized param: " << lineElements[0] << "\n"; 
          }

          if(newSizeX>0 && newSizeY>0) {
            SubModel* subModel = new SubModel(newSizeX,newSizeY,SubModel::PortType::everysw,2,2);
            _sbModel = new SbModel(subModel);
          }
          break;
      case Switch:
          strIter = line.begin();
          caps = getCaptureList(posRE,line,strIter);
          
          if(caps.size()!=2) {
              return;// false;
          }
            
          posX = atoi(caps[0].c_str());
          posY = atoi(caps[1].c_str());
          
          //cout << "posX,posY: " << posX << " " << posY << "\n";
          
          caps = getCaptureList(arrowRE,line,strIter);
            
          if(caps.size()%2==1) {
              return;
          } else if (caps.size()==0) {
              continue;
          }

          //iterate over each inlink/outlinks of switch
          for(unsigned i = 0; i < caps.size(); i=i+2) {
              SbDIR::DIR inDir = SbDIR::toDir(caps[i],false);
              SbDIR::DIR outDir = SbDIR::toDir(caps[i+1],true);
              if(inDir == SbDIR::END_DIR || outDir == SbDIR::END_DIR) {
                  cerr << "Bad Direction";
                  continue;
              }

              //Check if siwtches dir is input
              if (SbDIR::isInputDir(inDir)) {
                  //int n = outDir == SbDIR::OP0 ? 0:1;
                  d_input=dynamic_cast<sbinput*>(_sbModel->subModel()->switchAt(posX,posY)
                    ->getInLink(inDir)->orig());
                  if(pdgnode_for.count(d_input)==0) {
                    pdg_in = new SbPDG_Input(_sbPDG);
                    pdgnode_for[d_input]=pdg_in;
                    _sbPDG->addInput(pdg_in);
                  } else {
                    pdg_in = dynamic_cast<SbPDG_Input*>(pdgnode_for[d_input]);
                  }
                  pdg_in->setVPort(d_input->port());
              } 
              
              if (SbDIR::isInputDir(outDir)) {
                  //int n = outDir == SbDIR::OP0 ? 0:1;
                  d_output=dynamic_cast<sboutput*>(_sbModel->subModel()->switchAt(posX,posY)
                    ->getOutLink(outDir)->dest());
                    
                  pdg_out = new SbPDG_Output(_sbPDG);
                  pdgnode_for[d_output]=pdg_out;
                  _sbPDG->addOutput(pdg_out);
                  pdg_out->setVPort(d_output->port());
              }
              routeMap[_sbModel->subModel()->switchAt(posX,posY)][outDir]=inDir;
          }

          break;
          
          
      case FuncUnit:
          strIter = line.begin();
          caps = getCaptureList(posRE,line,strIter);
          if(caps.size()!=2) {
              cerr << "Bad FU Pos\n";
              break;
          }
          posX = atoi(caps[0].c_str());
          posY = atoi(caps[1].c_str());

          //create corresponding PDG Node
          pdg_inst = new SbPDG_Inst(_sbPDG);

          //adding the sbnode to pdgnode mapping
          pdgnode_for[_sbModel->subModel()->fuAt(posX,posY)]=pdg_inst;
          _sbPDG->addInst(pdg_inst);

          //get instruction        
          caps = getCaptureList(fuRE,line,strIter,true);
          if(caps.size()!=1) {
              cerr << "Bad FU\n";
              break;
          }
         
          //FU TYPE
          ModelParsing::trim(caps[0]);
          pdg_inst->setInst(inst_from_config_name(caps[0].c_str()));

          //FuArray[posX][posY]->routeInitialLink();

          caps = getCaptureList(dirRE,line,strIter);

          if(caps.size()!=3) {
              cerr << "3 Directions Not Found\n";
              break;
          }

          //record ordering of inputs
          //posMap[pdg_inst].resize(3);
          for(int i = 0; i < 3; i++) {
            SbDIR::DIR inDir =  SbDIR::toDir(caps[i],false);
            posMap[pdg_inst].push_back(inDir);
            if(inDir == SbDIR::IM) {
              pdg_inst->setImmSlot(i);
            }
            /*
              //cout << "caps[" << i << "]=" << caps[i] << "\n";
              
              if(inDir == SbDIR::END_DIR) {
                  //empty direction
                  //cerr << "Bad Direction\n";
                  
                  continue;
              } else if(inDir == SbDIR::IM) {
                  //FuArray[posX][posY]->set_IM_slot(i);
                  
              } else {
                  //FuArray[posX][posY]->setFUInput(i,FuArray[posX][posY]->getArrow(inDir,true));
              }*/
          }

          caps = getCaptureList(numsRE,line,strIter,true);

          //numsRE.indexIn(line,strIndex);
          if(caps.size()!=1) {
              cerr << "No Predicate Inversion Symbol";
              break;
          }

          pred = atoi(caps[0].c_str());
          pdg_inst->setPredInv(pred);
          
          if(strIter >= line.end()) {
              continue;
          }

          caps = getCaptureList(constRE,line,strIter,true);
          if(caps.size()==1) {
              pdg_inst->setImm((uint32_t)strtoul(caps[0].c_str(),NULL,0));
              //FuArray[posX][posY]->setConst(constRE.capturedTexts()[1]);
              //strIndex+=constRE.matchedLength();
          }
          
          if(strIter>=line.end()) {
              continue;
          }

          //set extra text here
          //extraText = line.mid(strIndex);
          //FuArray[posX][posY]->setExtraText(extraText);

          break;

      case WidePort:
         strIter = line.begin();
         caps = getCaptureList(portRE,line,strIter);
         //cout << "caps size: " << caps.size() << "\n";
         if(caps.size()!=1) {
              cerr << "Bad Wide Port num\n";
              break;
         }
         //cout << "capL" << caps[0] << "\n";
         portNum = strtol(caps[0].c_str(),NULL,0);
         
         assert((unsigned)portNum==_wide_ports.size());
         
         _wide_ports.resize(portNum+1);
         
         caps = getCaptureList(xnumsRE,line,strIter);
        
         //8 offset elements for each wide vector port
         assert(caps.size() >0 && caps.size() <= 8);
         
         for(unsigned i = 0; i < caps.size(); ++i) {
            _wide_ports[portNum].push_back(strtol(caps[i].c_str(),NULL,0));
         }
         
          break;
          
      default:
          break;
      }

  }
  reconstructSchedule(routeMap,pdgnode_for,posMap);

/*
Debugging code to print out maps
  map<sbnode*, map<SbDIR::DIR,SbDIR::DIR> >::iterator II,EE;
  for(II=routeMap.begin(),EE=routeMap.end();II!=EE;++II) {
    map<SbDIR::DIR,SbDIR::DIR>::iterator I,E;
    cout << "node: " << (*(II)).first->name() << "\n";
    for(I=(*(II)).second.begin(), E=(*(II)).second.end(); I!=E; ++I) {
      SbDIR::DIR newOutDir = I->first;
      SbDIR::DIR newInDir = I->second;
      cerr << SbDIR::dirName(newOutDir) << " : " << SbDIR::dirName(newInDir) << "\n";
    }
  }
*/


return;
  
}

//reconstruct the schedule
void Schedule::reconstructSchedule(
                                    map<sbnode*, map<SbDIR::DIR,SbDIR::DIR> >& routeMap, 
                                    map<sbnode*, SbPDG_Node* >& pdgnode_for, 
                                    map<SbPDG_Node*, vector<SbDIR::DIR> >& posMap
                                    ) {
  //iterate over inputs (sbinputs)
  SubModel::const_input_iterator Iin,Ein;
  for(Iin=_sbModel->subModel()->input_begin(), 
      Ein=_sbModel->subModel()->input_end(); Iin!=Ein; ++Iin) {
    sbinput* sbinput_node = (sbinput*) &(*Iin);
   
    //get the pdg node for sbinput
    if(pdgnode_for.count(sbinput_node)!=0) {
        //cout << "reconstruction from input" << sbinput_node->name() << " " << sbinput_node->port() << "\n";
        SbPDG_Node* pdg_node = pdgnode_for[sbinput_node];
        tracePath(sbinput_node, pdg_node, routeMap, pdgnode_for, posMap);
    }
  }

  //iterate over fus
  vector< vector<sbfu> >& fus = _sbModel->subModel()->fus();
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* sbfu_node = &fus[i][j];
      if(pdgnode_for.count(sbfu_node)!=0) {
        //cout << "reconstruct from fu " << i << " " << j << "\n";
        SbPDG_Node* pdg_node = pdgnode_for[sbfu_node];
        tracePath(sbfu_node, pdg_node, routeMap, pdgnode_for, posMap);
      }
        
    }
  }
}

/*
struct edgeLinkItem{ 
  sblink* dlink;
  int SbPDG_Edge*
}
*/
void Schedule::calcAssignEdgeLink_single(SbPDG_Node* pdgnode) {
  //_assignEdgeLink

  //paris of link to edge
  list<pair<sblink*,SbPDG_Edge*>> openset;
  
  sbnode* node = locationOf(pdgnode);  

  if(!node) {
    //cerr << "SbPDG_Node: " << pdgnode->name() << " is not scheduled\n";
    return;
    //assert(0);
  }
  
  SbPDG_Node::const_edge_iterator I,E;
  for(I=pdgnode->ops_begin(), E=pdgnode->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    SbPDG_Edge* source_pdgegde = (*I);
    SbPDG_Node* source_pdgnode = source_pdgegde->def();

    //route edge if source pdgnode is scheduled
    if(isScheduled(source_pdgnode)) {
      sbnode::const_iterator Il,El;
      for(Il = node->ibegin(), El = node->iend(); Il!=El; ++Il) {
         sblink* link = *Il;
         if(pdgNodeOf(link)==source_pdgnode) {
           openset.push_back(make_pair(link,source_pdgegde));
         }
      }
    }
  }
  /*

  
  sbnode::const_iterator Il,El;
  for(Il = node->ibegin(), El = node->iend(); Il!=El; ++Il) {
      sblink* link = *Il;
      if(pdgNodeOf(link,config)!=NULL) {
        set<SbPDG_Edge*>& edgelist = _assignEdgeLink[make_pair(link,config)];
        set<SbPDG_Edge*>::iterator Ie,Ee;
        for(Ie=edgelist.begin(), Ee=edgelist.end(); Ie!=Ee; Ie++) {
          SbPDG_Edge* pdgedge = *Ie;
          openset.push_back(make_pair(make_pair(link,config),source_pdgedge));
        }
      }
  }*/
  
  while(!openset.empty()) {
    sblink* cur_link = openset.front().first;
    sbnode* cur_node = cur_link->orig();
    SbPDG_Edge* cur_edge = openset.front().second;
    SbPDG_Node* cur_pdgnode = cur_edge->def();
    openset.pop_front();
    
    //cout << cur_link->name() << " gets " << cur_edge->name() << "\n";

    _linkProp[cur_link].edges.insert(cur_edge);
    
    sbnode::const_iterator Il,El;
    for(Il = cur_node->ibegin(), El = cur_node->iend(); Il!=El; ++Il) {
        sblink* from_link = *Il;
        
        if(from_link->orig()==_sbModel->subModel()->cross_switch()) continue;
        if(from_link->orig()==_sbModel->subModel()->load_slice()  ) continue;
        
        if(pdgNodeOf(from_link)==cur_pdgnode) {
          openset.push_back(make_pair(from_link,cur_edge));
        }
    }
  }

}

void Schedule::calcAssignEdgeLink() {
  for(auto& i : _linkProp) {
    i.second.edges.clear();
  }
  
  SbPDG::const_inst_iterator Ii,Ei;
  for(Ii=_sbPDG->inst_begin(), Ei=_sbPDG->inst_end(); Ii!=Ei; ++Ii) {
    SbPDG_Inst* pdginst = *Ii; 
    calcAssignEdgeLink_single(pdginst);
  }
  
  SbPDG::const_output_iterator Io,Eo;
  for(Io=_sbPDG->output_begin(),Eo=_sbPDG->output_end();Io!=Eo;++Io) {
    SbPDG_Output* pdgout = *Io;
    calcAssignEdgeLink_single(pdgout);
  }
  
  
  /*
  
  SubModel::const_input_iterator I,E;
  for(I=_sbModel->subModel()->input_begin(),
      E=_sbModel->subModel()->input_end(); I!=E; ++I) {
     sbinput* cand_input = const_cast<sbinput*>(&(*I));
    
    if(pdgNodeOf(cand_input,config)!=NULL) {
       sblink* firstOutLink = cand_input->getFirstOutLink();
       openset.push_back(firstOutLink);
       lat_edge[firstOutLink]=0;
    }
  }
    
    
  
  while(!openset.empty()) {
    sblink* inc_link = openset.front(); 
    openset.pop_front();
    
    sbnode* node = inc_link->dest();
    sbnode::const_iterator I,E,II,EE;
    
    SbPDG_Node* cur_pdgnode = pdgNodeOf(inc_link,config);
    assert(cur_pdgnode);
    
    if(sbfu* next_fu = dynamic_cast<sbfu*>(node)) {
      SbPDG_Node* next_pdgnode = pdgNodeOf(node,config);
      //cout << next_fu->name() << "\n"; 
      assert(next_pdgnode);
      SbPDG_Inst* next_pdginst = dynamic_cast<SbPDG_Inst*>(next_pdgnode); 
      assert(next_pdginst);
    
      bool everyone_is_here = true;
      int latency=0;
      for(II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
        sblink* inlink = *II;
        if(pdgNodeOf(inlink,config) != NULL) {
          if(lat_edge.count(inlink)==1) {
            if(lat_edge[inlink]>latency) {
              latency=lat_edge[inlink];
            }
          } else {
            everyone_is_here = false;
            break;
          }
        }
      }
      if(everyone_is_here) {
        sblink* new_link = next_fu->getFirstOutLink();
        lat_edge[new_link] = latency + inst_lat(next_pdginst->inst());;
        openset.push_back(new_link);
      }
    } else if (dynamic_cast<sboutput*>(node)) {
      if(lat_edge[inc_link] > max_lat) {
        max_lat = lat_edge[inc_link];
      }
    } else {
    
      for(I = node->obegin(), E = node->oend(); I!=E; ++I) {
        sblink* link = *I;
        
        if(pdgNodeOf(link,config) == pdgNodeOf(inc_link,config)) {
          lat_edge[link] = lat_edge[inc_link] + 1;
          openset.push_back(link);
        }
      }
    }
  }
    

  return max_lat; 
  */
}

void Schedule::stat_printOutputLatency(){
  /*SbPDG::const_output_iterator Io,Eo;
  cout<<"Output Latency: ";
  for(Io=_sbPDG->output_begin(),Eo=_sbPDG->output_end();Io!=Eo;++Io) {
    SbPDG_Output* pdgout = *Io;
    cout<<_latOf[pdgout]<<" ";
  }
  cout<<endl;*/
  int n = _sbPDG->num_vec_output();
  cout << "** Output Vector Latencies **\n";
  for (int i=0; i<n; i++) {
    SbPDG_VecOutput* vec_out = _sbPDG->vec_out(i);
    cout<<vec_out->gamsName()<<": ";
    for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      cout<<latOf(pdgout)<<" ";
    }
    cout<<endl;
  }
}

void Schedule::checkOutputMatch(int &max_lat_mis) {

  //int violation=0;

  for (int i=0; i<_sbPDG->num_vec_output(); i++) {
    SbPDG_VecOutput* vec_out = _sbPDG->vec_out(i);
    int low_lat=10000, up_lat=0;
    for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      int lat = latOf(pdgout);
      if(lat < low_lat) {
        low_lat = lat;
      }
      if(lat > up_lat) {
        up_lat = lat;
      }
      int diff = up_lat - low_lat;
      if(diff > max_lat_mis) {
        max_lat_mis = diff;
      }
    }

    //int min_vec_lat = 0;
    //int max_vec_lat = 1000000;
    //for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
    //   SbPDG_Output* pdgout = vec_out->getOutput(m);

    //   auto p = _latBounds[pdgout];
    //   int min_inc_lat = p.first; 
    //   int max_inc_lat = p.second;
 
    //   //earliest starting time is *latest* incomming edge
    //   if(min_inc_lat > min_vec_lat) {
    //     min_vec_lat = min_inc_lat;
    //   }
    //   //latest starting time is *earliest* incomming edge
    //   if(max_inc_lat < max_vec_lat) {
    //     max_vec_lat = max_inc_lat;
    //   }
    //}

    //for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
    //   SbPDG_Output* pdgout = vec_out->getOutput(m);
    //   assign_lat_bounds(pdgout,min_vec_lat,max_vec_lat);
    //}


    //if(min_vec_lat > max_vec_lat) {
    //  violation += min_vec_lat-max_vec_lat;
    //}
  }
  //add_violation(violation);
}

bool Schedule::fixLatency(int &max_lat, int &max_lat_mis) {
  bool succ = true;
  for(auto& i : _edgeProp) {
    i.second.extra_lat=0;
  }

  succ = fixLatency_fwd(max_lat, max_lat_mis); //fwd pass
  if (succ) succ = fixLatency_bwd(max_lat, max_lat_mis); //bwd pass
  calcLatency(max_lat, max_lat_mis); //fwd pass
  checkOutputMatch(max_lat_mis);

  int max_lat2=0,max_lat_mis2=0;
  cheapCalcLatency(max_lat2, max_lat_mis2);


  if(max_lat2 != max_lat || max_lat_mis2 != max_lat_mis) {
     cout << "\nmax_lat:" << max_lat << " " << max_lat2 << "\n";
     cout << "max_lat_mis:" << max_lat_mis << " " << max_lat_mis2 << "\n";

    _sbPDG->printGraphviz("viz/remap-fail.dot");
    assert(max_lat2 == max_lat);
    assert(max_lat_mis2 == max_lat_mis);
  }  

  return succ;
}

bool Schedule::fixLatency_bwd(int &max_lat, int &max_lat_mis) {
  int n = _sbPDG->num_vec_output();
  for (int i=0; i<n; i++) { //iterate over output vectors
    SbPDG_VecOutput* vec_out = _sbPDG->vec_out(i);
    int maxLat = 0;
    
    for(unsigned m=0; m < vec_out->num_outputs(); ++m) { //iterate over each cgra_port to find max latency of the vector port
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      if (latOf(pdgout) > maxLat) {
        maxLat = latOf(pdgout);
      }
    }
    unordered_set<SbPDG_Node*> visited;
    //TODO: sort ports from small to large latency
    for (unsigned m=0; m < vec_out->num_outputs(); ++m) { //iterate over each cgra_ports and fix each individual port
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      int ed = maxLat - latOf(pdgout); //extra delay to insert
      //cout<<"Output "<<pdgout->name()<<"  maxLat: "<<maxLat<<" _latOf: "<<_latOf[pdgout]<<" ed: "<<ed<<endl;
      if (ed > 0) {
        bool succ = fixDelay(pdgout, ed, visited);
        if (!succ) return false;
      }
    }    
  }
  return true;
}


bool Schedule::fixDelay(SbPDG_Output* pdgout, int ed, unordered_set<SbPDG_Node*>& visited) {

  SbPDG_Node::const_edge_iterator I,E;
  SbPDG_Inst* n = pdgout->out_inst();
  if(!n || n->num_out()>1) { //bail if orig is input or has >1 use
    return false;
  }

  //assert(n && "output is always associated with an instruction node or input node");
  //cout<<"Fixing delay for node "<<n->name()<<" associated with output "<<pdgout->name()<<endl;
  if (visited.count(n) == 1) {
    cout<<"PDGNode "<<n->name()<<" is already visited!"<<endl;
    assert(0);
    return false;
  }
  visited.insert(n);

  for(I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate, constant within FU
    SbPDG_Edge* source_pdgedge = (*I);
    _edgeProp[source_pdgedge].extra_lat += ed;
    if (_edgeProp[source_pdgedge].extra_lat > _sbModel->maxEdgeDelay()) {
      return false;
    }
  }
  return true;
}


bool Schedule::fixLatency_fwd(int &max_lat, int &max_lat_mis) {
  list<sblink*> openset;
  //map<sbnode*,sblink*> came_from;
  map<sblink*,int> lat_edge;
  
  max_lat=0;  
  max_lat_mis=0;

  SubModel::const_input_iterator I,E;
  for(I=_sbModel->subModel()->input_begin(),
      E=_sbModel->subModel()->input_end(); I!=E; ++I) {
     sbinput* cand_input = const_cast<sbinput*>(&(*I));
   
    SbPDG_Node* pdgnode = pdgNodeOf(cand_input);
    if(pdgnode!=NULL) {
       sblink* firstOutLink = cand_input->getFirstOutLink();
       assert(firstOutLink);
       openset.push_back(firstOutLink);
       lat_edge[firstOutLink] = latOf(pdgnode);
    }
  }
    
    
 //Outlinks of all the inputs 
  while (!openset.empty()) {
    sblink* inc_link = openset.front(); 
    openset.pop_front();

    //dest node
    sbnode* node = inc_link->dest();
    sbnode::const_iterator I,E,II,EE;
    
    if(sbfu* next_fu = dynamic_cast<sbfu*>(node)) {
      SbPDG_Node* next_pdgnode = pdgNodeOf(node);
      //cout << next_fu->name() << "\n"; 
      if(!next_pdgnode && !isPassthrough(node)) {
        cout << "problem with latency calculation!\n";
        cout << node->name() << " has no mapping\n"; 
        max_lat=-1;
        max_lat_mis=-1;
        assert(next_pdgnode);
        return false;
      }

      SbPDG_Inst* next_pdginst = dynamic_cast<SbPDG_Inst*>(next_pdgnode); 
      assert(next_pdginst || isPassthrough(node));
    
      bool everyone_is_here = true;

      int latency=0;
      int low_latency=100000000;  //magic number, forgive me
      for(II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
        sblink* inlink = *II;
        if(pdgNodeOf(inlink) != NULL) {
          if(lat_edge.count(inlink)==1) {
            if(lat_edge[inlink]>latency) {
              latency=lat_edge[inlink];
            }
            if(lat_edge[inlink]<low_latency) {
              low_latency=latency;
            }
          } else {
            everyone_is_here = false;
            break;
          }
        }
      }
     
      if (everyone_is_here && !isPassthrough(node)) {
        for(II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
          sblink* inlink = *II;
          SbPDG_Node* origNode = pdgNodeOf(inlink);
          if(origNode != NULL) {
            SbPDG_Edge* edge = origNode->getLinkTowards(next_pdgnode);
            if(lat_edge[inlink]<latency) {
              int diff = latency - lat_edge[inlink]; //latency per edge
              assert(edge);
              if(diff > _sbModel->maxEdgeDelay()) {

                //cout << diff  << " > " << _sbModel->maxEdgeDelay() << "\n";
                return false;
              }
              set_edge_delay(diff,edge);
            }
            //cout<<"Edge Name: "<<edge->name()<< " extra lat:" << _extraLatOfEdge[edge] << endl;
            //cout << "(fwdPass) Edge "<<edge->name()<<"  maxLat: " << latency << "  lat_edge: "<<lat_edge[inlink]<<"  extralat: " << _extraLatOfEdge[edge] << "\n";
          }
        }
        //cout << next_fu->name() << " latency " << latency <<"\n";
      }

      if (everyone_is_here) {
        sblink* new_link = next_fu->getFirstOutLink();
        if(isPassthrough(node)) {
          lat_edge[new_link] = latency + 1; //TODO: Check this
        } else { //regular inst
          lat_edge[new_link] = latency + inst_lat(next_pdginst->inst());
        }
        openset.push_back(new_link);
        
        int diff = latency-low_latency;
        if (diff>max_lat_mis) {
          max_lat_mis=diff;
        }
      }
    } else if (dynamic_cast<sboutput*>(node)) {
      sboutput* sb_out = dynamic_cast<sboutput*>(node);
      SbPDG_Node* pdgnode = pdgNodeOf(sb_out);
      assign_lat(pdgnode,lat_edge[inc_link]);

      if(lat_edge[inc_link] > max_lat) {
        max_lat = lat_edge[inc_link];
      }
    } else {
      for(I = node->obegin(), E = node->oend(); I!=E; ++I) {
        sblink* link = *I;
        
        if(pdgNodeOf(link) == pdgNodeOf(inc_link)) {
          lat_edge[link] = lat_edge[inc_link] + 1;
          openset.push_back(link);
        }
      }
    }
  }
  return true;
}

void Schedule::calcNodeLatency(SbPDG_Inst* inst, int &max_lat, int &max_lat_mis) {
  int low_lat=MAX_SCHED_LAT, up_lat=0;

  if(isScheduled(inst)) {

    for(auto i = inst->ops_begin(), e=inst->ops_end();i!=e;++i) {
      SbPDG_Edge* edge=*i;
      if(edge == NULL) continue;
      
      SbPDG_Node* origNode = edge->def();

      if(isScheduled(origNode)) {
        if (origNode != NULL) {
          int edge_lat = edge_delay(edge) + link_count(edge)-1;
          assert(edge_lat >= 0);
          int lat = latOf(origNode) + edge_lat; 

          if(lat>up_lat) up_lat=lat;
          if(lat<low_lat) low_lat=lat;
        }
      }
    }
  }

  assign_lat_bounds(inst,low_lat,up_lat);

  int diff = up_lat - low_lat - _sbModel->maxEdgeDelay();
  if(diff>max_lat_mis) max_lat_mis=diff;

  int new_lat = inst_lat(inst->inst()) + up_lat;
  assign_lat(inst,new_lat);
  if(max_lat < new_lat) max_lat=new_lat;

  if(diff > 0) {
    add_violation(diff);
    record_violation(inst,diff);
  }
  //cout << "C-inst " << inst->name() << " " << lat_node[inst]  
  //  << " inst: " << inst << " " << inst->name() 
  //     << " up:" << up_lat << " low:" << low_lat << "\n";
  //if(lat_node[inst] > 10) {

  //  for(auto i = inst->ops_begin(), e=inst->ops_end();i!=e;++i) {
  //    SbPDG_Edge* edge=*i;
  //    cout << edge->name() << ": ";
  //    if(edge == NULL) continue;
  //    for(auto& i : _edgeProp[edge].links) {
  //      cout << i->name() << " "; 
  //    }
  //    cout << "\n";
  //  }
  //}
}

void Schedule::cheapCalcLatency(int &max_lat, int &max_lat_mis) {
  _totalViolation=0;

  std::vector<SbPDG_Inst*>& ordered_insts = _sbPDG->ordered_insts();
  for(SbPDG_Inst* inst : ordered_insts) {
    calcNodeLatency(inst,max_lat,max_lat_mis);
  }

  for (int i=0; i<_sbPDG->num_vec_output(); i++) {
    SbPDG_VecOutput* vec_out = _sbPDG->vec_out(i);
    int low_lat=MAX_SCHED_LAT, up_lat=0;

    if(vecMapped(vec_out)) { 

      for (unsigned m=0; m < vec_out->num_outputs(); ++m) {
        SbPDG_Output* pdgout = vec_out->getOutput(m);
        SbPDG_Edge* edge = pdgout->first_inc_edge();
  
        if(isScheduled(pdgout)) { 
          int lat = latOf(edge->def()) + edge_delay(edge) + link_count(edge)-1;
          //cout << "C " << vec_out->name() << m << " " << lat << " -- " 
          //     << " def_lat:" << lat_node[edge->def()] <<" ed:"<< edge_delay(edge) 
          //     << " link_count:" << link_count(edge) << "inst: " 
          //     << edge->def() << " " << edge->def()->name() << "\n";
         
          if(lat>up_lat) up_lat=lat;
          if(lat<low_lat) low_lat=lat;
        }
      }
    }

    assign_lat_bounds(vec_out,low_lat,up_lat);
 
    int diff = up_lat - low_lat;// - _sbModel->maxEdgeDelay();
    if(diff>max_lat_mis) max_lat_mis=diff;
  
    if(max_lat < up_lat) max_lat=up_lat;
 
    if(diff > 0) {
      add_violation(diff);
    } 
    //cout << "C " <<vec_out->name()<< " up:" << up_lat << " low:" << low_lat << "\n";
  }
}

void Schedule::calcLatency(int &max_lat, int &max_lat_mis, 
    bool warnMismatch) {
  list<sblink*> openset;
  //map<sbnode*,sblink*> came_from;
  unordered_map<sblink*,int> lat_edge;
  
  max_lat=0;  
  max_lat_mis=0;

  SubModel::const_input_iterator I,E;
  for(I=_sbModel->subModel()->input_begin(),
      E=_sbModel->subModel()->input_end(); I!=E; ++I) {
     sbinput* cand_input = const_cast<sbinput*>(&(*I));
   
    SbPDG_Node* pdgnode = pdgNodeOf(cand_input);
    if(pdgnode!=NULL) {
       sblink* firstOutLink = cand_input->getFirstOutLink();
       openset.push_back(firstOutLink);
       lat_edge[firstOutLink]=latOf(pdgnode);
    }
  }
    
    
 //Outlinks of all the inputs 
  while(!openset.empty()) {
    sblink* inc_link = openset.front(); 
    openset.pop_front();
    //cout << inc_link->name() << "\n";   

    //dest node
    sbnode* node = inc_link->dest();
    sbnode::const_iterator I,E,II,EE;
    
    if(sbfu* next_fu = dynamic_cast<sbfu*>(node)) {
      sblink* new_link = next_fu->getFirstOutLink();
      if(lat_edge.count(new_link)) {
        //cout << "skipping because full:" << new_link->name() << "\n";
        continue; //skip if we've done it already
      }

      SbPDG_Node* next_pdgnode = pdgNodeOf(node);
      //cout << next_fu->name() << "\n"; 
      if(!next_pdgnode && !isPassthrough(node)) {
        assert(next_pdgnode);
        cout << "problem with latency calculation!\n";
        max_lat=-1;
        max_lat_mis=-1;
        return;
      }

      SbPDG_Inst* next_pdginst = dynamic_cast<SbPDG_Inst*>(next_pdgnode); 
      assert(next_pdginst || isPassthrough(node));
    
      bool everyone_is_here = true;

      for(II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
        sblink* inlink = *II;
        if(pdgNodeOf(inlink) != NULL) {
          if(!lat_edge.count(inlink)) {
            everyone_is_here = false;
            break;
          }
        }
      }

      int max_latency=0;
      int low_latency=100000000;  //magic number, forgive me

 
      if (everyone_is_here) {
        //cout << "----------------------------------- DONE WITH " 
        //     <<  next_fu->name() << "\n";
        // Latency should be the same across all the incoming edges
        for (II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
          sblink* inlink = *II;
          SbPDG_Node* origNode = pdgNodeOf(inlink);
          if (origNode != NULL) {
            int curLat = lat_edge[inlink];
            //cout << "reading: " << inlink->name() << "\n";

            if(!isPassthrough(node)) {
              SbPDG_Edge* edge = origNode->getLinkTowards(next_pdgnode);
              if(!edge) {
                cout << "Edge: " << origNode->name() << " has no link towards "
                     << next_pdgnode->name() << "\n";
                cout << "dummy:" << next_pdginst->isDummy() << "\n";

                _sbPDG->printGraphviz("viz/remap-fail2.dot");
              
                assert(edge);
              }
              if(edge_delay(edge)) {
                curLat += edge_delay(edge);
              }
            }

            if(curLat>max_latency) {
              max_latency=curLat;
            }
            if(curLat<low_latency) {
              low_latency=curLat;
            }

            if(warnMismatch && max_latency != low_latency) {
               cout << "Mismatch, min_lat:" << low_latency 
                             << ", max_lat:" << max_latency 
                    << ", link:" << inlink->name() << "\n";
               if(!isPassthrough(node)) {
                 SbPDG_Edge* edge = origNode->getLinkTowards(next_pdgnode);
                 cout << "(calcLat) Edge "<<edge->name()<< "  lat_edge: "<<lat_edge[inlink] << "  extralat:" << edge_delay(edge) << "\n";
               } else {
                 cout << "passthrough\n";
               }

            }
          }
        }        
      }

      if (everyone_is_here) {
        // Update latency of outgoing edge
        if (isPassthrough(node)) {
          lat_edge[new_link] = max_latency + 1; //TODO: Check this
        } else { //regular inst
          int l = max_latency + inst_lat(next_pdginst->inst());
          lat_edge[new_link] = l;
          assign_lat(next_pdgnode,l);
        }

        //if(next_pdginst) {
        //  cout << "L " << next_pdginst->name() << " " << latOf(next_pdgnode) 
        //       << " up:" << max_latency << " low:" << low_latency << "\n";
        //}

        openset.push_back(new_link);

        //cout << "lat of " << next_pdgnode->name() 
        //     << ", old:" << _latOf[next_pdgnode]
        //     << ", new:" << max_latency << " " << lat_edge[new_link] << "\n";

        int diff = max_latency-low_latency - _sbModel->maxEdgeDelay();
        if(diff>max_lat_mis) {
          max_lat_mis=diff;
        }
      }
    } else if (dynamic_cast<sboutput*>(node)) {
      sboutput* sb_out = dynamic_cast<sboutput*>(node);
      SbPDG_Node* pdgnode = pdgNodeOf(sb_out);

      int l = lat_edge[inc_link];
      assign_lat(pdgnode,l);

      pdgnode->set_sched_lat(l); //for graphviz printing

      //cout<<"L " << pdgnode->name() << " "
      //       <<inc_link->name()<<" lat: "<<lat_edge[inc_link]<<endl; 
      if(lat_edge[inc_link] > max_lat) {
        max_lat = lat_edge[inc_link];
      }
    } else {
    
      for(I = node->obegin(), E = node->oend(); I!=E; ++I) {
        sblink* out_link = *I;        

        //We'll need to check to make sure if there is ambiguity between links
        if(have_switch_links()) {
          sbswitch* sbsw = dynamic_cast<sbswitch*>(node);
          assert(sbsw);
          sblink* new_inc_link = get_switch_in_link(sbsw,out_link);
          if(new_inc_link != inc_link) {
            continue;
          }
          assert(pdgNodeOf(out_link) == pdgNodeOf(inc_link));
        } else if(pdgNodeOf(out_link) != pdgNodeOf(inc_link)) {
          continue;
        }

        lat_edge[out_link] = lat_edge[inc_link] + 1;
        openset.push_back(out_link);
      }
    }
  }

  for(auto& i : lat_edge) {
    sblink* link = i.first;
   int lat = i.second;
   set_link_order(link,lat);
  }

  _max_lat=max_lat;
  _max_lat_mis=max_lat_mis; 

}

//Trace the path of a schedule to help re-create the PDG
//This is not a high-performance implementation, but shouldn't have to be
//because its not called often
void Schedule::tracePath(sbnode* sbspot, SbPDG_Node* pdgnode, 
    map<sbnode*, map<SbDIR::DIR,SbDIR::DIR> >& routeMap, 
    map<sbnode*, SbPDG_Node* >& pdgnode_for, 
    map<SbPDG_Node*, vector<SbDIR::DIR> >& posMap) {
  
  assign_node(pdgnode,sbspot);
  
  vector<tuple<sbnode*, SbDIR::DIR,std::vector<sblink*>> > worklist;

  sblink* firstLink = sbspot->getFirstOutLink();
  //FIXME: assign_link(pdgnode,firstLink); //TODO: Check if broke anything
  std::vector<sblink*> lvec;
  lvec.push_back(firstLink);

  sbnode* startItem = firstLink->dest();
  SbDIR::DIR initialDir = firstLink->dir();
  worklist.push_back(make_tuple(startItem,initialDir,lvec));

  //cerr << "---   tracing " << sbspot->name() << "   ---\n"; 
  
  while(!worklist.empty()) {
    
    //cerr << "worklist: ";
    //for(unsigned i = 0; i < worklist.size(); ++i) {
    //  sbnode* item = worklist[i].first;
    //  SbDIR::DIR dir = worklist[i].second;
    //  cerr << SbDIR::dirName(dir) << ", " << item->name() << "";
    //  cerr << " | ";
    //}
    //cerr << "\n";
   
    auto& item = worklist.back();
    sbnode* curItem = std::get<0>(item);
    SbDIR::DIR inDir = std::get<1>(item);

    worklist.pop_back();
    
    map<SbDIR::DIR,SbDIR::DIR>::iterator I,E;
    for(I=routeMap[curItem].begin(), E=routeMap[curItem].end(); I!=E; ++I) {
      SbDIR::DIR newOutDir = I->first;
      SbDIR::DIR newInDir = I->second;
      
      if(inDir == newInDir) { //match!

        //sblink* inLink = curItem->getInLink(newInDir);
        
        sblink* outLink = curItem->getOutLink(newOutDir);
        //FIXME: assign_link(pdgnode,outLink);
        std::vector<sblink*> links = std::get<2>(item);
        links.push_back(outLink);
        
        if(outLink==NULL) {
          cerr << "outlink is null: ";
          cerr << curItem->name() << " (dir:" << SbDIR::dirName(newOutDir) << "\n";
          assert(0);
        }

        sbnode* nextItem = outLink->dest();

        //cerr << "match" << curItem->name() << " (dir:" << SbDIR::dirName(newOutDir) 
        //     << " " << nextItem->name() << "\n";


       //Output node 
        if(sboutput* sbout = dynamic_cast<sboutput*>(nextItem)) {
          //cerr << SbDIR::dirName(newInDir) << " -> " << SbDIR::dirName(newOutDir) 
          //     << "    out port" << sbout->port() << "\n";

          SbPDG_Node* dest_pdgnode = pdgnode_for[sbout];
          assert(dest_pdgnode);
          
          //_assignNode[sbout]=dest_pdgnode;  //perform the assignment
          //_sbnodeOf[dest_pdgnode]=sbout;
          assign_node(dest_pdgnode,sbout);

          _sbPDG->connect(pdgnode,dest_pdgnode,0, SbPDG_Edge::data); 

        } else if(sbfu* fu_node = dynamic_cast<sbfu*>(nextItem)) {

          //cerr << SbDIR::dirName(newInDir) << " -> " << SbDIR::dirName(newOutDir) 
          //     << "    (" << fu_node->x() << " " << fu_node->y() << " .. FU)" << "\n";

          SbPDG_Inst* dest_pdgnode = dynamic_cast<SbPDG_Inst*>(pdgnode_for[fu_node]);
          assert(dest_pdgnode);

          int n_ops = SB_CONFIG::num_ops[dest_pdgnode->inst()];

          for(int slot = 0; slot < NUM_IN_FU_DIRS; ++slot) { 
            if(posMap[dest_pdgnode][slot] == outLink->dir()) {
              auto edge_type = SbPDG_Edge::data;
              if(slot==n_ops) edge_type = dest_pdgnode->predInv()? 
                             SbPDG_Edge::ctrl_false : SbPDG_Edge::ctrl_true;

              SbPDG_Edge * edge = _sbPDG->connect(pdgnode,dest_pdgnode,
                                                  slot, edge_type); 
              for(auto& i : links) {
                assign_edgelink(edge,i);
              }
            }
          }
        
        } else if(dynamic_cast<sbswitch*>(nextItem)){ //must be switch
          worklist.push_back(make_tuple(nextItem,newOutDir,links));
        } else {
          assert(0);
        }
      }
    }
  }
}

