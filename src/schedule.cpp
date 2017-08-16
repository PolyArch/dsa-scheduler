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

void Schedule::stat_printLinkCount(){
	cout<<"============="<<endl;
	 cout<<"Link . . . . . Freq." << endl;
   for (auto& i: linkCount) {
     cout << (i.first)->name() << " . . . . . " << i.second << endl;
   }
	cout<<"============="<<endl;
}

//For a given pdgnode
//return the input or ouput port num if the pdfgnode is a
//sbinput ot sboutput
int Schedule::getPortFor(SbPDG_Node* sbpdg_in) 
{ 
  if (_sbnodeOf.count(sbpdg_in) != 0) {
    if (sbinput *assigned_sbinput = dynamic_cast<sbinput*>(_sbnodeOf[sbpdg_in])) {
      return assigned_sbinput->port();
    }

    if (sboutput *assigned_sboutput = dynamic_cast<sboutput*>(_sbnodeOf[sbpdg_in])) {
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
      _sbPDG->insert_vec_in(vec_input);

      //cout << "vp" << i << "  ";

      vector<bool> mask;
      mask.resize(port_m.size());
      for(unsigned mi = 0; mi < port_m.size(); ++mi) {
        mask[mi]=slices().read_slice(slice, 
                                     start_bits_vp_mask+mi,start_bits_vp_mask+mi);
        if(mask[mi]) {
          int sb_in_port = port_m[mi].first;
          sbinput* in = _sbModel->subModel()->get_input(sb_in_port);
          SbPDG_Input* pdg_in = new SbPDG_Input();
          pdg_in->setVPort(_sbPDG->num_vec_input());
          pdgnode_for[in]=pdg_in;
          _sbPDG->addInput(pdg_in);

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
      _sbPDG->insert_vec_out(vec_output);

      vector<bool> mask;
      mask.resize(port_m.size());
      for(unsigned mi = 0; mi < port_m.size(); ++mi) {
        mask[mi]=slices().read_slice(VP_MAP_SLICE_OUT, 
                                     start_bits_vp_mask+mi,start_bits_vp_mask+mi);
        if(mask[mi]) {
          sboutput* out = _sbModel->subModel()->get_output(port_m[mi].first);
          SbPDG_Output* pdg_out = new SbPDG_Output();
          pdg_out->setVPort(_sbPDG->num_vec_output());
          pdgnode_for[out]=pdg_out;
          _sbPDG->addOutput(pdg_out);
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


  //---------------------------------DECODE FUNC UNITS ---------------------------
  cur_slice=SWITCH_SLICE;
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j,++cur_slice) {
      sbfu* sbfu_node = &fus[i][j];
        
      //opcode
      uint64_t op=_bitslices.read_slice(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1);
      if(op!=0) { //if O
        pdg_inst = new SbPDG_Inst();
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
        //uint64_t p=_bitslices.read_slice(cur_slice,FU_PRED_INV_LOC,
        //                                           FU_PRED_INV_LOC+FU_PRED_INV_BITS-1);
        //pdg_inst->setPredInv(p);
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
    ++cur_slice;
    uint64_t imm = _bitslices.read_slice(cur_slice,0,63);
    ++cur_slice;
    assert(row <  _sbModel->subModel()->sizey());
    assert(col <  _sbModel->subModel()->sizex());
    sbfu* sbfu_node = &fus[col][row];
    assert(sbfu_node);
    //cout << "row,col" << row << " " << col << "\n";
    SbPDG_Node* node = pdgnode_for[sbfu_node];
    assert(node);
    SbPDG_Inst* inst = dynamic_cast<SbPDG_Inst*>(node);
    assert(inst->immSlot() != -1);
    inst->setImm(imm);
  }

  //routemap -- for each sbnode - inlink and outlinks
  //pdgnode_for -- sbnode to pdgnode mapping
  //posMap -- for each pdgnode, vector of incoming dirs

  reconstructSchedule(routeMap, pdgnode_for, posMap);

  // Iterate over FUs, get the inc_edge assoc. with each FU_INPUT, set extra lat.
  cur_slice=SWITCH_SLICE;
  for (int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
    for (int i = 0; i < _sbModel->subModel()->sizex(); ++i, ++cur_slice) {
      sbfu* sbfu_node = &fus[i][j];
      if (_assignNode.count(sbfu_node) != 0) {
        SbPDG_Inst* pdg_node = dynamic_cast<SbPDG_Inst*>(_assignNode[sbfu_node]);
 
        for (int i = 0; i < NUM_IN_FU_DIRS; ++i) {
          if (pdg_node->immSlot() == i) {
            //Do Nothing 
          } else if (i < (pdg_node->ops_end() - pdg_node->ops_begin())) {
            SbPDG_Edge* inc_edge = *(pdg_node->ops_begin() + i);
            if (!inc_edge) {
              continue;
            }
 
            // delay for each input
            int d1 = IN_DELAY_LOC + BITS_PER_DELAY * i;
            int d2 = d1 + BITS_PER_DELAY - 1;
            _extraLatOfEdge[inc_edge] = _bitslices.read_slice(cur_slice, d1, d2);
 
            //_bitslices.write(cur_slice, d1, d2, _extraLatOfEdge[inc_edge]);
            //cout << "slice: " << cur_slice 
            //     << ",delay:" << _extraLatOfEdge[inc_edge] << "\n";
 
          } else {
            assert(i != 0 && "can't be no slot for first input");
          }
        }
      }
    }
    cur_slice += 1; // because we skipped the switch
  }
 

  int max_lat, max_lat_mis;
  calcLatency(max_lat, max_lat_mis,true);
  if(max_lat != max_lat_mis) {
    cerr << "The FU input latencies don't match, this may or may not be a problem!\n";
  }

  calc_out_lat();

  return inst_histo;
}

//Configuration
//Slice0 - 64b: Active Input Ports (interfaces)
//Slice1 - 64b: Active Output Ports (interfaces)
//Slice2 - 64b: In Row 1 Delay
//Slice3 - 64b: In Row 2 Delay
//Slice4 - 64b: In Row 3 Delay
//Slice5 - Switch Data 


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


#if 0
  // --------------------------- ENCODE DELAY ------------------------------
   for(auto Iin=_sbModel->subModel()->input_begin(), 
      Ein=_sbModel->subModel()->input_end(); Iin!=Ein; ++Iin) {
     sbinput* sbinput_node = (sbinput*) &(*Iin);
     SbPDG_Node* input_pdgnode = pdgNodeOf(sbinput_node,0);
     int lat = _latOf[input_pdgnode];
     assert(lat<16 && "max delay supported by 4 bits is 15");
 
     int input_port_num = sbinput_node->port();
     int delay_slot,offset;

     if(input_port_num < 16) {
       delay_slot=DELAY_SLICE_1;
       offset=0;
     } else if(input_port_num < 32) {
       delay_slot=DELAY_SLICE_2;
       offset=16;
     } else { //if(input_port_num < 48)
       delay_slot=DELAY_SLICE_3;
       offset=32;
     }

     int start = BITS_PER_DELAY*(input_port_num - offset);
     _bitslices.write(delay_slot, start,start+3,lat);
   }
#endif
 
  xfer_link_to_switch(); // makes sure we have switch representation of link
  int cur_slice=5;

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

//sbdir.fu_dir_of(_bitslices.read_slice(cur_slice,FU_DIR_LOC+0*BITS_PER_FU_DIR,FU_DIR_LOC+1*BITS_PER_FU_DIR-1));
//sbdir.fu_dir_of(_bitslices.read_slice(cur_slice,FU_DIR_LOC+1*BITS_PER_FU_DIR,FU_DIR_LOC+2*BITS_PER_FU_DIR-1));
//sbdir.fu_dir_of(_bitslices.read_slice(cur_slice,FU_DIR_LOC+2*BITS_PER_FU_DIR,FU_DIR_LOC+3*BITS_PER_FU_DIR-1));


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

        if(_passthrough_nodes.count(sbfu_node)) {
          int cur_bit_pos=FU_DIR_LOC;
          int i = 0; //only one dir allowed
          unsigned p1 = cur_bit_pos+BITS_PER_FU_DIR*i;
          unsigned p2 = p1 + BITS_PER_FU_DIR-1;


          for(auto Ie=sbfu_node->ibegin(), Ee=sbfu_node->iend(); Ie!=Ee; ++Ie) {
            sblink* inlink=*Ie;
            if(_assignLink.count(inlink)!=0) {
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
        if(_assignNode.count(sbfu_node)!=0) {
          SbPDG_Inst* pdg_node = 
            dynamic_cast<SbPDG_Inst*>(_assignNode[sbfu_node]);
          
          int cur_bit_pos=FU_DIR_LOC;

          for(int i = 0; i < NUM_IN_FU_DIRS; ++i) {
            unsigned p1 = cur_bit_pos+BITS_PER_FU_DIR*i;
            unsigned p2 = p1 + BITS_PER_FU_DIR-1;

            if(pdg_node->immSlot()==i) {
              _bitslices.write(cur_slice,p1,p2,sbdir.encode_fu_dir(SbDIR::IM));  //imm slot for FU
            } else if(i  < (pdg_node->ops_end()-pdg_node->ops_begin())) {
              SbPDG_Edge* inc_edge = *(pdg_node->ops_begin()+i);
              if(!inc_edge) {continue;}
              SbPDG_Node* inc_pdg_node = inc_edge->def();
              
              bool assigned=false;
              for(auto Ie=sbfu_node->ibegin(), Ee=sbfu_node->iend(); Ie!=Ee; ++Ie) {
                sblink* inlink=*Ie;
                if(_assignLink.count(inlink)!=0
                  &&_assignLink[inlink]==inc_pdg_node) {
                  assert(inlink->dir() != SbDIR::END_DIR);
                  int in_encode = sbdir.encode_fu_dir(inlink->dir()); //get the encoding of the dir
                  _bitslices.write(cur_slice,p1,p2,in_encode);      //input direction for each FU in
                  assigned=true;
                  break;
                }
              }
              assert(assigned);

              //delay for each input
              int d1=IN_DELAY_LOC+BITS_PER_DELAY*i;
              int d2=d1+BITS_PER_DELAY-1;
              _bitslices.write(cur_slice, d1, d2, _extraLatOfEdge[inc_edge]);
              //cout << "slice: " << cur_slice 
              //   << ",delay:" << _extraLatOfEdge[inc_edge] << "\n";

            } else {
              assert(i!=0 && "can't be no slot for first input");
            }

          }
          
          //print predicate
          //if(pdg_node->predInv()) {
          //  _bitslices.write(cur_slice,FU_PRED_INV_LOC,
          //                             FU_PRED_INV_LOC+FU_PRED_INV_BITS-1,1);
          //} 
         
          //opcode encdoing
          unsigned op_encode = sbfu_node->fu_def()->encoding_of(pdg_node->inst());
          _bitslices.write(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1,op_encode);
        }

      } //end if for func encode
    
    }//end for switch x
  }//end for switch y 

  //cout << "cur slice: " << cur_slice << "\n";
  //--------------------------------------- ENCODE CONSTANTS ------------------------
  for(int i = 0; i < _sbModel->subModel()->sizex()+1; ++i) {    
    for(int j = 0; j < _sbModel->subModel()->sizey()+1; ++j) {
      sbfu* sbfu_node = &fus[i][j];
      if(_assignNode.count(sbfu_node)!=0) {
        SbPDG_Inst* pdg_node = dynamic_cast<SbPDG_Inst*>(_assignNode[sbfu_node]);
        if(pdg_node->immSlot()!=-1) {
           cout << i << " " << j << " " << pdg_node->immSlot() << "\n";
           _bitslices.write(cur_slice,ROW_LOC,ROW_LOC+ROW_BITS-1,j);
           _bitslices.write(cur_slice,COL_LOC,COL_LOC+COL_BITS-1,i);
           ++cur_slice;
           _bitslices.write(cur_slice,0,63,pdg_node->imm());
           ++cur_slice;
        }
      }
    }
  } 
  //cout << "cur slice: " << cur_slice << "\n";

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

      if(_passthrough_nodes.count(sbfu_node)) {
         os << i << "," << j << ": ";
         os << "Copy ";

         //Where did it come from?
         for(auto Ie=sbfu_node->ibegin(), Ee=sbfu_node->iend(); Ie!=Ee; ++Ie) {
           sblink* inlink=*Ie;
           if(_assignLink.count(inlink)!=0) {
              os << SbDIR::dirName(inlink->dir(),true);   //reverse
              os << " -  -  \n";
           }
         }
      }

      if(_assignNode.count(sbfu_node)!=0) {
        SbPDG_Inst* pdg_node = dynamic_cast<SbPDG_Inst*>(_assignNode[sbfu_node]);
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
              if(_assignLink.count(inlink)!=0
                &&_assignLink[inlink]==inc_pdg_node) {
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
                    pdg_in = new SbPDG_Input();
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
                    
                  pdg_out = new SbPDG_Output();
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
          pdg_inst = new SbPDG_Inst();

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
    cerr << "SbPDG_Node: " << pdgnode->name() << " is not scheduled\n"; 
    return;
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
    
    cout << cur_link->name() << " gets " << cur_edge->name() << "\n";

    _assignEdgeLink[cur_link].insert(cur_edge);
    
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
  _assignEdgeLink.clear();
  
  
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
      cout<<_latOf[pdgout]<<" ";
    }
    cout<<endl;
  }
}

bool Schedule::checkOutputMatch() {
  /*SbPDG::const_output_iterator Io,Eo;
  cout<<"Output Latency: ";
  for(Io=_sbPDG->output_begin(),Eo=_sbPDG->output_end();Io!=Eo;++Io) {
    SbPDG_Output* pdgout = *Io;
    cout<<_latOf[pdgout]<<" ";
  }
  cout<<endl;*/
  int n = _sbPDG->num_vec_output();
  for (int i=0; i<n; i++) {
    SbPDG_VecOutput* vec_out = _sbPDG->vec_out(i);
    int lat = -1;
    for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      if (lat == -1) {
        lat = _latOf[pdgout];
      }
      if (lat != _latOf[pdgout]) {
        assert(0);
        return false;
      }
    }
  }
  return true;
}

bool Schedule::fixLatency(int &max_lat, int &max_lat_mis) {
  bool succ = true;
  succ = fixLatency_fwd(max_lat, max_lat_mis); //fwd pass
  if (!succ) return false;
  succ = fixLatency_bwd(max_lat, max_lat_mis); //bwd pass
  if (!succ) return false;
  calcLatency(max_lat, max_lat_mis); //fwd pass
  succ = checkOutputMatch();
  if (!succ) return false;
  return true;
}

bool Schedule::fixLatency_bwd(int &max_lat, int &max_lat_mis) {
  int n = _sbPDG->num_vec_output();
  for (int i=0; i<n; i++) { //iterate over output vectors
    SbPDG_VecOutput* vec_out = _sbPDG->vec_out(i);
    int maxLat = 0;
    
    for(unsigned m=0; m < vec_out->num_outputs(); ++m) { //iterate over each cgra_port to find max latency of the vector port
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      if (_latOf[pdgout] > maxLat) {
        maxLat = _latOf[pdgout];
      }
    }
    unordered_set<SbPDG_Node*> visited;
    //TODO: sort ports from small to large latency
    for (unsigned m=0; m < vec_out->num_outputs(); ++m) { //iterate over each cgra_ports and fix each individual port
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      int ed = maxLat - _latOf[pdgout]; //extra delay to insert
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
    _extraLatOfEdge[source_pdgedge] += ed;
    if (_extraLatOfEdge[source_pdgedge] > 15) {
       //cout<<"ed = " << _extraLatOfEdge[source_pdgedge] << " exceeded 16"<<endl;
     //assert(0);
      return false;
    }
  }
  //cout<<"Found one that does not go beyond 15!\n";
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
       lat_edge[firstOutLink]=_latOf[pdgnode];
    }
  }
    
    
 //Outlinks of all the inputs 
  while (!openset.empty()) {
    sblink* inc_link = openset.front(); 
    openset.pop_front();
    //cout << "Link: "<<inc_link->name() << "\n";   

    //dest node
    sbnode* node = inc_link->dest();
    sbnode::const_iterator I,E,II,EE;
    
    

    if(sbfu* next_fu = dynamic_cast<sbfu*>(node)) {
      SbPDG_Node* next_pdgnode = pdgNodeOf(node);
      //cout << next_fu->name() << "\n"; 
      if(!next_pdgnode && !_passthrough_nodes.count(node)) {
        assert(next_pdgnode);
        cout << "problem with latency calculation!\n";
        max_lat=-1;
        max_lat_mis=-1;
        return false;
      }

      SbPDG_Inst* next_pdginst = dynamic_cast<SbPDG_Inst*>(next_pdgnode); 
      assert(next_pdginst || _passthrough_nodes.count(node));
    
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
     
      if (everyone_is_here && !_passthrough_nodes.count(node)) {
        for(II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
          sblink* inlink = *II;
          SbPDG_Node* origNode = pdgNodeOf(inlink);
          if(origNode != NULL) {
            SbPDG_Edge* edge = origNode->getLinkTowards(next_pdgnode);
            if(lat_edge[inlink]<latency) {
              int diff = latency - lat_edge[inlink]; //latency per edge
              assert(edge);
              assert(((_extraLatOfEdge.count(edge)==0) || (_extraLatOfEdge[edge]==diff)) && "Error: Someone else set this edge before!");
              _extraLatOfEdge[edge]=diff;
            }
            //cout<<"Edge Name: "<<edge->name()<< " extra lat:" << _extraLatOfEdge[edge] << endl;
            //cout << "(fwdPass) Edge "<<edge->name()<<"  maxLat: " << latency << "  lat_edge: "<<lat_edge[inlink]<<"  extralat: " << _extraLatOfEdge[edge] << "\n";
          }
        }
        //cout << next_fu->name() << " latency " << latency <<"\n";
      }

      if (everyone_is_here) {
        sblink* new_link = next_fu->getFirstOutLink();
        if(_passthrough_nodes.count(node)) {
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
      //TODO: check we aren't over-riding something
      _latOf[pdgnode]=lat_edge[inc_link];

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

void Schedule::calcLatency(int &max_lat, int &max_lat_mis, bool warnMismatch) {
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
       openset.push_back(firstOutLink);
       lat_edge[firstOutLink]=_latOf[pdgnode];
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
    
    SbPDG_Node* cur_pdgnode = pdgNodeOf(inc_link);
    assert(cur_pdgnode);
    

    if(sbfu* next_fu = dynamic_cast<sbfu*>(node)) {
      SbPDG_Node* next_pdgnode = pdgNodeOf(node);
      //cout << next_fu->name() << "\n"; 
      if(!next_pdgnode && !_passthrough_nodes.count(node)) {
        assert(next_pdgnode);
        cout << "problem with latency calculation!\n";
        max_lat=-1;
        max_lat_mis=-1;
        return;
      }

      SbPDG_Inst* next_pdginst = dynamic_cast<SbPDG_Inst*>(next_pdgnode); 
      assert(next_pdginst || _passthrough_nodes.count(node));
    
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
        // Latency should be the same across all the incoming edges
        for (II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
          sblink* inlink = *II;
          SbPDG_Node* origNode = pdgNodeOf(inlink);
          if (origNode != NULL) {
            int curLat = lat_edge[inlink];
            if(!_passthrough_nodes.count(node)) {
              SbPDG_Edge* edge = origNode->getLinkTowards(next_pdgnode);
              assert(edge);
              curLat +=  _extraLatOfEdge[edge];
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
               if(!_passthrough_nodes.count(node)) {
                 SbPDG_Edge* edge = origNode->getLinkTowards(next_pdgnode);
                 cout << "(calcLat) Edge "<<edge->name()<< "  lat_edge: "<<lat_edge[inlink] << "  extralat:" << _extraLatOfEdge[edge] << "\n";
               } else {
                 cout << "passthrough\n";
               }

            }
          }
        }
      }



      if (everyone_is_here) {
        // Update latency of outgoing edge
        sblink* new_link = next_fu->getFirstOutLink();

        if (_passthrough_nodes.count(node)) {
          lat_edge[new_link] = max_latency + 1; //TODO: Check this
        } else { //regular inst
          lat_edge[new_link] = max_latency + inst_lat(next_pdginst->inst());
        }
        openset.push_back(new_link);
        
        //if (dynamic_cast<sboutput*>(new_link->dest())) {
        // cout<<"(Func) link: "<<new_link->name()<<" lat: "<<lat_edge[new_link]<<endl; 
        //}
        int diff = max_latency-low_latency;
        if(diff>max_lat_mis) {
          max_lat_mis=diff;
        }
      }
    } else if (dynamic_cast<sboutput*>(node)) {
      sboutput* sb_out = dynamic_cast<sboutput*>(node);
      SbPDG_Node* pdgnode = pdgNodeOf(sb_out);
      _latOf[pdgnode]=lat_edge[inc_link];
      //cout<<"(Output) link: "<<inc_link->name()<<" lat: "<<lat_edge[inc_link]<<endl; 
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
}

void Schedule::tracePath(sbnode* sbspot, SbPDG_Node* pdgnode, 
    map<sbnode*, map<SbDIR::DIR,SbDIR::DIR> >& routeMap, 
    map<sbnode*, SbPDG_Node* >& pdgnode_for, 
    map<SbPDG_Node*, vector<SbDIR::DIR> >& posMap) {
  
  //_assignNode[sbspot]=pdgnode;  //perform the assignment
  
  assign_node(pdgnode,sbspot);
  
  vector<pair<sbnode*, SbDIR::DIR> > worklist;

  sblink* firstLink = sbspot->getFirstOutLink();
  assign_link(pdgnode,firstLink);
  
  sbnode* startItem = firstLink->dest();
  SbDIR::DIR initialDir = firstLink->dir();
  worklist.push_back(make_pair(startItem,initialDir));

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
    
    sbnode* curItem = worklist.back().first;
    SbDIR::DIR inDir = worklist.back().second;
    worklist.pop_back();
    
    map<SbDIR::DIR,SbDIR::DIR>::iterator I,E;
    for(I=routeMap[curItem].begin(), E=routeMap[curItem].end(); I!=E; ++I) {
      SbDIR::DIR newOutDir = I->first;
      SbDIR::DIR newInDir = I->second;
      
      if(inDir == newInDir) { //match!

        //sblink* inLink = curItem->getInLink(newInDir);
        
        sblink* outLink = curItem->getOutLink(newOutDir);
        assign_link(pdgnode,outLink);
        
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
          
          _assignNode[sbout]=dest_pdgnode;  //perform the assignment
          _sbnodeOf[dest_pdgnode]=sbout;

          _sbPDG->connect(pdgnode,dest_pdgnode,0, SbPDG_Edge::data); 

        } else if(sbfu* fu_node = dynamic_cast<sbfu*>(nextItem)) {

          //cerr << SbDIR::dirName(newInDir) << " -> " << SbDIR::dirName(newOutDir) 
          //     << "    (" << fu_node->x() << " " << fu_node->y() << " .. FU)" << "\n";


          //auto* pdgnode = pdgnode_for[fu_node];
          //cout << pdgnode->name() << "\n";

          SbPDG_Inst* dest_pdgnode = dynamic_cast<SbPDG_Inst*>(pdgnode_for[fu_node]);
          assert(dest_pdgnode);
          
          int slot=0;
          for(; slot < NUM_IN_FU_DIRS; ++slot) {
            if(posMap[dest_pdgnode][slot] == outLink->dir()) {
              break; 
            }
          }
          
          assert(slot>=0 && slot <NUM_IN_FU_DIRS);

          if(slot==2 && !dest_pdgnode->predInv()) {
            _sbPDG->connect(pdgnode,dest_pdgnode,slot, SbPDG_Edge::ctrl_true);
          } else if(slot==2 && dest_pdgnode->predInv()) {
            _sbPDG->connect(pdgnode,dest_pdgnode,slot, SbPDG_Edge::ctrl_false);
          } else {
            _sbPDG->connect(pdgnode,dest_pdgnode,slot, SbPDG_Edge::data); 
          }
        
        } else if(sbswitch* sbsw = dynamic_cast<sbswitch*>(nextItem)){ //must be switch
//          cerr << SbDIR::dirName(newInDir) << " -> " << SbDIR::dirName(newOutDir) 
//               << "    (" << sbsw->x() << " " << sbsw->y() << ")" << "\n";
          sbsw=sbsw;
          worklist.push_back(make_pair(nextItem,newOutDir));
        } else {
          assert(0);
        }
      }
    }


  }
  
}

