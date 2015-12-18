#include "schedule.h"

#include <map>
#include <regex>
#include <iostream>
#include <fstream>

#include "model_parsing.h"
#include "dypdg.h"
#include "dyinst.h"
#include <assert.h>
#include <regex>
#include <list>

using namespace std;
using namespace SB_CONFIG;

//Scheduling Interface

extern "C" void libsbscheduler_is_present() {}

void Schedule::printAllConfigs(const char *base) {
  
  for(int i = 0; i < _n_configs; ++i) {
    std::stringstream ss;
    ss << base << "-" << i;
    printConfigText(ss.str().c_str(),i);
  }
  if(*base==4) {
    interpretConfigBits();
  }
}

std::pair<int,int> Schedule::getConfigAndPort(DyPDG_Node* dypdg_in) {
  if (_dynodeOf.count(dypdg_in) != 0) {
    if (dyinput *assigned_dyinput = dynamic_cast<dyinput*>(_dynodeOf[dypdg_in].first)) {
      return make_pair(_dynodeOf[dypdg_in].second, assigned_dyinput->port());
    }
    if (dyoutput *assigned_dyoutput=dynamic_cast<dyoutput*>(_dynodeOf[dypdg_in].first)){
      return make_pair(_dynodeOf[dypdg_in].second,assigned_dyoutput->port());
    }
  }
  return make_pair(-1,-1); 
}

//Old Interface

int Schedule::getPortFor(DyPDG_Node* dypdg_in) 
{ 
  if (_dynodeOf.count(dypdg_in) != 0) {
    if (dyinput *assigned_dyinput = dynamic_cast<dyinput*>(_dynodeOf[dypdg_in].first)) {
      return assigned_dyinput->port();
    }
    if (dyoutput *assigned_dyoutput = dynamic_cast<dyoutput*>(_dynodeOf[dypdg_in].first)) {
      return assigned_dyoutput->port();
    }
  }
  return -1; 
}

void Schedule::interpretConfigBits() {
  DyDIR dydir;
  vector< vector<dyfu> >& fus = _dyModel->subModel()->fus();
  DyPDG_Output* pdg_out;
  DyPDG_Input *pdg_in;
  DyPDG_Inst  *pdg_inst;

  map<dynode*, map<DyDIR::DIR,DyDIR::DIR> > routeMap;
  map<DyPDG_Node*, vector<DyDIR::DIR> > posMap;
  map<dynode*, DyPDG_Node* > pdgnode_for;
  _dyPDG = new DyPDG();

  std::set<uint64_t> inputs_used;
  std::set<uint64_t> outputs_used;

  //Read input nodes?
  for(int i = 0; i < 64; ++i) {
    uint64_t inact = _bitslices.read_slice(IN_ACT_SLICE ,i,i);
    uint64_t outact= _bitslices.read_slice(OUT_ACT_SLICE,i,i);
    if(inact) {
      auto& vp = _dyModel->subModel()->io_interf().in_vports[i];
      for(auto& p : vp) {     //iterate through ports of vector port i
        inputs_used.insert(p.first);
      }
    }
    if(outact) {
      auto& vp = _dyModel->subModel()->io_interf().out_vports[i];
      for(auto& p : vp) {     //iterate through ports of vector port i
        outputs_used.insert(p.first);
      }
    }
  }


  int cur_slice=SWITCH_SLICE;

  //In1, In2, In3, Opcode, S1, S2, ... S8, Row
  vector< vector<dyswitch> >& switches = _dyModel->subModel()->switches();
  for(int i = 0; i < _dyModel->subModel()->sizex()+1; ++i) {
    bool left = (i==0);
    bool right = (i==_dyModel->subModel()->sizex());
    for(int j = 0; j < _dyModel->subModel()->sizey()+1; ++j,++cur_slice) {
      bool top = (j==0);
      bool bottom = (j==_dyModel->subModel()->sizey());
      dyswitch* dysw = &switches[i][j];

     // cout << i << "," << j << "\n";

      //read the [Row]
      int row = _bitslices.read_slice(cur_slice,ROW_LOC,ROW_LOC+ROW_BITS-1);
      assert(row==j);

      int cur_bit_pos=SWITCH_LOC;

      //---------------------------------DECODE SWITCHES ---------------------------
      for(int o=0; o < NUM_OUT_DIRS; ++ o, cur_bit_pos += BITS_PER_DIR) {
        uint64_t b=_bitslices.read_slice(cur_slice,cur_bit_pos,
                                                   cur_bit_pos+BITS_PER_DIR-1);
        DyDIR::DIR out_dir = dydir.dir_for_slot(o,top,bottom,left,right);
        DyDIR::DIR in_dir  = dydir.decode(b,      top,bottom,left,right);
        in_dir = DyDIR::reverse(in_dir);

        dylink* inlink  = dysw->getInLink(in_dir);
        if(!inlink) {
          //cout << "no in_dir:" << DyDIR::dirName(in_dir) << " bits:" << b << " (pos:" << o << ")\n";
          continue; //no worries, this wasn't even a valid inlink
        }
        dylink* outlink = dysw->getOutLink(out_dir);
        if(!outlink) {
          //cout << "no out_dir:" << DyDIR::dirNameDBG(out_dir) << " loc:" << o << "\n";
          continue; //skip if no corresponding link
        }

        //cout << DyDIR::dirName(in_dir) << "->" <<
        //        DyDIR::dirName(out_dir) << ":";

        //cout << b << " @pos: " << o << "\n";


        assert(outlink->orig() == inlink->dest());
        assign_switch(dysw,inlink,outlink);
        //For better or worse, reconstruct takes in a route map, yes, this is redundant
        //with assign_switch
        routeMap[dysw][out_dir]=in_dir;

        //if(dyfu* fu =dynamic_cast<dyfu*>(outlink->dest())) {
        //  //create corresponding PDG Node
        //  pdg_inst = new DyPDG_Inst();
        //  pdgnode_for[fu]=pdg_inst;
        //  _dyPDG->addInst(pdg_inst);
        //} else 

        if(dyoutput* out = dynamic_cast<dyoutput*>(outlink->dest())) {
          pdg_out = new DyPDG_Output();
          pdgnode_for[out]=pdg_out;
          _dyPDG->addOutput(pdg_out);
          pdg_out->setVPort(out->port());
        }
        if (dyinput* in=dynamic_cast<dyinput*>(inlink->orig())) {
          //Need to check if this is actually one of the useful inputs
          if(inputs_used.count(in->port())) {
            if(pdgnode_for.count(in)==0) {
              pdg_in = new DyPDG_Input();
              pdgnode_for[in]=pdg_in;
              _dyPDG->addInput(pdg_in);
            } else {
              pdg_in = dynamic_cast<DyPDG_Input*>(pdgnode_for[in]);
            }
            pdg_in->setVPort(in->port());
          }
        }
      }
    }
  }
  //---------------------------------DECODE FUNC UNITS ---------------------------
  cur_slice=5;
  for(int i = 0; i < _dyModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _dyModel->subModel()->sizey(); ++j,++cur_slice) {
      dyfu* dyfu_node = &fus[i][j];
        
      //opcode
      uint64_t op=_bitslices.read_slice(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1);
      if(op!=0) { //if O
        pdg_inst = new DyPDG_Inst();
        pdgnode_for[dyfu_node]=pdg_inst;
        _dyPDG->addInst(pdg_inst);
        pdg_inst->setInst(dyfu_node->fu_def()->inst_of_encoding(op));

        //8-switch_dirs + 4bits_for_row
        unsigned cur_bit_pos=FU_DIR_LOC;
    
        for(int f=0; f < NUM_IN_FU_DIRS; ++f, cur_bit_pos += BITS_PER_FU_DIR) {
          uint64_t b=_bitslices.read_slice(cur_slice,cur_bit_pos,
                                                     cur_bit_pos+BITS_PER_FU_DIR-1);
          DyDIR::DIR dir = dydir.fu_dir_of(b);
          assert(f!=0 || (f==0 && dir != DyDIR::END_DIR)); 
          posMap[pdg_inst].push_back(dir);
          if(dir == DyDIR::IM) {
            pdg_inst->setImmSlot(i);
          }
        }
  
        //predictate inverse
        uint64_t p=_bitslices.read_slice(cur_slice,FU_PRED_INV_LOC,
                                                   FU_PRED_INV_LOC+FU_PRED_INV_BITS-1);
        pdg_inst->setPredInv(p);
      }
    }
    cur_slice+=1; // because we skipped the switch
  }
  reconstructSchedule(routeMap,pdgnode_for,posMap);

}

//Configuration
//64: Active Input Ports (interfaces)
//64: Active Output Ports (interfaces)
//64: In Row 1 Delay
//64: In Row 2 Delay
//64: In Row 3 Delay
//Switch Data 

void Schedule::printConfigBits(ostream& os, std::string cfg_name) {
  //Step 1: Place bits into fields

  //Active Input Ports
  for(auto& i : _assignVPort) {
    std::pair<bool,int> pn = i.first;
    //DyPDG_Vec* pv = i.second;
    if(pn.first) { //INPUT
      _bitslices.write(IN_ACT_SLICE, pn.second,pn.second,1);
    } else { //OUTPUT
      _bitslices.write(OUT_ACT_SLICE,pn.second,pn.second,1);
    }
  }

  xfer_link_to_switch(); // makes sure we have switch representation of link
  int cur_slice=5;

  vector< vector<dyfu> >& fus = _dyModel->subModel()->fus();

  //In1, In2, In3, Opcode, S1, S2, ... S8, Row
  vector< vector<dyswitch> >& switches = _dyModel->subModel()->switches();
  for(int i = 0; i < _dyModel->subModel()->sizex()+1; ++i) {
    bool left = (i==0);
    bool right = (i==_dyModel->subModel()->sizex());
    for(int j = 0; j < _dyModel->subModel()->sizey()+1; ++j,++cur_slice) {
      bool top = (j==0);
      bool bottom = (j==_dyModel->subModel()->sizey());

      cout << i << "," << j << "\n";

      //Write the [Row]
      _bitslices.write(cur_slice,ROW_LOC,ROW_LOC+ROW_BITS-1,j);

      //---------------------------------ENCODE SWITCHES -------------------------------
      dyswitch* dysw = &switches[i][j];
      std::map<SB_CONFIG::dylink*,SB_CONFIG::dylink*>& link_map = _assignSwitch[dysw]; 
      if(link_map.size()!=0) {

        //Step 1: Encode all output switches with unused input

        std::set<int> used_in_enc;
        for(auto I=link_map.begin(), E=link_map.end();I!=E;++I) {
          dylink* inlink=I->second;
          int in_encode = dydir.encode(inlink->dir(),top,bottom,left,right);
          used_in_enc.insert(in_encode);
        }
        int unused_dir_enc=NUM_IN_DIRS; //num input dirs
        for(int i = 0; i < NUM_IN_DIRS; ++i) {
          if(used_in_enc.count(i)==0) {
            unused_dir_enc=i;
            break;
          }
        }
        assert(unused_dir_enc!=NUM_IN_DIRS&&"no unused direction,does this ever happen?");
        //cout << "unused:" << unused_dir_enc << "\n";

        for(int i = 0; i < NUM_OUT_DIRS; ++i) {
          unsigned p1 = SWITCH_LOC+i*BITS_PER_DIR;
          unsigned p2 = p1 + BITS_PER_DIR-1; 
          _bitslices.write(cur_slice,p1,p2,unused_dir_enc);
        }

        //Step 2: Fill in correct switches
        for(auto I=link_map.begin(), E=link_map.end();I!=E;++I) {
          dylink* outlink=I->first;
          dylink* inlink=I->second;
         
          auto in_port  = DyDIR::reverse(inlink->dir());
          int in_encode = dydir.encode(in_port,top,bottom,left,right);
          int out_pos   = dydir.slot_for_dir(outlink->dir(),top,bottom,left,right);
         
//          cout << DyDIR::dirName(inlink->dir()) << "->" <<
//                  DyDIR::dirName(outlink->dir()) << ":";
//
//          cout << in_encode << " @pos: " << out_pos << "\n";

          unsigned p1 = SWITCH_LOC+out_pos*BITS_PER_DIR;
          unsigned p2 = p1 + BITS_PER_DIR-1; 

          _bitslices.write(cur_slice,p1,p2,in_encode,false /*don't check*/);
        }
      }


      //---------------------------------ENCODE FUNC UNITS ---------------------------
      if(i < _dyModel->subModel()->sizex() && j < _dyModel->subModel()->sizey()) {
        dyfu* dyfu_node = &fus[i][j];

        if(_assignNode.count(make_pair(dyfu_node,0))!=0) {
          DyPDG_Inst* pdg_node = dynamic_cast<DyPDG_Inst*>(_assignNode[make_pair(dyfu_node,0)]);
          int cur_bit_pos=FU_DIR_LOC;
          for(int i = 0; i < NUM_IN_FU_DIRS; ++i) {
            unsigned p1 = cur_bit_pos+BITS_PER_FU_DIR*i;
            unsigned p2 = p1 + BITS_PER_FU_DIR-1;

            if(pdg_node->immSlot()==i) {
              _bitslices.write(cur_slice,p1,p2,0b100);
            } else if(i < (pdg_node->ops_end()-pdg_node->ops_begin())) {
              DyPDG_Edge* inc_edge = *(pdg_node->ops_begin()+i);
              if(!inc_edge) {continue;}
              DyPDG_Node* inc_pdg_node = inc_edge->def();
              
              bool assigned=false;
              for(auto Ie=dyfu_node->ibegin(), Ee=dyfu_node->iend(); Ie!=Ee; ++Ie) {
                dylink* inlink=*Ie;
                if(_assignLink.count(make_pair(inlink,0))!=0
                  &&_assignLink[make_pair(inlink,0)]==inc_pdg_node) {
                  assert(inlink->dir() != DyDIR::END_DIR);
                  int in_encode = dydir.encode_fu_dir(inlink->dir());
                  _bitslices.write(cur_slice,p1,p2,in_encode);
                  assigned=true;
                  break;
                }
              }
              assert(assigned);
            } else {
              assert(i!=0); //can't be no slot for first input
            }
          }
          
          //print predicate
          if(pdg_node->predInv()) {
            _bitslices.write(cur_slice,FU_PRED_INV_LOC,
                                       FU_PRED_INV_LOC+FU_PRED_INV_BITS-1,1);
          } 
          
          unsigned op_encode = dyfu_node->fu_def()->encoding_of(pdg_node->inst());
          _bitslices.write(cur_slice,OPCODE_LOC,OPCODE_LOC+OPCODE_BITS-1,op_encode);
        }
      }
    }
  }


  //Step 2: Write to output stream
  os << "#ifndef " << cfg_name << "_H\n";
  os << "#define " << cfg_name << "_H\n";

  os << "#define " << cfg_name << "_size " << _bitslices.size() << "\n\n";

  for(auto& i : _assignVPort) {
    std::pair<bool,int> pn = i.first;
    DyPDG_Vec* pv = i.second;
    os << "#define P_" << cfg_name << "_" << pv->name() << " " << pn.second << "\n";    
  }
  os<< "\n";

  os << "unsigned long long " << cfg_name << "_config[" << _bitslices.size() << "] = {";
  for(unsigned i = 0; i < _bitslices.size(); ++i) {
    if(i!=0) {os <<", ";}
    if(i%8==0) {os <<"\n";}
    os << "0x" << std::hex << _bitslices.read_slice(i);
  }
  os << "};\n";
  os << std::dec;

  os << "#endif //" << cfg_name << "_H\n"; 
}


void Schedule::printConfigText(ostream& os, int config) 
{
  //print dimension
  os << "[dimension]\n";
  os << "height = " << _dyModel->subModel()->sizey() << "\n";
  os << "width = " << _dyModel->subModel()->sizex() << "\n";
  os << "\n";
  
  xfer_link_to_switch(); // makes sure we have switch representation of link

  //for each switch -- print routing if there is some
  os << "[switch]\n";
  vector< vector<dyswitch> >& switches = _dyModel->subModel()->switches();
  for(int i = 0; i < _dyModel->subModel()->sizex()+1; ++i) {
    for(int j = 0; j < _dyModel->subModel()->sizey()+1; ++j) {
    
      stringstream ss;
      dyswitch* dysw = &switches[i][j];
     
      std::map<SB_CONFIG::dylink*,SB_CONFIG::dylink*>& link_map = _assignSwitch[dysw]; 
      if(link_map.size()!=0) {
        ss << i << "," << j << ":\t";

        for(auto I=link_map.begin(), E=link_map.end();I!=E;++I) {
          dylink* outlink=I->first;
          dylink* inlink=I->second;
          ss << DyDIR::dirName(inlink->dir(),true) << "->" << DyDIR::dirName(outlink->dir(),false) << "\t";
        }

      ss << "\n";
      os << ss.str();
      }
    }
  }

  os << "\n";
  
  os << "[funcunit]\n";
  //for each fu -- print assignment if there is some
  
  vector< vector<dyfu> >& fus = _dyModel->subModel()->fus();
  for(int i = 0; i < _dyModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _dyModel->subModel()->sizey(); ++j) {
      dyfu* dyfu_node = &fus[i][j];
      if(_assignNode.count(make_pair(dyfu_node,config))!=0) {
        DyPDG_Inst* pdg_node = dynamic_cast<DyPDG_Inst*>(_assignNode[make_pair(dyfu_node,config)]);
        os << i << "," << j << ": ";
        os << config_name_of_inst(pdg_node->inst());
        os << "\t";
        
        // print inputs and ordering 
        
        DyPDG_Node::const_edge_iterator I,E;
        //for(I=pdg_node->ops_begin(),E=pdg_node->ops_end();I!=E;++I) {
        //DyPDG_Node* inc_pdg_node=*I;
        for(int i = 0; i < NUM_IN_FU_DIRS; ++i) {
          
          if(pdg_node->immSlot()==i) {
            os << "IM ";
          } else if(i < (pdg_node->ops_end()-pdg_node->ops_begin())) {
            
            DyPDG_Edge* inc_edge = *(pdg_node->ops_begin()+i);
            
            if(!inc_edge) {
              os << "-  ";
              continue;
            }
            
            DyPDG_Node* inc_pdg_node = inc_edge->def();
            
            dynode::const_iterator Ie,Ee;
            for(Ie=dyfu_node->ibegin(), Ee=dyfu_node->iend(); Ie!=Ee; ++Ie) {
              dylink* inlink=*Ie;
              if(_assignLink.count(make_pair(inlink,config))!=0
                &&_assignLink[make_pair(inlink,config)]==inc_pdg_node) {
                os << DyDIR::dirName(inlink->dir(),true);
                break;
              }
            }
            os << " ";
          } else {
            os << "-  ";
          }

        }
        
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
      }
      
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
      }
      os << "\n"; 
    }
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



Schedule::Schedule(string filename, bool multi_config) {
  enum loadstate{Dimension, Switch,FuncUnit, WidePort} state;

  ifstream ifs(filename.c_str(), ios::in);
  
  if(ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  map<dynode*, map<DyDIR::DIR,DyDIR::DIR> > routeMap;
  map<DyPDG_Node*, vector<DyDIR::DIR> > posMap;
  map<dynode*, DyPDG_Node* > pdgnode_for;
  _dyPDG = new DyPDG();
  
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
      DyPDG_Output* pdg_out;
      DyPDG_Input *pdg_in;
      DyPDG_Inst  *pdg_inst;
      int pred;
      dyinput* d_input;
      dyoutput* d_output;
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
            SubModel* subModel = new SubModel(newSizeX,newSizeY,SubModel::PortType::everysw,2,2,multi_config);
            _dyModel = new DyModel(subModel);
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

          for(unsigned i = 0; i < caps.size(); i=i+2) {
              DyDIR::DIR inDir = DyDIR::toDir(caps[i],false);
              DyDIR::DIR outDir = DyDIR::toDir(caps[i+1],true);
              if(inDir == DyDIR::END_DIR || outDir == DyDIR::END_DIR) {
                  cerr << "Bad Direction";
                  continue;
              }
              if (DyDIR::isInputDir(inDir)) {
                  //int n = outDir == DyDIR::OP0 ? 0:1;
                  d_input=dynamic_cast<dyinput*>(_dyModel->subModel()->switchAt(posX,posY)
                    ->getInLink(inDir)->orig());
                  if(pdgnode_for.count(d_input)==0) {
                    pdg_in = new DyPDG_Input();
                    pdgnode_for[d_input]=pdg_in;
                    _dyPDG->addInput(pdg_in);
                  } else {
                    pdg_in = dynamic_cast<DyPDG_Input*>(pdgnode_for[d_input]);
                  }
                  pdg_in->setVPort(d_input->port());
              } 
              if (DyDIR::isInputDir(outDir)) {
                  //int n = outDir == DyDIR::OP0 ? 0:1;
                  d_output=dynamic_cast<dyoutput*>(_dyModel->subModel()->switchAt(posX,posY)
                    ->getOutLink(outDir)->dest());
                    
                  pdg_out = new DyPDG_Output();
                  pdgnode_for[d_output]=pdg_out;
                  _dyPDG->addOutput(pdg_out);
                  pdg_out->setVPort(d_output->port());
              }
              routeMap[_dyModel->subModel()->switchAt(posX,posY)][outDir]=inDir;
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
          pdg_inst = new DyPDG_Inst();
          pdgnode_for[_dyModel->subModel()->fuAt(posX,posY)]=pdg_inst;
          _dyPDG->addInst(pdg_inst);

          //get instruction        
          caps = getCaptureList(fuRE,line,strIter,true);
          if(caps.size()!=1) {
              cerr << "Bad FU\n";
              break;
          }
          
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
            DyDIR::DIR inDir =  DyDIR::toDir(caps[i],false);
            posMap[pdg_inst].push_back(inDir);
            if(inDir == DyDIR::IM) {
              pdg_inst->setImmSlot(i);
            }
            /*
              //cout << "caps[" << i << "]=" << caps[i] << "\n";
              
              if(inDir == DyDIR::END_DIR) {
                  //empty direction
                  //cerr << "Bad Direction\n";
                  
                  continue;
              } else if(inDir == DyDIR::IM) {
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
          
          if(strIter>=line.end()) {
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
  map<dynode*, map<DyDIR::DIR,DyDIR::DIR> >::iterator II,EE;
  for(II=routeMap.begin(),EE=routeMap.end();II!=EE;++II) {
    map<DyDIR::DIR,DyDIR::DIR>::iterator I,E;
    cout << "node: " << (*(II)).first->name() << "\n";
    for(I=(*(II)).second.begin(), E=(*(II)).second.end(); I!=E; ++I) {
      DyDIR::DIR newOutDir = I->first;
      DyDIR::DIR newInDir = I->second;
      cerr << DyDIR::dirName(newOutDir) << " : " << DyDIR::dirName(newInDir) << "\n";
    }
  }
*/


return;
  
}

void Schedule::reconstructSchedule(
    map<dynode*, map<DyDIR::DIR,DyDIR::DIR> >& routeMap, 
    map<dynode*, DyPDG_Node* >& pdgnode_for, 
    map<DyPDG_Node*, vector<DyDIR::DIR> >& posMap) {
  //iterate over inputs
  SubModel::const_input_iterator Iin,Ein;
  for(Iin=_dyModel->subModel()->input_begin(), 
      Ein=_dyModel->subModel()->input_end(); Iin!=Ein; ++Iin) {
    dyinput* dyinput_node = (dyinput*) &(*Iin);
    if(pdgnode_for.count(dyinput_node)!=0) {
        DyPDG_Node* pdg_node = pdgnode_for[dyinput_node];
        tracePath(dyinput_node,pdg_node,routeMap,pdgnode_for,posMap);
    }
  }

  //iterate over fus
  vector< vector<dyfu> >& fus = _dyModel->subModel()->fus();
  for(int i = 0; i < _dyModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _dyModel->subModel()->sizey(); ++j) {
      dyfu* dyfu_node = &fus[i][j];
      if(pdgnode_for.count(dyfu_node)!=0) {
        DyPDG_Node* pdg_node = pdgnode_for[dyfu_node];
        tracePath(dyfu_node,pdg_node,routeMap,pdgnode_for,posMap);
      }
        
    }
  }
}

/*
struct edgeLinkItem{ 
  dylink* dlink;
  int config;
  int DyPDG_Edge*
}
*/
void Schedule::calcAssignEdgeLink_single(DyPDG_Node* pdgnode) {
  //_assignEdgeLink

  list<pair<pair<dylink*,int>,DyPDG_Edge*>> openset;
  
  pair<dynode*,int> loc = locationOf(pdgnode);
  dynode* node = loc.first;
  int config = loc.second;
  
  if(!node) {
    cerr << "DyPDG_Node: " << pdgnode->name() << " is not scheduled\n"; 
    return;
  }
  
  DyPDG_Node::const_edge_iterator I,E;
  for(I=pdgnode->ops_begin(), E=pdgnode->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    DyPDG_Edge* source_pdgegde = (*I);
    DyPDG_Node* source_pdgnode = source_pdgegde->def();

    //route edge if source pdgnode is scheduled
    if(isScheduled(source_pdgnode)) {
      dynode::const_iterator Il,El;
      for(Il = node->ibegin(), El = node->iend(); Il!=El; ++Il) {
         dylink* link = *Il;
         if(pdgNodeOf(link,config)==source_pdgnode) {
           openset.push_back(make_pair(make_pair(link,config),source_pdgegde));
         }
      }
    }
  }
  /*

  
  dynode::const_iterator Il,El;
  for(Il = node->ibegin(), El = node->iend(); Il!=El; ++Il) {
      dylink* link = *Il;
      if(pdgNodeOf(link,config)!=NULL) {
        set<DyPDG_Edge*>& edgelist = _assignEdgeLink[make_pair(link,config)];
        set<DyPDG_Edge*>::iterator Ie,Ee;
        for(Ie=edgelist.begin(), Ee=edgelist.end(); Ie!=Ee; Ie++) {
          DyPDG_Edge* pdgedge = *Ie;
          openset.push_back(make_pair(make_pair(link,config),source_pdgedge));
        }
      }
  }*/
  
  while(!openset.empty()) {
    dylink* cur_link = openset.front().first.first;
    dynode* cur_node = cur_link->orig();
    int cur_config = openset.front().first.second; 
    DyPDG_Edge* cur_edge = openset.front().second;
    DyPDG_Node* cur_pdgnode = cur_edge->def();
    openset.pop_front();
    
    _assignEdgeLink[make_pair(cur_link,cur_config)].insert(cur_edge);
    
    dynode::const_iterator Il,El;
    for(Il = cur_node->ibegin(), El = cur_node->iend(); Il!=El; ++Il) {
        dylink* from_link = *Il;
        
        if(from_link->orig()==_dyModel->subModel()->cross_switch()) continue;
        if(from_link->orig()==_dyModel->subModel()->load_slice()  ) continue;
        
        if(pdgNodeOf(from_link,cur_config)==cur_pdgnode) {
          openset.push_back(make_pair(make_pair(from_link,cur_config),cur_edge));
        }
    }
   
  }
  
  

}

void Schedule::calcAssignEdgeLink() {
  _assignEdgeLink.clear();
  
  
  DyPDG::const_inst_iterator Ii,Ei;
  for(Ii=_dyPDG->inst_begin(), Ei=_dyPDG->inst_end(); Ii!=Ei; ++Ii) {
    DyPDG_Inst* pdginst = *Ii; 
    calcAssignEdgeLink_single(pdginst);
  }
  
  DyPDG::const_output_iterator Io,Eo;
  for(Io=_dyPDG->output_begin(),Eo=_dyPDG->output_end();Io!=Eo;++Io) {
    DyPDG_Output* pdgout = *Io;
    calcAssignEdgeLink_single(pdgout);
  }
  
  if(_dyModel->subModel()->multi_config()) {
    for(int config = 0; config < nConfigs();++config) {
      SubModel::const_input_iterator I,E;
      for(I=_dyModel->subModel()->input_begin(),
          E=_dyModel->subModel()->input_end(); I!=E; ++I) {
        dyinput* cand_input = const_cast<dyinput*>(&(*I));
        dylink* in_link = cand_input->getFirstInLink();
        dylink* out_link = cand_input->getFirstOutLink();  //links to loadslice
      
        if(DyPDG_Node* pdgnode = pdgNodeOf(out_link,config)) {
          assign_link(pdgnode,in_link,config);
          
          set<DyPDG_Edge*>::const_iterator Ie,Ee;
          set<DyPDG_Edge*>& edgelist= _assignEdgeLink[make_pair(out_link,config)];
          for(Ie=edgelist.begin(), Ee=edgelist.end(); Ie!=Ee; ++Ie) {
            DyPDG_Edge* pdgedge = *Ie;
            _assignEdgeLink[make_pair(in_link,config)].insert(pdgedge);
          }
        }
      }
    
      {SubModel::const_output_iterator I,E;
      for(I=_dyModel->subModel()->output_begin(),
          E=_dyModel->subModel()->output_end(); I!=E; ++I) {
        dyoutput* cand_output = const_cast<dyoutput*>(&(*I));
        dylink* in_link = cand_output->getFirstInLink();
        dylink* out_link = cand_output->getFirstOutLink();  //links to loadslice
      
        if(DyPDG_Node* pdgnode = pdgNodeOf(in_link,config)) {
          assign_link(pdgnode,out_link,config);
          
          set<DyPDG_Edge*>::const_iterator Ie,Ee;
          set<DyPDG_Edge*>& edgelist= _assignEdgeLink[make_pair(in_link,config)];
          for(Ie=edgelist.begin(), Ee=edgelist.end(); Ie!=Ee; ++Ie) {
            DyPDG_Edge* pdgedge = *Ie;
            _assignEdgeLink[make_pair(out_link,config)].insert(pdgedge);
          }
        }
      }}
    }
  }
  
  /*
  
  SubModel::const_input_iterator I,E;
  for(I=_dyModel->subModel()->input_begin(),
      E=_dyModel->subModel()->input_end(); I!=E; ++I) {
     dyinput* cand_input = const_cast<dyinput*>(&(*I));
    
    if(pdgNodeOf(cand_input,config)!=NULL) {
       dylink* firstOutLink = cand_input->getFirstOutLink();
       openset.push_back(firstOutLink);
       lat_edge[firstOutLink]=0;
    }
  }
    
    
  
  while(!openset.empty()) {
    dylink* inc_link = openset.front(); 
    openset.pop_front();
    
    dynode* node = inc_link->dest();
    dynode::const_iterator I,E,II,EE;
    
    DyPDG_Node* cur_pdgnode = pdgNodeOf(inc_link,config);
    assert(cur_pdgnode);
    
    if(dyfu* next_fu = dynamic_cast<dyfu*>(node)) {
      DyPDG_Node* next_pdgnode = pdgNodeOf(node,config);
      //cout << next_fu->name() << "\n"; 
      assert(next_pdgnode);
      DyPDG_Inst* next_pdginst = dynamic_cast<DyPDG_Inst*>(next_pdgnode); 
      assert(next_pdginst);
    
      bool everyone_is_here = true;
      int latency=0;
      for(II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
        dylink* inlink = *II;
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
        dylink* new_link = next_fu->getFirstOutLink();
        lat_edge[new_link] = latency + inst_lat(next_pdginst->inst());;
        openset.push_back(new_link);
      }
    } else if (dynamic_cast<dyoutput*>(node)) {
      if(lat_edge[inc_link] > max_lat) {
        max_lat = lat_edge[inc_link];
      }
    } else {
    
      for(I = node->obegin(), E = node->oend(); I!=E; ++I) {
        dylink* link = *I;
        
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



void Schedule::calcLatency(int &max_lat, int &max_lat_mis) {
  list<dylink*> openset;
  //map<dynode*,dylink*> came_from;
  map<dylink*,int> lat_edge;
  
  max_lat=0;  
  max_lat_mis=0;

  int config = 0;
  
  SubModel::const_input_iterator I,E;
  for(I=_dyModel->subModel()->input_begin(),
      E=_dyModel->subModel()->input_end(); I!=E; ++I) {
     dyinput* cand_input = const_cast<dyinput*>(&(*I));
    
    if(pdgNodeOf(cand_input,config)!=NULL) {
       dylink* firstOutLink = cand_input->getFirstOutLink();
       openset.push_back(firstOutLink);
       lat_edge[firstOutLink]=0;
    }
  }
    
    
  
  while(!openset.empty()) {
    dylink* inc_link = openset.front(); 
    openset.pop_front();
    
    dynode* node = inc_link->dest();
    dynode::const_iterator I,E,II,EE;
    
    DyPDG_Node* cur_pdgnode = pdgNodeOf(inc_link,config);
    assert(cur_pdgnode);
    
    if(dyfu* next_fu = dynamic_cast<dyfu*>(node)) {
      DyPDG_Node* next_pdgnode = pdgNodeOf(node,config);
      //cout << next_fu->name() << "\n"; 
      //
      if(!next_pdgnode) {
        //assert(next_pdgnode);
        cout << "problem with latency calculation!\n";
        max_lat=-1;
        max_lat_mis=-1;
        return;
      }

      DyPDG_Inst* next_pdginst = dynamic_cast<DyPDG_Inst*>(next_pdgnode); 
      assert(next_pdginst);
    
      bool everyone_is_here = true;

      int latency=0;
      int low_latency=100000000;  //magic number, forgive me
      for(II = next_fu->ibegin(), EE = next_fu->iend(); II!=EE; ++II) {
        dylink* inlink = *II;
        if(pdgNodeOf(inlink,config) != NULL) {
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
      

      if(everyone_is_here) {
        dylink* new_link = next_fu->getFirstOutLink();
        lat_edge[new_link] = latency + inst_lat(next_pdginst->inst());;
        openset.push_back(new_link);
        
        int diff = latency-low_latency;
        if(diff>max_lat_mis) {
          max_lat_mis=diff;
        }
      }
    } else if (dynamic_cast<dyoutput*>(node)) {
      if(lat_edge[inc_link] > max_lat) {
        max_lat = lat_edge[inc_link];
      }
    } else {
    
      for(I = node->obegin(), E = node->oend(); I!=E; ++I) {
        dylink* link = *I;
        
        if(pdgNodeOf(link,config) == pdgNodeOf(inc_link,config)) {
          lat_edge[link] = lat_edge[inc_link] + 1;
          openset.push_back(link);
        }
      }
    }
  }
}

void Schedule::tracePath(dynode* dyspot, DyPDG_Node* pdgnode, 
    map<dynode*, map<DyDIR::DIR,DyDIR::DIR> >& routeMap, 
    map<dynode*, DyPDG_Node* >& pdgnode_for, 
    map<DyPDG_Node*, vector<DyDIR::DIR> >& posMap) {
  
  //_assignNode[make_pair(dyspot,0)]=pdgnode;  //perform the assignment
  //_dynodeOf[pdgnode]=make_pair(dyspot,0);
  
  assign_node(pdgnode,dyspot,0);
  
  vector<pair<dynode*, DyDIR::DIR> > worklist;

  dylink* firstLink = dyspot->getFirstOutLink();
  assign_link(pdgnode,firstLink,0);
  
  dynode* startItem = firstLink->dest();
  DyDIR::DIR initialDir = firstLink->dir();
  worklist.push_back(make_pair(startItem,initialDir));

  //cerr << "---   tracing " << dyspot->name() << "   ---\n"; 
  
  while(!worklist.empty()) {
    
    //cerr << "worklist: ";
    //for(unsigned i = 0; i < worklist.size(); ++i) {
    //  dynode* item = worklist[i].first;
    //  DyDIR::DIR dir = worklist[i].second;
    //  cerr << DyDIR::dirName(dir) << ", " << item->name() << "";
    //  cerr << " | ";
    //}
    //cerr << "\n";
    
    dynode* curItem = worklist.back().first;
    DyDIR::DIR inDir = worklist.back().second;
    worklist.pop_back();
    
    map<DyDIR::DIR,DyDIR::DIR>::iterator I,E;
    for(I=routeMap[curItem].begin(), E=routeMap[curItem].end(); I!=E; ++I) {
      DyDIR::DIR newOutDir = I->first;
      DyDIR::DIR newInDir = I->second;
      
      if(inDir == newInDir) { //match!
       
        //dylink* inLink = curItem->getInLink(newInDir);
        
        //_assignLink[inLink]=pdgnode;
        
         
        dylink* outLink = curItem->getOutLink(newOutDir);
        assign_link(pdgnode,outLink,0);
        
        if(outLink==NULL) {
          //cerr << "outlink is null: ";
          //cerr << curItem->name() << " (dir:" << DyDIR::dirName(newOutDir) << "\n";
        }

        dynode* nextItem = outLink->dest();

        
        if(dynode* dyout = dynamic_cast<dyoutput*>(nextItem)) {
 //         cerr << DyDIR::dirName(newInDir) << " -> " << DyDIR::dirName(newOutDir) 
//               << "    (out)\n";

          DyPDG_Node* dest_pdgnode = pdgnode_for[dyout];
          assert(dest_pdgnode);
          
          _assignNode[make_pair(dyout,0)]=dest_pdgnode;  //perform the assignment
          _dynodeOf[dest_pdgnode]=make_pair(dyout,0);

          _dyPDG->connect(pdgnode,dest_pdgnode,0, DyPDG_Edge::data); 

        } else if(dyfu* fu_node = dynamic_cast<dyfu*>(nextItem)) {
//          cerr << DyDIR::dirName(newInDir) << " -> " << DyDIR::dirName(newOutDir) 
//               << "    (" << fu_node->x() << " " << fu_node->y() << " .. FU)" << "\n";

          DyPDG_Inst* dest_pdgnode = dynamic_cast<DyPDG_Inst*>(pdgnode_for[fu_node]);
          assert(dest_pdgnode);
          
          int slot=0;
          for(; slot < NUM_IN_FU_DIRS; ++slot) {
            if(posMap[dest_pdgnode][slot] == outLink->dir()) {
              break; 
            }
          }
          
          assert(slot>=0 && slot <NUM_IN_FU_DIRS);

          if(slot==2 && !dest_pdgnode->predInv()) {
            _dyPDG->connect(pdgnode,dest_pdgnode,slot, DyPDG_Edge::ctrl_true);
          } else if(slot==2 && dest_pdgnode->predInv()) {
            _dyPDG->connect(pdgnode,dest_pdgnode,slot, DyPDG_Edge::ctrl_false);
          } else {
            _dyPDG->connect(pdgnode,dest_pdgnode,slot, DyPDG_Edge::data); 
          }
        
        } else if(dyswitch* dysw = dynamic_cast<dyswitch*>(nextItem)){ //must be switch
//          cerr << DyDIR::dirName(newInDir) << " -> " << DyDIR::dirName(newOutDir) 
//               << "    (" << dysw->x() << " " << dysw->y() << ")" << "\n";
          dysw=dysw;
          worklist.push_back(make_pair(nextItem,newOutDir));
        } else {
          assert(0);
        }
      }
    }


  }
  
}

