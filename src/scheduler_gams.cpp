#include "scheduler_gams.h"

using namespace SB_CONFIG;
using namespace std;

#include <unordered_map>
#include <fstream>
#include <list>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>


#include "model_parsing.h"

#include "gams_models/softbrain_gams.h"
#include "gams_models/softbrain_gams_hw.h"
#include "gams_models/spill_model.h"
#include "gams_models/multi_model.h"
#include "gams_models/single_fixed_general.h"
#include "gams_models/timing_model.h"
#include "gams_models/hw_model.h"
#include "gams_models/stage_model.h"



//MIP START IS DEFUNCT NOW THAT THE SCHEDULER CAN'T KEEP UP WITH REQs OF PROBLEM
#define USE_MIP_START 0 

bool GamsScheduler::schedule(SbPDG* sbPDG,Schedule*& schedule) {
  string hw_model          = string((const char*)gams_models_hw_model_gms);
  string timing_model      = string((const char*)gams_models_timing_model_gms);
  char* transfer_model = (char*) malloc(gams_models_stage_model_gms_len+1);
  memcpy((void*) transfer_model, (void*) gams_models_stage_model_gms, gams_models_stage_model_gms_len);
  transfer_model[gams_models_stage_model_gms_len] = '\0';
  string stage_model       = string((char*) transfer_model);
  stage_model = stage_model.substr(0, stage_model.length()-2);
  cout << stage_model << endl;
  string softbrain_gams    = string((const char*)gams_models_softbrain_gams_gms);
  string softbrain_gams_hw = string((const char*)gams_models_softbrain_gams_hw_gms);

  //mkfifo("/tmp/gams_fifo",S_IRWXU);
  stringstream ss;
  ss << _gams_work_dir << "/softbrain.out";
  string gams_out_file = ss.str();
  
  ss.str(std::string());
  ss << "softbrain.gams";
  string gams_file_name = ss.str();
  
  system(("rm -f " + gams_out_file).c_str());
  
  #if USE_MIP_START 
  schedule = scheduleGreedyBFS(sbPDG); // Get the scheduled pdg object
  schedule->calcAssignEdgeLink();
  #else
  schedule = new Schedule(_sbModel,sbPDG);
  #endif

 
  //bool use_hw=true;
  bool use_hw=false;

  // ----------------- setup the sbmodel gams files --------------------------
  if(!_gams_files_setup) {
        
    // Print the Constraints
    ofstream ofs_constraints(_gams_work_dir+"/constraints.gams", ios::out);
    assert(ofs_constraints.good());
    //ofs_constraints << multi_model;
    if(use_hw) {
      ofs_constraints << hw_model;
    } else {
      //ofs_constraints << timing_model;
      ofs_constraints << stage_model;
    }

    ofs_constraints.close();
    free(transfer_model);
     // Print the kinds of instructions
    ofstream ofs_kinds(_gams_work_dir+"/softbrain_kind.gams", ios::out);
    assert(ofs_kinds.good());
    _sbModel->printGamsKinds(ofs_kinds);
    ofs_kinds.close();
  
    _gams_files_setup=true;
  }
  
  // Print the controlling file
  ofstream ofs_sb_gams(_gams_work_dir+"/"+gams_file_name, ios::out);
  assert(ofs_sb_gams.good());
  
  ofs_sb_gams << "option reslim=" << _reslim << ";\n"
                 << "option optcr="  <<  _optcr << ";\n"   
                 << "option optca="  <<  _optca << ";\n";
  if(use_hw) {
    ofs_sb_gams << softbrain_gams_hw;
  } else {
    ofs_sb_gams << softbrain_gams;
  }
  ofs_sb_gams.close();
  
  ofstream ofs_mipstart(_gams_work_dir+"/mip_start.gams", ios::out);
  assert(ofs_mipstart.good());

  #if USE_MIP_START
  int config=0;
  //print mipstart
  Schedule::assign_node_iterator I,E;
  for(I = schedule->assign_node_begin(), E= schedule->assign_node_end();I!=E;++I) {
    sbnode* spot = I->first;
    SbPDG_Node* pdgnode = I->second;
    ofs_mipstart << "Mn.l('" << pdgnode->gamsName() 
                 << "','" << spot->gams_name(config) << "')=1;\n";
  }
 

  Schedule::assign_link_iterator Il,El;
  for(Il = schedule->assign_link_begin(), 
      El= schedule->assign_link_end();Il!=El;++Il) {
    sblink* link = Il->first;
    SbPDG_Node* pdgnode = Il->second;
    ofs_mipstart << "Mvl.l('" << pdgnode->gamsName() 
                 << "','" << link->gams_name(config) << "')=1;\n";
  }
  
  Schedule::assign_edgelink_iterator Ile,Ele;
  for(Ile = schedule->assign_edgelink_begin(), 
      Ele= schedule->assign_edgelink_end();Ile!=Ele;++Ile) {
    
    sblink* link = Ile->first;
    set<SbPDG_Edge*>& edgelist= Ile->second;
    
    set<SbPDG_Edge*>::const_iterator Ie,Ee;
    for(Ie=edgelist.begin(), Ee=edgelist.end(); Ie!=Ee; ++Ie) {
      SbPDG_Edge* pdgedge = *Ie;
      
      ofs_mipstart << "Mel.l('" << pdgedge->gamsName() << "','" 
                 << link->gams_name(config) << "')=1;\n";
    }
  }
  
  ofs_mipstart << "Tv.l(v)=0;\n";
  
  {
   map<SbPDG_Node*,bool> seen;
  
  list<SbPDG_Node* > openset;
  SbPDG::const_input_iterator I,E;
  for(I=sbPDG->input_begin(),E=sbPDG->input_end();I!=E;++I) {
    SbPDG_Input* n = *I;
    //openset.push_back(n);
    ofs_mipstart << "Tv.l('" << n->gamsName() << "')=0;\n";
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }
  
  while(!openset.empty()) {
    SbPDG_Node* n = openset.front(); 
    openset.pop_front();

    if(!seen[n]) {
      ofs_mipstart << "Tv.l('" << n->gamsName() << "')="
                   << "smax((v1,e)$(Gve(v1,e) and "
                   << "Gev(e,'" << n->gamsName() << "')),Tv.l(v1)"
                   << "+ sum(l,Mel.l(e,l)) + delta(e));\n";

    }
    seen[n]=true;
    
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }

  }

  //ofs_mipstart << "Tv.l(v2) = smax((v1,e)$(Gve(v1,e) and Gev(e,v2)),Tv.l(v1) + sum(l,Ml.l(e,l))) + delta.l(e);\n";
  ofs_mipstart << "length.l=smax(v,Tv.l(v));\n";
  ofs_mipstart << "cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));\n";
  ofs_mipstart << "display Tv.l;\n";
  ofs_mipstart << "display length.l;\n";
  ofs_mipstart << "display cost.l;\n";
  ofs_mipstart.close();
#endif

  schedule->clearAll();

  cout << "Total Nodes: " << sbPDG->num_nodes() << "\n";
  
  int numInsts = sbPDG->inst_end()-sbPDG->inst_begin();
  cout << "Total Insts: " <<  numInsts << "\n";
  //assert(numInsts > 0);

  // Print the softbrain model   
  ofstream ofs_sb_model(_gams_work_dir + "/softbrain_model.gams", ios::out);
  assert(ofs_sb_model.good());
  gamsToSbnode.clear(); gamsToSblink.clear();
  _sbModel->subModel()->PrintGamsModel(ofs_sb_model,gamsToSbnode,gamsToSblink,
                                       gamsToSbswitch,gamsToPortN,0/*nconfigs*/);

  cout << gamsToSbnode.size() << " " << gamsToSblink.size() << " " << gamsToSbswitch.size() << "\n";

  ofs_sb_model.close();
  
  // ----------------- setup the pdg gams files ------------------------------
  ofstream ofs_sb_pdg(_gams_work_dir + "/softbrain_pdg.gams", ios::out);
  if(ofs_sb_pdg.fail()) {
    cerr << "could not open " + _gams_work_dir + "/softbrain_pdg.gams";
    return false;
  }
  gamsToPdgnode.clear(); 
  gamsToPdgedge.clear();
  gamsToPortV.clear();

  //Also populates these maps
  //--gamsToPdgnode
  //--gamsToPdgegde
  //--gamsToPortV
  sbPDG->printGams(ofs_sb_pdg,gamsToPdgnode,gamsToPdgedge,gamsToPortV);
  sbPDG->printPortCompatibilityWith(ofs_sb_pdg,_sbModel);
  ofs_sb_pdg.close();
  
  // ----------------- run gams! --------------------------------------------
  if (_use_server) {
    char buf[1024];
    std::string fullname = std::string(getcwd(buf, 1023));
    fullname += std::string("/") + gams_file_name;
    requestGams(fullname.c_str());
  } else {
    stringstream ss_cmd;
    ss_cmd << "gams " << gams_file_name << " wdir=" << _gams_work_dir;
    if(_showGams) {
      ss_cmd << " -lo=3";
    } else {
       ss_cmd << " -o=/dev/null -lo=2"; 
    }
    cout << ss_cmd.str().c_str() << "\n";
    system(ss_cmd.str().c_str());
  }

  // ----------------- parse output -----------------------------------------


  string line, edge_name, vertex_name, switch_name, link_name, out_link_name, sbnode_name,list_of_links,latency_str;
  ifstream gamsout(gams_out_file.c_str());
  enum {VtoN,VtoL,LtoL,EL,EDGE_DELAY,TIMING,PortMap,PASSTHROUGH,Parse_None}parse_stage;
  parse_stage=Parse_None;
  bool message_start=false, message_fus_ok=false, message_ports_ok=false;

  while(gamsout.good()) {  
    getline(gamsout,line);
    ModelParsing::trim_comments(line);
    ModelParsing::trim(line);

    if(line.empty()) {
      continue;
    }
    //if(ModelParsing::StartsWith(line,"#")) continue;
    if(line[0]=='[') {
      parse_stage = Parse_None; 
      if(ModelParsing::StartsWith(line,"[vertex-node-map]")) {
        parse_stage = VtoN; continue;
      } else if(ModelParsing::StartsWith(line,"[vertex-link-map]")) {
        parse_stage = VtoL; continue;
      } else if(ModelParsing::StartsWith(line,"[switch-map]")) {
        parse_stage = LtoL; continue;
      } else if(ModelParsing::StartsWith(line,"[extra-lat]")) {
        parse_stage = EL; continue;
      } else if(ModelParsing::StartsWith(line,"[edge-delay]")) {
        parse_stage = EDGE_DELAY; continue;
      } else if(ModelParsing::StartsWith(line,"[timing]")) {
        parse_stage = TIMING; continue;
      } else if(ModelParsing::StartsWith(line,"[passthrough]")) {
        parse_stage = PASSTHROUGH; continue;
      } else if(ModelParsing::StartsWith(line,"[port-port-map]")) {
        parse_stage = PortMap; continue;
      } else if(ModelParsing::StartsWith(line,"[status_message_begin_scheduling]")) {
        message_start=true; continue;
      } else if(ModelParsing::StartsWith(line,"[status_message_fus_ok]")) {
        message_fus_ok=true; continue;
      } else if(ModelParsing::StartsWith(line,"[status_message_ports_ok]")) {
        message_ports_ok=true; continue;
      } 
    }

    if(parse_stage==PortMap) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      ss >> std::ws;
      getline(ss, sbnode_name, ' ');

      //cout << "PORT MAP: " << vertex_name << " " <<sbnode_name << "\n";

      ModelParsing::trim(vertex_name);
      ModelParsing::trim(sbnode_name);

      if(sbnode_name.empty()) {
        cout << "failed to parse line: \"" << line << "\"\n";
        assert(0);
      }

      SbPDG_Vec* pv = gamsToPortV[vertex_name];
      assert(pv);
      std::pair<bool,int> pn = gamsToPortN[sbnode_name];  

      unsigned size_of_vp;
      if(pn.first) {
       size_of_vp = _sbModel->subModel()->io_interf().in_vports[pn.second].size();
      } else {
       size_of_vp = _sbModel->subModel()->io_interf().out_vports[pn.second].size();
      }

      std::vector<bool> mask;
      //mask.resize(pv->locMap().size());
      mask.resize(size_of_vp);
      
      while(ss.good()) {
        string ind_str;
        getline(ss, ind_str, ' ');
        ModelParsing::trim(ind_str);
        if(ind_str.empty()) continue;
        unsigned ind = (int)(stof(ind_str))-1;
        
        //cout << vertex_name << " " << sbnode_name << " " << ind << " " << size_of_vp << "\n";
        assert(ind < size_of_vp && "went off end of vec");
        assert(mask[ind]==false && "I already assigned this place in the vec!");

        mask[ind]=true;
      }
      //cout << "\n";

      schedule->assign_vport(pv,pn,mask);

    } else if(parse_stage==PASSTHROUGH) {
      stringstream ss(line);
      
      while(ss.good()) {
        getline(ss,sbnode_name, ' ');
        ModelParsing::trim(sbnode_name);
        if(sbnode_name.empty()) continue;
        
        sbnode* sbnode  = gamsToSbnode[sbnode_name].first;  
        if(sbnode==NULL) {
          cerr << "null sbnode:\"" << sbnode_name << "\"\n";
        }
        schedule->add_passthrough_node(sbnode);

      }

    } else if(parse_stage==TIMING) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      getline(ss, latency_str, '.');
      ModelParsing::trim(vertex_name);
      ModelParsing::trim(latency_str);
      SbPDG_Node* pdgnode = gamsToPdgnode[vertex_name];

      int lat = stoi(latency_str);
      schedule->assign_lat(pdgnode,lat);

    } else if(parse_stage==EL) {
      stringstream ss(line);
      getline(ss, edge_name, ':');
      getline(ss, sbnode_name);
      //TODO: FINISH THIS IF EVER NEED EDGE -> LINK MAPPING

    } else if(parse_stage==EDGE_DELAY) {
      stringstream ss(line);
      getline(ss, edge_name, ':');

      ModelParsing::trim(edge_name);
      SbPDG_Edge* pdgedge = gamsToPdgedge[edge_name];
      assert(pdgedge);

      string delay_str;
      getline(ss, delay_str);
      ModelParsing::trim(delay_str);
      if(delay_str.empty()) continue;
      unsigned delay = (unsigned)(stof(delay_str));

      pdgedge->set_delay(delay);

    } else if(parse_stage==VtoN) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      getline(ss, sbnode_name);
      ModelParsing::trim(vertex_name);
      ModelParsing::trim(sbnode_name);
      
      if(sbnode_name.empty()) {
        return false;
      }

      SbPDG_Node* pdgnode = gamsToPdgnode[vertex_name];
      sbnode* sbnode  = gamsToSbnode[sbnode_name].first;  
      
      if(vertex_name.empty()) continue;
      
     
      schedule->assign_node(pdgnode,sbnode);
      
        /*if(sboutput* sbout = dynamic_cast<sboutput*>(sbnode) ) {
           cout << pdgnode->name() << " new=" << schedule->getPortFor(pdgnode) << "\n";
        }*/
        
      //schedule
    } else if (parse_stage==LtoL) { //PARSE SWITCH MAP --------------------------
      stringstream ss(line);
      getline(ss, switch_name, ':');
      ModelParsing::trim(switch_name);
      if(switch_name.empty()) continue;

      sbswitch* sbsw = gamsToSbswitch[switch_name].first;
      if(sbsw==NULL) {
        cerr << "null sbsw:\"" << switch_name << "\"\n";
      }

      while(ss.good()) {
        getline(ss, link_name, ' ');
        getline(ss, out_link_name, ',');

        ModelParsing::trim(link_name);
        ModelParsing::trim(out_link_name);

        if(link_name.empty()) continue;
        if(out_link_name.empty()) continue;
        sblink* slink = gamsToSblink[link_name].first;
        sblink* slink_out = gamsToSblink[out_link_name].first;
        assert(slink);
        assert(slink_out);
        schedule->assign_switch(sbsw,slink,slink_out);

      }
      
    } else if (parse_stage==VtoL) {
//      if(_assignSwitch.size()!=0) {
//        continue;
//      }

      stringstream ss(line);
      getline(ss, vertex_name, ':');
      //getline(ss, list_of_links);
      
      ModelParsing::trim(vertex_name);
      if(vertex_name.empty()) continue;
      
      SbPDG_Node* pdgnode = gamsToPdgnode[vertex_name];
      if(pdgnode==NULL) {
        cerr << "null pdgnode:\"" << vertex_name << "\"\n";
      }
      
      while(ss.good()) {
        getline(ss, link_name, ' ');
        ModelParsing::trim(link_name);
        if(link_name.empty()) continue;
        sblink* slink = gamsToSblink[link_name].first;
        
        if(slink==NULL) {
          cerr << "null slink:\"" << link_name << "\"\n";
        }
        
        schedule->assign_link(pdgnode,slink);
        
        if(sbinput* sbin = dynamic_cast<sbinput*>(slink->orig())) {
          schedule->assign_node(pdgnode,sbin); 
        } else if(sboutput* sbout = dynamic_cast<sboutput*>(slink->dest())) {
          //find output for this output edge
          SbPDG_Node::const_edge_iterator I,E;
          for(I=pdgnode->uses_begin(), E=pdgnode->uses_end();I!=E;++I) {
            if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>((*I)->use())) {
              schedule->assign_node(pdg_out,sbout);
            }
          }
        }
        
      }
    }
    
  }
 
  if(!message_start) {
    cerr << "\n\nError: Scheduling Not Started -- Likely Error in Gams Code Gen\n\n";
    exit(1);
  } else if (!message_fus_ok) {
    cerr << "\n\nError: Combination of FUs requested are NOT satisfiable with given SBCONFIG.\n\n";
    exit(1);
  } else if (!message_ports_ok) {
    cerr << "\n\nError: Port specifications are NOT satisfiable with given SBCONFIG.\n\n";
    exit(1);
  }

  if(_showGams) {
    //Print the I/Os
    std::cout << "in/out mapping:";

    SbPDG::const_input_iterator Ii,Ei;
    for(Ii=schedule->sbpdg()->input_begin(),Ei=schedule->sbpdg()->input_end();Ii!=Ei;++Ii) {
      SbPDG_Input* in = *Ii;
      int p = schedule->getPortFor(in);
      cout << in->name() << " " << p << ", ";
    }

    SbPDG::const_output_iterator Io,Eo;
    for(Io=schedule->sbpdg()->output_begin(),Eo=schedule->sbpdg()->output_end();Io!=Eo;++Io) {
      SbPDG_Output* out = *Io;
      int p = schedule->getPortFor(out);
      cout << out->name() << " " << p << ", ";
    }
    std::cout << "\n";
  }
  
  
  return true;
}


void error(const char *msg)
{
  perror(msg);
  exit(0);
}



bool GamsScheduler::requestGams(const char *filename)
{
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;

  char cmd_buf[256];
  char buffer[1024];

  portno = 20202;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  server = gethostbyname("arcturus.cs.wisc.edu");
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host\n");
    exit(0);
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    error("ERROR connecting");

  strcpy(cmd_buf, "run-gams");
  n = write(sockfd, cmd_buf, strlen(cmd_buf));
  if (n < 0)
    error("ERROR writing to socket");

  n = write(sockfd, filename,strlen(filename));
  if (n < 0)
    error("ERROR writing to socket");
  bzero(buffer,256);
  n = read(sockfd,buffer,255);
  if (n < 0)
    error("ERROR reading from socket");
  if (strcmp(buffer, "__DONE__") == 0)
    return true;
  error("Error running gams");
  return false;
}

