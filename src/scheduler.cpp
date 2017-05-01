#include "scheduler.h"

using namespace SB_CONFIG;
using namespace std;

#include <unordered_map>
#include <fstream>
#include <sstream>


//#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <list>

/*
class proposedPaths {
  public:
  //vector<pair<PDG_ID,
  //vector<pair<int,vector<int> > path;
  std::map<int,int>;
}*/
bool HeuristicScheduler::assignVectorInputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
	sbio_interface si =  subModel->io_interf();
	vector<pair<int,int>> sd;

  int n = sbPDG->num_vec_input();

	sbPDG->sort_vec_in();
	si.sort_in_vports(sd);

  for(int j = 0; j < n; ++j) {
    SbPDG_VecInput* vec_in = sbPDG->vec_in(j);
    //cout << "Assigning Vector Port:" << vec_in->gamsName() <<"\n";
    
    bool found_vector_port = false;

    //for(auto& i : subModel->io_interf().in_vports) {

		int curNum = progress_getCurNum(Input);

    for(auto& i : sd) {
			progress_updateCurNum(Input, curNum);
      int vport_num = i.first;
      auto vport_id = std::make_pair(true/*input*/,vport_num);
      vector<pair<int, vector<int>>>& vport_desc = si.getDesc_I(vport_num);

      //Check if the vetcor port is 1. big enough & 2. unassigned
      if(vec_in->num_inputs() <= vport_desc.size() && 
         sched->vportOf(vport_id) == NULL) {
        			std::vector<bool> mask; 
        			mask.resize(vport_desc.size());

        			bool ports_okay_to_use=true;

        			//Check if it's okay to assign to these ports
        			for(unsigned m=0; m < vec_in->num_inputs(); ++m) {
        			  //Get the sbnode corresponding to mask[m]
        			  int cgra_port_num = vport_desc[m].first;
        			  sbinput* cgra_in_port = subModel->get_input(cgra_port_num);

        			  if(sched->pdgNodeOf(cgra_in_port) != NULL) {
        			    ports_okay_to_use=false;
        			    break;
        			  } 
								progress_incCurNum(Input);
        			}
							progress_saveBestNum(Input);
        			if(!ports_okay_to_use) {
        			  //cout << "skipping this port assignment\n";
        			  continue; //don't assign these ports
        			}
        			// Assign Individual Elements
        			for(unsigned m=0; m < vec_in->num_inputs(); ++m) {
        			  mask[m]=true;
        			 
        			  //Get the sbnode corresponding to mask[m]
        			  int cgra_port_num = vport_desc[m].first;
        			  sbinput* cgra_in_port = subModel->get_input(cgra_port_num);

        			  //Get the input pdgnode corresponding to m
        			  SbPDG_Node* sbpdg_input = vec_in->getInput(m);
        			  sched->assign_node(sbpdg_input,cgra_in_port,0/*config*/);
        			}
        			//Perform the vector assignment
        			sched->assign_vport(vec_in,vport_id,mask);
        			found_vector_port=true;
      }
      if(found_vector_port) {
        break;
      }
    }
    if(!found_vector_port) {
			progress_saveBestNum(Input);	
      //cout << "Could not find Input hardware vector port\n";
      return false;
    }
		//progress_incCurNum(Input);
  }
	progress_saveBestNum(Input);	
  return true;
}


bool HeuristicScheduler::assignVectorOutputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  CandidateRouting candRouting;  
	sbio_interface si =  subModel->io_interf();
	vector<pair<int,int>> sd;

  int n = sbPDG->num_vec_output();
	

	sbPDG->sort_vec_out();
	si.sort_out_vports(sd);

  for(int j = 0; j < n; ++j) {
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(j);
    //cout << "Assigning Vector Port:" << vec_out->gamsName() <<"\n";
    
    bool found_vector_port = false;

		int curNum = progress_getCurNum(Output);

    for(auto& i : sd) {
			progress_updateCurNum(Output, curNum);
      int vport_num = i.first;
      auto vport_id = std::make_pair(true/*input*/,vport_num);
      vector<pair<int, vector<int>>>& vport_desc = si.getDesc_I(vport_num);

      //Check if the vetcor port is 1. big enough & 2. unassigned
      if(vec_out->num_outputs() <= vport_desc.size() && 
         sched->vportOf(vport_id) == NULL) {
        			std::vector<bool> mask; 
        			mask.resize(vport_desc.size());
        		
							bool ports_okay_to_use=true;
    					candRouting.clear();
    					//Check if it's okay to assign to these ports
    					for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
    					  //Get the sbnode corresponding to mask[m]
    					  int cgra_port_num = vport_desc[m].first;
    					  sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
    					  //Get the input pdgnode corresponding to m
    					  SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
    					  if(sched->pdgNodeOf(cgra_out_port) != NULL) {
    					    ports_okay_to_use=false;
    					    break;
    					  } 
								std::pair<int,int> fscore = make_pair(0, MAX_ROUTE); /*BUG: 0 should change to MAX_ROUTE for MLG, a better way to fix this is to define fscore for each subclass of HeursisticScheduler*/
    					  std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, 0, candRouting, fscore); 
    					  if(curScore>=fscore) { //?????
    					    ports_okay_to_use=false;
    					    break;
    					  }
								progress_incCurNum(Output);
    					}
							progress_saveBestNum(Output);
    					if(!ports_okay_to_use) {
    					  cout << "skipping this port assignment\n";
    					  continue; //don't assign these ports
    					}
							cout<<"Succeeded!"<<endl;
    					applyRouting(sched, 0/*config*/, &candRouting); //Commit the routing
    					// Assign Individual Elements
    					for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
    					  mask[m]=true;

    					  //Get the sbnode corresponding to mask[m]
    					  int cgra_port_num = vport_desc[m].first;
    					  sboutput* cgra_out_port = subModel->get_output(cgra_port_num);

    					  //Get the input pdgnode corresponding to m
    					  SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
    					  sched->assign_node(sbpdg_output,cgra_out_port,0/*config*/);
    					}
    					//Perform the vector assignment
    					sched->assign_vport(vec_out,vport_id,mask);
    					found_vector_port=true;
			}
			if (found_vector_port) {
				break;
			}
		}
    if(!found_vector_port) {

			progress_saveBestNum(Output);	
      //cout << "Could not find output hardware vector port\n";
      return false;
    }
		//progress_incCurNum(Output);
	}

	progress_saveBestNum(Output);	
  return true;
}

void HeuristicScheduler::applyRouting(Schedule* sched, int config,
                             CandidateRouting* candRouting) {

  std::map< std::pair<SB_CONFIG::sblink*,int>,SbPDG_Edge* >::iterator I,E;
  for(I= candRouting->routing.begin(), E=candRouting->routing.end();I!=E;++I) {
    sched->assign_link(I->second->def(),I->first.first, I->first.second);
		sched->updateLinkCount(I->first.first);
  }
  //TODO: Apply forwarding

}

void HeuristicScheduler::applyRouting(Schedule* sched, SbPDG_Node* pdgnode,
                             sbnode* here, int config, CandidateRouting* candRouting){
  
  //cout << "pdgnode: " << pdgnode->name()  << " sbnode: " << here->name() 
  //<< " nlinks: " << candRouting->routing.size() << "\n";  
  sched->assign_node(pdgnode,here,config);
  applyRouting(sched,config,candRouting);
}

void HeuristicScheduler::fillInputSpots(Schedule* sched,SbPDG_Input* pdginst,
                              int config, vector<sbnode*>& spots) {
  spots.clear();
  
  SubModel::const_input_iterator I,E;
  for(I=_sbModel->subModel()->input_begin(),
      E=_sbModel->subModel()->input_end(); I!=E; ++I) {
     sbinput* cand_input = const_cast<sbinput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_input,config)==NULL) {
       spots.push_back(cand_input);
    }
  }
}

void HeuristicScheduler::fillOutputSpots(Schedule* sched,SbPDG_Output* pdginst,
                              int config, vector<sbnode*>& spots) {
  spots.clear();
  
  SubModel::const_output_iterator I,E;
  for(I=_sbModel->subModel()->output_begin(),
      E=_sbModel->subModel()->output_end(); I!=E; ++I) {
     sboutput* cand_output = const_cast<sboutput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_output,config)==NULL) {
       spots.push_back(cand_output);
    }

  }
}

void HeuristicScheduler::fillInstSpots(Schedule* sched,SbPDG_Inst* pdginst,
                              int config, vector<sbnode*>& spots) {
  spots.clear();
  
  for(int i = 2; i < _sbModel->subModel()->sizex(); ++i) {
    sbfu* cand_fu = _sbModel->subModel()->fuAt(i,0);
    
    if((cand_fu->fu_def()==NULL || cand_fu->fu_def()->is_cap(pdginst->inst()))
       && sched->pdgNodeOf(cand_fu,config)==NULL) {
       spots.push_back(cand_fu);
    }
  }
  
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);
      
      if((cand_fu->fu_def()==NULL||cand_fu->fu_def()->is_cap(pdginst->inst()))
         && sched->pdgNodeOf(cand_fu,config)==NULL) {
         spots.push_back(cand_fu);
      }
    }
  }
}

pair<int,int> HeuristicScheduler::route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, int config, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  
	pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
  list<sbnode*> openset;
  map<sbnode*,int> node_dist;
  map<sbnode*,sblink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  SbPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      openset.push_back(I->first->dest());
    }
  }
  
  while(!openset.empty()) {
    sbnode* node = openset.front(); 
    openset.pop_front();
    
    //don't search this node if it's too much anyways
    //if(x->links+1>=cost_allotted) continue; 
    
    //check neighboring nodes
    sbnode::const_iterator I,E;
    for(I = node->obegin(), E = node->oend(); I!=E; ++I)
    {
      sblink* link = *I;
      
      //check if connection is closed..
      SbPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link,config);
      if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      pair<sblink*,int> p = make_pair(link,config);
      SbPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      sbnode* next = link->dest();
      
      if(next==_sbModel->subModel()->cross_switch()) continue;
      if(next==_sbModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      found_dest=(next==dest);

      if(dynamic_cast<sbfu*>(next) && !found_dest) continue;  //don't route through fu

      came_from[next] = link;
      node_dist[next] = node_dist[node]+1;

      if(found_dest) break; 
      
      openset.push_back(next);
    }
    if(found_dest) break;
  } 
	if(!found_dest) return fscore;  //routing failed, no routes exist!
  
  pair<int,int> score;
  score = make_pair(0,node_dist[dest]);
  
  sbnode* x = dest;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

//routes only inside a configuration
pair<int,int> HeuristicScheduler::route_minimizeOverlapping(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest,
                     int config, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  
	pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
  list<sbnode*> openset;
  map<sbnode*,int> node_dist;
  map<sbnode*,int> node_overlap;
  map<sbnode*,sblink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  SbPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      openset.push_back(I->first->dest());
    }
  }
  
  while(!openset.empty()) {
    sbnode* node = openset.front(); 
    openset.pop_front();
    
    //don't search this node if it's too much anyways
    //if(x->links+1>=cost_allotted) continue; 
    
    //check neighboring nodes
    sbnode::const_iterator I,E;
    for(I = node->obegin(), E = node->oend(); I!=E; ++I)
    {
      sblink* link = *I;
      
      //check if connection is closed..
      SbPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link,config);
      //if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      std::pair<sblink*,int> p = make_pair(link,config);
      SbPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      //if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      sbnode* next = link->dest();
      
   //   if(next==_sbModel->subModel()->cross_switch()) continue;
    //  if(next==_sbModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      found_dest=(next==dest);

      if(dynamic_cast<sbfu*>(next) && !found_dest) continue;  //don't route through fu

      came_from[next] = link;
      node_dist[next] = node_dist[node]+1;
      if ((sched_exist_pdg != NULL && sched_exist_pdg != pdgnode) 
				|| (cand_exist_pdg != NULL && cand_exist_pdg != pdgnode)) {
				node_overlap[next]=node_overlap[node]+1;
		  }
 
      if(found_dest) break; 
      
      openset.push_back(next);
    }
    if(found_dest) break;
  }
    
  if(!found_dest) return fscore;  //routing failed, no routes exist!
  
  int distScore;
  distScore = node_dist[dest];

	int overlapScore;
	overlapScore = node_overlap[dest];
 
	std::pair<int,int> score = make_pair(overlapScore,distScore);
	 
  sbnode* x = dest;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

//routes only inside a configuration
int HeuristicScheduler::route_to_output(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source,
                     int config, CandidateRouting& candRouting, int scoreLeft) {
  
  list<sbnode*> openset;
  map<sbnode*,int> node_dist;
  map<sbnode*,sblink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  SbPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      openset.push_back(I->first->dest());
    }
  }
  
  sboutput* the_output = NULL;
      
  while(!openset.empty()) {
    sbnode* node = openset.front(); 
    openset.pop_front();
    
    //don't search this node if it's too much anyways
    //if(x->links+1>=cost_allotted) continue; 
    
    //check neighboring nodes
    sbnode::const_iterator I,E;
    for(I = node->obegin(), E = node->oend(); I!=E; ++I)
    {
      sblink* link = *I;
      
      //check if connection is closed..
      SbPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link,config);
      if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      pair<sblink*,int> p = make_pair(link,config);
      SbPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      
      sbnode* next = link->dest();
      
      if(next==_sbModel->subModel()->cross_switch()) continue;
      if(next==_sbModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      the_output = dynamic_cast<sboutput*>(next);
      found_dest=(the_output!=NULL);

      if(dynamic_cast<sbfu*>(next) && !found_dest) continue;  //don't route through fu

      came_from[next] = link;
      node_dist[next] = node_dist[node]+1;

      if(found_dest) break; 
      
      openset.push_back(next);
    }
    if(found_dest) break;
  }
    
  
  if(!found_dest) return MAX_ROUTE;  //routing failed, no routes exist!
  
  int score;
  score = node_dist[the_output];
  
  sbnode* x = the_output;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

bool Scheduler::check_res(SbPDG* sbPDG, SbModel* sbmodel) {
  int ninsts = sbPDG->inst_end() - sbPDG->inst_begin();

  int nfus = sbmodel->subModel()->sizex() * sbmodel->subModel()->sizey();

  if(ninsts > nfus) {
    cerr << "\n\nError: Too many instructions in SbPDG for given SBCONIG\n\n";
    exit(1);
  }

  bool failed_count_check=false;

  std::map<sb_inst_t,int> count_types;
  for(auto Ii=sbPDG->inst_begin(), Ei=sbPDG->inst_end(); Ii!=Ei; ++Ii) {
    count_types[(*Ii)->inst()]++;
  }

  for(auto& pair : count_types) {
    sb_inst_t sb_inst = pair.first;
    int pdg_count = pair.second;

    int fu_count =0;
    for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
      for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
        sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);
        if(cand_fu->fu_def()->is_cap(sb_inst)) {
          fu_count++;
        }
      }
    }
    if(fu_count < pdg_count) {
      failed_count_check=true;
      cerr << "Error: PDG has " << pdg_count << " " << name_of_inst(sb_inst) 
           << " insts, but only " << fu_count << " fus to support them\n";
    }
  }

  if(failed_count_check) {
    cerr << "\n\nError: FAILED Basic FU Count Check\n\n";
    exit(1);
  }
  //TODO: add code from printPortcompatibility here
  
  return true;
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


/*
bool Scheduler::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {
  
  pair<int,int> bestScore = make_pair(MAX_ROUTE,MAX_ROUTE); //a big number
	pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
  CandidateRouting* bestRouting = new CandidateRouting();
  sbnode* bestspot;
  int bestconfig;
  
  CandidateRouting* curRouting = new CandidateRouting();
  
  std::vector<sbnode*> spots;
  
  //for each configuration
  for(int config = 0; config < sched->nConfigs(); ++config) {
    if(SbPDG_Inst* pdginst= dynamic_cast<SbPDG_Inst*>(pdgnode))  { 
      fillInstSpots(sched, pdginst, config, spots);             //all possible candidates based on FU capability 
    } else if(SbPDG_Input* pdg_in = dynamic_cast<SbPDG_Input*>(pdgnode)) {
      fillInputSpots(sched,pdg_in,config,spots); 
    } else if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>(pdgnode)) {
      fillOutputSpots(sched,pdg_out,config,spots); 
    }
   
    //populate a scheduling score for each of canidate sbspot
    for(unsigned i=0; i < spots.size(); i++) {
      sbnode* cand_spot = spots[i];
      
      curRouting->routing.clear();
      curRouting->forwarding.clear();
      
      pair<int,int> curScore = scheduleHere(sched, pdgnode, cand_spot, config,*curRouting,bestScore);
                  
      if(curScore < bestScore) {
        bestScore=curScore;
        bestspot=cand_spot;
        bestconfig=config;
        std::swap(bestRouting,curRouting);
      }
      
      if(bestScore <= make_pair(0,1))  { //????
        applyRouting(sched,pdgnode,bestspot,bestconfig,bestRouting);
        return true;
      }//apply routing step
    
    }//for loop -- check for all sbnode spots
  }
  
  
  //TODO: If not scheduled, then increase the numConfigs, and try again
  
  if (bestScore < fscore) {
    applyRouting(sched,pdgnode,bestspot,bestconfig,bestRouting);
  } else {
    cout << "WARNING!!!! No route found for pdgnode: " << pdgnode->name() << "\n";
    return false; 
  }
  return true;
}

pair<int,int> Scheduler::scheduleHere(Schedule* sched, SbPDG_Node* n, 
                                sbnode* here, int config, 
                                CandidateRouting& candRouting,
                                pair<int,int> bestScore) {
  pair<int,int> score=make_pair(0,0);
	pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
  
  SbPDG_Node::const_edge_iterator I,E;
  
  for(I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    SbPDG_Edge* source_pdgegde = (*I);
    SbPDG_Node* source_pdgnode = source_pdgegde->def();     //could be input node also

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(source_pdgnode)) {
      pair<sbnode*,int> source_loc = sched->locationOf(source_pdgnode); //scheduled location
     
      //route using source node, sbnode
      pair<int,int> tempScore = route(sched, source_pdgegde, source_loc.first, here,config,candRouting,bestScore-score);
			score = score + tempScore;
      //cout << n->name() << " " << here->name() << " " << score << "\n";
      if(score>bestScore) return fscore;
    }
  }
  
  SbPDG_Node::const_edge_iterator Iu,Eu;
  for(Iu=n->uses_begin(), Eu=n->uses_end();Iu!=Eu;++Iu) {
    SbPDG_Edge* use_pdgedge = (*Iu); 
    SbPDG_Node* use_pdgnode = use_pdgedge->use();
     
     //riir<sbnode*,int> use_loc = sched->locationOf(use_pdgnode);
     //       pair<int,int> tempScore = route(sched, use_pdgedge, here, use_loc.first,config,candRouting,bestScore-score);   |
     //              score = score + tempScore;                                                                                     |       pair<int,int> tempScore = route(sched, use_pdgedge, here, use_loc.first,config,candRouting,bestScore-score);
     //                     //cout << n->name() << " " << here->name() << " " << score << "\n";                                            |       score = score + tempScore;
     //                            if(score>bestScore) return score;                                                                  oute edge if source pdgnode is scheduled
     if(sched->isScheduled(use_pdgnode)) {
       pair<sbnode*,int> use_loc = sched->locationOf(use_pdgnode);
       
       pair<int,int> tempScore = route(sched, use_pdgedge, here, use_loc.first,config,candRouting,bestScore-score);
			 score = score + tempScore;
       //cout << n->name() << " " << here->name() << " " << score << "\n";
       if(score>bestScore) return score;
     }
  }
  
  return score;
}

pair<int,int> Scheduler::route(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, int config, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
	pair<int,int> score = route_minimizeDistance(sched, pdgedge, source, dest, config, candRouting, scoreLeft);
	pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
	if (score == fscore) {
		score = route_minimizeOverlapping(sched, pdgedge, source, dest, config, candRouting, scoreLeft);
	}
	return score;
}
*/
//routes only inside a configuration
//return value <numOverlappedLinks, Distance>
/*bool Scheduler::schedule(SbPDG* sbPDG, Schedule*& sched) {

  sched = new Schedule(_sbModel,sbPDG);
  sched->setNConfigs(1);
 
	bool vec_in_assigned = assignVectorInputs(sbPDG,sched);
  if(!vec_in_assigned) {
    return false;
  }

  map<SbPDG_Inst*,bool> seen;
	bool schedule_okay=true;
  
	list<SbPDG_Inst* > openset;
  SbPDG::const_input_iterator I,E;
  
	//pdg input nodes
  for(I=sbPDG->input_begin(),E=sbPDG->input_end();I!=E;++I) {
    SbPDG_Input* n = *I;
    
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }
 
  //populate the schedule object
  while(!openset.empty()) {
    SbPDG_Inst* n = openset.front(); 
    openset.pop_front();
    
    if(!seen[n]) {
			schedule_okay &= scheduleNode(sched,n);
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
  
  bool vec_out_assigned = assignVectorOutputs(sbPDG,sched);
  if(!vec_out_assigned) {
    return false;
  }
  return schedule_okay;
}
*/

