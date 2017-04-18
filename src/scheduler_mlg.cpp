#include "scheduler_mlg.h"

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


bool SchedulerMultipleLinkGreedy::schedule(SbPDG* sbPDG, Schedule*& sched) {

  sched = new Schedule(getSBModel(),sbPDG);
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

bool SchedulerMultipleLinkGreedy::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {

  std::pair<int,int> bestScore = make_pair(MAX_ROUTE,MAX_ROUTE);
	std::pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
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
      
      if(bestScore<=make_pair(0,1))  {
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



pair<int,int> SchedulerMultipleLinkGreedy::scheduleHere(Schedule* sched, SbPDG_Node* n, 
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

     //route edge if source pdgnode is scheduled
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

pair<int,int> SchedulerMultipleLinkGreedy::route(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, int config, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
	pair<int,int> score = route_minimizeDistance(sched, pdgedge, source, dest, config, candRouting, scoreLeft);
	pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
	if (score == fscore) {
		score = route_minimizeOverlapping(sched, pdgedge, source, dest, config, candRouting, scoreLeft);
	}
	return score;
}
//routes only inside a configuration
//return value <numOverlappedLinks, Distance>
/*pair<int,int> Scheduler::route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, int config, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  
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
pair<int,int> Scheduler::route_minimizeOverlapping(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest,
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
*/
