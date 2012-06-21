#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"
#include <boost/regex.hpp>

using namespace std;
using namespace DY_MODEL;

#include<sys/stat.h>

int file_exists (char * fileName)
{
   struct stat buf;
   int i = stat ( fileName, &buf );
     /* File found */
     if ( i == 0 )
     {
       return 1;
     }
     return 0;
       
}

void replaceLine(string& line, char* name, boost::regex& rx,map<int,int> portMapping,int lineno) 
{
  boost::smatch capturedTexts;
  //boost::match_flag_type flags = boost::match_default | boost::match_continuous; 
  //boost::match_flag_type flags; = !boost::match_continuous; 
  boost::match_flag_type flags = boost::match_default; 
  if(regex_search(line, capturedTexts, rx,flags)) {
    //cout << "found match: " << line << "\n";
    int oldPort = atoi(capturedTexts[2].str().c_str());
    //cout << "oldport:" << oldPort << "\n";
    
    if(portMapping.count(oldPort)==0) {
      cout << "Can't Update Port (line " << lineno << "):\"" << line << "\"\n";
    } else {
      int newPort = portMapping[oldPort];
      //cout << "newport:" << newPort << "\n";
      stringstream ss;
      ss << name << "(" << capturedTexts[1] << "," << newPort << ")";
      line = regex_replace(line, rx, ss.str(),flags);
      //cout << "new: " << line << "\n";
    }
  }
}

void replaceLineOut(string& line, char* name, boost::regex& rx,map<int,int> portMapping,int lineno) 
{
  boost::smatch capturedTexts;
  //boost::match_flag_type flags = boost::match_default | boost::match_continuous; 
  //boost::match_flag_type flags; = !boost::match_continuous; 
  boost::match_flag_type flags = boost::match_default; 
  if(regex_search(line, capturedTexts, rx,flags)) {
    //cout << "found match: " << line << "\n";
    int oldPort = atoi(capturedTexts[1].str().c_str());
    //cout << "oldport:" << oldPort << "\n";
    
    if(portMapping.count(oldPort)==0) {
      cout << "Can't Update Port (line " << lineno << "):\"" << line << "\"\n";
    } else {
      int newPort = portMapping[oldPort];
      //cout << "newport:" << newPort << "\n";
      stringstream ss;
      ss << name << "(" << newPort << "," << capturedTexts[2] << ")";
      line = regex_replace(line, rx, ss.str(),flags);
      //cout << "new: " << line << "\n";
    }
  }
}

int main(int argc, char* argv[])
{
  
  if(argc<2) {
    cerr <<  "Usage: resched config_file [something.dymodel]\n";
    exit(1);
  }
  
  
  Schedule sched(argv[1]);
  
  ofstream ofs("pdgout.dot", ios::out);
  if(ofs.fail()) {
    cerr << "could not open pdgout.dot";
  }
  sched.dypdg()->printGraphviz(ofs);
  ofs.close();
  
  cout << "Ports for Original Schedule:\n";
  for(int i = 0; i < sched.numWidePorts(); ++i) {
    cout << i << ": ";
    vector<int>& widePort = sched.widePort(i);
    for(unsigned j = 0; j < widePort.size(); ++j) {
      cout << widePort[j] << " ";
    }
    cout << "\n";
  }
  
  //Perform scheduling:
  Schedule* new_sched;  //this is where schedule will be returned
  
  //DyModel dymodel("/u/t/j/tjn/dyser/src/dymodel/test/dyser8x8.dy_model");
  //DyModel dymodel("/u/t/j/tjn/dyser/src/dymodel/models/dyser8x8_hetero.dymodel");
  char* filename;
  if(argc==2) {
    filename = "/u/t/j/tjn/dyser/src/dymodel/models/dyser4x4.dymodel";
  } else {
    filename = argv[2];    
  }
  
  //filename = "/u/t/j/tjn/dyser/src/dymodel/models/dyser8x8_hetero.dymodel";
  
  
  DyModel* dymodel;
  
  if(file_exists(filename)) {
    cout << "Using file for dymodel: " << filename << "\n";
    dymodel = new DyModel(filename);
  } else {
    cout << "Using dymodel from schedule\n";
    dymodel = sched.dyModel();
  }
  
  Scheduler scheduler(dymodel);
  scheduler.setGap(0.1f,999.0f);
  scheduler.setTimeout(300.0f);
  
  if(argc>2 && strcmp(argv[2],"greedy")==0) {
    new_sched = scheduler.scheduleGreedyBFS(sched.dypdg());
  } else {
    scheduler.scheduleGAMS(sched.dypdg(),new_sched);
  }
  
  
  
  stringstream ss;
  ss << argv[1];
  ss << ".resched";  
  
  //ofstream resched(ss.str().c_str(), ios::out);
  //if(resched.fail()) {
  //  cerr << "could not open" << ss.str();
  //}
  
  //new_sched->printConfigText(resched);
  
  map<int,int> inPortMapping;
  map<int,int> outPortMapping;
  
  inPortMapping[0xff]=0xff;
  outPortMapping[0xff]=0xff;
  
  DyPDG::const_input_iterator I,E;
  for(I=new_sched->dypdg()->input_begin(),E=new_sched->dypdg()->input_end();I!=E;++I) {
     DyPDG_Input* in = *I;
     pair<int,int> oldp = sched.getConfigAndPort(in);
     pair<int,int> p = new_sched->getConfigAndPort(in);
     cout << in->name() << " old " << " " << oldp.first << "," << oldp.second;
     cout << "  new " << " " << p.first << "," << p.second << "\n";
     inPortMapping[oldp.second]=p.second;
  }
  
  {
  DyPDG::const_output_iterator I,E;
  for(I=new_sched->dypdg()->output_begin(),E=new_sched->dypdg()->output_end();I!=E;++I) {
     DyPDG_Output* out = *I;
     pair<int,int> oldp = sched.getConfigAndPort(out);
     pair<int,int> p = new_sched->getConfigAndPort(out);
     cout << out->name() << " old " << " " << oldp.first << "," << oldp.second;
     cout << "  new " << " " << p.first << "," << p.second << "\n";
     outPortMapping[oldp.second]=p.second;
    
  }
  }
  
  for(int i = 0; i < sched.numWidePorts(); ++i) {
    vector<int>& widePort = sched.widePort(i);
    vector<int> newWidePort;
    
    bool all_inputs=true;
    bool all_outputs=true;
    
    
    for(int j = 0; j < widePort.size(); ++j) {
      all_inputs &= (inPortMapping.count(widePort[j]) > 0);
      all_outputs &= (outPortMapping.count(widePort[j]) > 0);
      int ipm = inPortMapping.count(widePort[j]) > 0 ? inPortMapping[widePort[j]] : -1;
      int opm =outPortMapping.count(widePort[j]) > 0 ?outPortMapping[widePort[j]] : -1;
      cout << widePort[j] <<": "<< ipm <<", "<< opm <<", "<< all_inputs <<","<< all_outputs << "\n";
    }
    
    if(all_inputs & !all_outputs) {
      for(int j = 0; j < widePort.size(); ++j) {
        newWidePort.push_back(inPortMapping[widePort[j]]);
        //cout << inPortMapping[widePort[j]] << "\n";
      }
    } else if(all_outputs & !all_inputs) {
      for(int j = 0; j < widePort.size(); ++j) {
        newWidePort.push_back(outPortMapping[widePort[j]]);
      }
    } else if(all_outputs && all_inputs) {
        cerr << "vector port: " << i << " is indistinguishable b/t in and out\n";
    } else {
        cerr << "vector port: " << i << " isn't a fully utilized in or out port\n";
    }
    
    

    
    
    new_sched->addWidePort(newWidePort);
  }
  
  new_sched->printAllConfigs(ss.str().c_str());
  
  Schedule::forward_iterator If,Ef;
  for(If=new_sched->fbegin(),Ef=new_sched->fend();If!=Ef;++If) {
    cout << "Forward: "
        << "(" << If->first.first << ", " << If->first.second << ") -> "
        << "(" << If->second.first << ", " << If->second.second << ")\n";
  }
  
  cout << "Vector Ports for New Schedule:\n";
  for(int i = 0; i < new_sched->numWidePorts(); ++i) {
    cout << i << ": ";
    vector<int>& widePort = new_sched->widePort(i);
    for(int j = 0; j < widePort.size(); ++j) {
      cout << widePort[j] << " ";
    }
    cout << "\n";
  }
  
  for(int which_argv=3;which_argv<argc; ++which_argv) {
  
    int lineno=0;
    ifstream ifs(argv[which_argv], ios::in);
    assert(ifs.good());
    
    string outfile = string("resched.") + string(argv[which_argv]);
    cout << "outfile: " << outfile;
    ofstream ofs(outfile, ios::out);
    assert(ofs.good());
    
    boost::regex rx_dyload("DyLOAD\\((.+),(\\d+)\\)");
    boost::regex rx_dysend("DySEND\\((.+),(\\d+)\\)");
    boost::regex rx_dysendf("DySENDF\\((.+),(\\d+)\\)");
    boost::regex rx_dysendss("DySENDSS\\((.+),(\\d+)\\)");
    boost::regex rx_dyrecv("DyRECV\\((\\d+),(.+)\\)");
    boost::regex rx_dyrecvf("DyRECVF\\((\\d+),(.+)\\)");
    while(ifs.good())
    {
        string line;
        getline(ifs,line);
        lineno++;
        replaceLine(line,"DyLOAD",rx_dyload,inPortMapping,lineno);
        replaceLine(line,"DySEND",rx_dysend,inPortMapping,lineno);
        replaceLine(line,"DySENDF",rx_dysendf,inPortMapping,lineno);
        replaceLine(line,"DySENDSS",rx_dysendss,inPortMapping,lineno);
        replaceLineOut(line,"DyRECV",rx_dyrecv,outPortMapping,lineno);
        replaceLineOut(line,"DyRECVF",rx_dyrecvf,outPortMapping,lineno);
        
        //cout << line << "\n";
        ofs << line << "\n";
    }
    
    
  }
  
}

