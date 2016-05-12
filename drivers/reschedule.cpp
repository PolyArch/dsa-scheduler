#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"
#include <regex>

#include <unistd.h>


using namespace std;
using namespace SB_CONFIG;

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

void replaceLine(string& line, const char* name, regex& rx,map<int,int> portMapping,int lineno) 
{
  smatch capturedTexts;
  //match_flag_type flags = match_default | match_continuous; 
  //match_flag_type flags; = !match_continuous; 
  regex_constants::match_flag_type flags = regex_constants::match_default; 
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

void replaceLineOut(string& line, const char* name, regex& rx,map<int,int> portMapping,int lineno) 
{
  smatch capturedTexts;
  //match_flag_type flags = match_default | match_continuous; 
  //match_flag_type flags; = !match_continuous; 
  regex_constants::match_flag_type flags = regex_constants::match_default; 
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

void printHelp(){ 
  cerr <<  "Usage: resched [options] config_file source_file(s)...\n";
  cerr <<  "Options:\n";
  cerr <<  "m -- model file\n";
  cerr <<  "s -- scheduler algorithm: [mip,greedy]\n\n";
  
  cerr <<  "g -- gams relative optimality gap\n";
  cerr <<  "a -- gams absolute optimality gap\n";
  cerr <<  "t -- gams timeout\n\n";
  
  cerr <<  "h -- this help message\n\n";
}

int main(int argc, char* argv[])
{
  if(argc<2) {
    printHelp();
    exit(1);
  }

  enum sched_type {
    SCHED_MIP, SCHED_GREEDY
  } stype;
  
  stype = SCHED_MIP;
  
  char* model_file=NULL;
  
  float absolute_gap=999.0f;
  float relative_gap=0.1f;
  float timeout=300.0f;
  int c;
  bool multi_config=false;
  
  while ((c = getopt (argc, argv, "hxg:a:t:s:m:")) != -1) {
    switch (c) {
      case 'h':
        printHelp();
        return 1;
        break;
      case 'g':
        relative_gap=atof(optarg);
        break;
      case 'x':
        multi_config=true;
        break;
      case 'a':
        absolute_gap=atof(optarg);
        break;
      case 't':
        timeout=atof(optarg);
        break;
      case 's':
        if(strcmp(optarg,"greedy")==0) {
          stype = SCHED_GREEDY;
        }  else {
          stype = SCHED_MIP;
        }
        break;
      case 'm':
        model_file = optarg;
        if(!file_exists(model_file)) {
          cerr << "file: \"" << model_file << "\" does not exist\n";
          return 1;
        }
        break;
      case '?':
        if (optopt == 'c')
         fprintf (stderr, "Option -%c requires an argument.\n", optopt);
        else if (isprint (optopt))
         fprintf (stderr, "Unknown option `-%c'.\n", optopt);
        else
         fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
        return 1;
      default:
        abort ();
    }
  }
  
  system("mkdir -p gams/");
  system("mkdir -p dots/");

  //optind contiains configuration, [optind+1:argc) contains source files 
  
  Schedule sched(argv[optind]);
  
  ofstream ofs("dots/pdgout.dot", ios::out);
  assert(ofs.good());
  sched.sbpdg()->printGraphviz(ofs);
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
  

  SbModel* sbmodel;
  
  if(model_file) {
    cout << "Using file for sbmodel: " << model_file << "\n";
    sbmodel = new SbModel(model_file,multi_config);
  } else {
    cout << "Using sbmodel from schedule\n";
    sbmodel = sched.sbModel();
  }
  
  Scheduler scheduler(sbmodel);
  scheduler.setGap(relative_gap,absolute_gap);
  scheduler.setTimeout(timeout);
  
  Schedule* new_sched;              //scheduled object
  if(stype==SCHED_GREEDY) {
    //Greedy BFS
    new_sched = scheduler.scheduleGreedyBFS(sched.sbpdg());
  } else {

    //GAMS scheduler -- internally calls greedy bfs
    scheduler.scheduleGAMS(sched.sbpdg(),new_sched);
  }
  

  
  //ofstream resched(ss.str().c_str(), ios::out);
  //if(resched.fail()) {
  //  cerr << "could not open" << ss.str();
  //}
  
  //new_sched->printConfigText(resched);
  
  map<int,int> inPortMapping;
  map<int,int> outPortMapping;
  
  inPortMapping[0xff]=0xff;
  outPortMapping[0xff]=0xff;
  
  SbPDG::const_input_iterator I,E;

  //old and new mappings of input nodes
  for(I=new_sched->sbpdg()->input_begin(),E=new_sched->sbpdg()->input_end();I!=E;++I) {
     SbPDG_Input* in = *I;
     pair<int,int> oldp = sched.getConfigAndPort(in);
     pair<int,int> p = new_sched->getConfigAndPort(in);
     cout << in->name() << " old " << " " << oldp.first << "," << oldp.second;
     cout << "  new " << " " << p.first << "," << p.second << "\n";
     inPortMapping[oldp.second]=p.second;
  }
  
  SbPDG::const_output_iterator Io,Eo;

  //old and new mappings of output nodes
  for(Io=new_sched->sbpdg()->output_begin(),Eo=new_sched->sbpdg()->output_end();Io!=Eo;++Io) {
     SbPDG_Output* out = *Io;
     pair<int,int> oldp = sched.getConfigAndPort(out);
     pair<int,int> p = new_sched->getConfigAndPort(out);
     cout << out->name() << " old " << " " << oldp.first << "," << oldp.second;
     cout << "  new " << " " << p.first << "," << p.second << "\n";
     outPortMapping[oldp.second]=p.second;
    
  }
  
  for(int i = 0; i < sched.numWidePorts(); ++i) {
    vector<int>& widePort = sched.widePort(i);
    vector<int> newWidePort;
    
    bool all_inputs=true;
    bool all_outputs=true;
    
    for(unsigned j = 0; j < widePort.size(); ++j) {
      all_inputs &= (inPortMapping.count(widePort[j]) > 0);
      all_outputs &= (outPortMapping.count(widePort[j]) > 0);
      int ipm = inPortMapping.count(widePort[j]) > 0 ? inPortMapping[widePort[j]] : -1;
      int opm =outPortMapping.count(widePort[j]) > 0 ?outPortMapping[widePort[j]] : -1;
      cout << widePort[j] <<": "<< ipm <<", "<< opm <<", "<< all_inputs <<","<< all_outputs << "\n";
    }
    
    if(all_inputs & !all_outputs) {
      for(unsigned j = 0; j < widePort.size(); ++j) {
        newWidePort.push_back(inPortMapping[widePort[j]]);
        //cout << inPortMapping[widePort[j]] << "\n";
      }
    } else if(all_outputs & !all_inputs) {
      for(unsigned j = 0; j < widePort.size(); ++j) {
        newWidePort.push_back(outPortMapping[widePort[j]]);
      }
    } else if(all_outputs && all_inputs) {
        cerr << "vector port: " << i << " is indistinguishable b/t in and out\n";
    } else {
        cerr << "vector port: " << i << " isn't a fully utilized in or out port\n";
    }
    
    new_sched->addWidePort(newWidePort);
  }
 

  stringstream ss;
  ss << argv[optind];
  ss << ".resched";  
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
    for(unsigned j = 0; j < widePort.size(); ++j) {
      cout << widePort[j] << " ";
    }
    cout << "\n";
  }
  
  for(int which_argv=optind+1;which_argv<argc; ++which_argv) {
  
    int lineno=0;
    ifstream ifs(argv[which_argv], ios::in);
    assert(ifs.good());
    
    string outfile = string("resched.") + string(argv[which_argv]);
    cout << "outfile: " << outfile;
    ofstream ofs(outfile, ios::out);
    assert(ofs.good());
    
    regex rx_dyload("DyLOAD\\((.+),(\\d+)\\)");
    regex rx_dysend("DySEND\\((.+),(\\d+)\\)");
    regex rx_dysendf("DySENDF\\((.+),(\\d+)\\)");
    regex rx_dysendss("DySENDSS\\((.+),(\\d+)\\)");
    regex rx_dyrecv("DyRECV\\((\\d+),(.+)\\)");
    regex rx_dyrecvf("DyRECVF\\((\\d+),(.+)\\)");
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

