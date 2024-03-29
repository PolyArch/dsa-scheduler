%code requires {

#include <stdint.h>
#include <string.h>
#include <string>
#include "dsa/debug.h"
#include "dsa/dfg/node.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/symbols.h"
#include "dsa/dfg/node.h"
int parse_dfg(const char* dfgfile, SSDfg* dfg);
typedef std::pair<std::string,int> io_pair_t;
typedef std::pair<std::string,std::string> map_pair_t;
typedef std::pair<int,int> map_id_t;

using ParseResult = dsa::dfg::ParseResult;
using ControlEntry = dsa::dfg::ControlEntry;
using ConvergeEntry = dsa::dfg::ConvergeEntry;
using ConstDataEntry = dsa::dfg::ConstDataEntry;
using ValueEntry = dsa::dfg::ValueEntry;
using NodeEntry = dsa::dfg::NodeEntry;
using EdgeType = dsa::dfg::OperandType;

struct IODef {
  std::string name;
  int length;
  bool isTagged;
  std::string source;
  std::string destination;
  std::string penetrate;
  IODef(const std::string &n, int l, bool t, const std::string &s, const std::string &d, const std::string &p) :
    isTagged(t), name(n), length(l), source(s), destination(d), penetrate(p) {}
};

struct ArrayDef {
  std::string name;
  int size;
  std::string type;

  ArrayDef(const std::string &n, int s, const std::string &t) :
    name(n), size(s), type(t) {}
};

}

%{
#include "dfg-parser.tab.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <map>
#include <tuple>

extern int yylineno;
void yyrestart(FILE *); 

struct parse_param {
  SSDfg* dfg;
  dsa::dfg::MetaPort meta;
  std::map<std::string, std::string> regs;
  dsa::dfg::SymbolTable symbols;
};

int yylex();     
static void yyerror(parse_param*, const char *); 
%}

%parse-param {struct parse_param* p}

%union {
  uint64_t i;
  double d;

  std::string* s;
  IODef* io;
  ArrayDef* array;
  map_pair_t* map_pair;
  std::vector<ParseResult*> *sym_vec;
  dsa::dfg::ParseResult *sym_ent;
  std::vector<std::string>* str_vec;
  ctrl_def_t* ctrl_def;
  std::vector<std::pair<dsa::OpCode, int>> *fu_and_cnt;
  task_def_t* task_def;
  std::vector<map_pair_t*> *map_vec;
  map_id_t* map_id;

  YYSTYPE() {}   // this is only okay because sym_ent doesn't need a
  ~YYSTYPE() {}  // real constructor/deconstructuor (but string/vector do)
}

%token	  EOLN NEW_DFG PRAGMA ARROW
%token<s> IDENT STRING_LITERAL
%token<d> F_CONST
%token<i> I_CONST INPUT OUTPUT INDIRECT TASKDEP TASKPROP ARRAY

%type <io> io_def
%type <array> array_def
%type <map_vec> map_def
%type <map_id> task_map_id_def
%type <sym_vec> arg_list
%type <sym_ent> arg_expr rhs expr edge edge_list
%type <ctrl_def> ctrl_list
%type <fu_and_cnt> fu_list
%type <str_vec> ident_list value_list // @vidushi: what does it mean to have two?
%type <task_def> task_map_list

%start statement_list
%debug
%error-verbose

%%

statement_list: statement
	            | statement_list statement;

statement: INPUT ':' io_def  eol {
  DSA_CHECK($3->penetrate.empty()) << "Input port cannot have penetration.";
  auto name = $3->name;
  DSA_CHECK($3->destination.empty()) << "Input port cannot have destination.";
  DSA_CHECK(!p->symbols.Has(name)) << "Already has the symbol " << name;
  int len = $3->length;
  bool stated = $3->isTagged;
  int width = $1;
  int n = std::max(1, len);
  p->dfg->emplace_back<dsa::dfg::InputPort>(n, width, name, p->dfg, p->meta, stated);
  /*
   * If the a port is stated, the identifier of the port should be ${PortName}State.
   * It will be come a standalone port with this identifier in the DFG data structure representation.
   */
  if (stated) {
    auto symbol_id = "$" + name + "State";
    p->symbols.Set(symbol_id, new ValueEntry(p->dfg->vins.back().id(), 0, 0, 7));
  }
  for (int i = 0, cnt = 0, vid = stated; i < n; ++i) {
    std::stringstream ss;
    ss << name;
    if (len) ss << cnt++;
    // TODO(@were): Do I need to modularize these two clean up segment?

    DSA_LOG(PARSE) << "Input port " << ss.str() << " with stated " << stated <<" is created with vid " << vid << " and width " << width;

    p->symbols.Set(ss.str(), new ValueEntry(p->dfg->vins.back().id(), vid, 0, width - 1));

    vid++;
  }
  auto &in = p->dfg->vins.back();

  // Create an edge from source to this input port
  if (!$3->source.empty()) {
    auto pve = p->symbols.Get($3->source);
    DSA_CHECK(pve) << "Source port " << $3->source << " not found.";
    if (auto ve = dynamic_cast<dsa::dfg::ValueEntry*>(pve)) {
      if (auto reg = dynamic_cast<dsa::dfg::Register*>(p->dfg->nodes[ve->vid])) {
        DSA_CHECK(false) << "Register (" << $3->source << ") cannot be source of input port.";
      }
      // Create an edge from Array to this array
      p->dfg->edges.emplace_back(p->dfg, ve->nid, ve->vid, in.id(), 0, ve->l, ve->r);

      // Add Operand to this Input Port
      std::vector<int> es{p->dfg->edges.back().id};
      p->dfg->vins.back().ops().emplace_back(p->dfg, es, EdgeType::data);
      
      // Add Value User to the Array
      //p->dfg->nodes[ve->nid]->values[0].uses.emplace_back(p->dfg->edges.back().id);
    }
  }

  p->meta.clear();
  delete $3;
}
| OUTPUT ':' io_def eol {
  DSA_CHECK($3->source.empty()) << "Output port cannot have source.";
  DSA_CHECK(!$3->isTagged) << "An output cannot be stated!";
  using T = dsa::dfg::OutputPort;
  auto name = $3->name;
  int len = $3->length;
  int width = $1;
  int n = std::max(1, len);
  int penetrate = -1;
  dsa::dfg::ValueEntry *pve = nullptr;
  if (!$3->penetrate.empty()) {
    std::string state = "$" + $3->penetrate + "State";
    auto *entry = p->symbols.Get(state);
    pve = dynamic_cast<dsa::dfg::ValueEntry *>(entry);
    DSA_CHECK(pve);
    penetrate = pve->nid;
  }
  p->dfg->emplace_back<T>(n, width, name, p->dfg, p->meta, penetrate);
  if (penetrate != -1) {
    p->dfg->edges.emplace_back(
      p->dfg, pve->nid, pve->vid, p->dfg->vouts.back().id(), 0, pve->l, pve->r);
    std::vector<int> es{p->dfg->edges.back().id};
    p->dfg->vouts.back().ops().emplace_back(p->dfg, es, EdgeType::data);
  }
  auto &out = p->dfg->vouts.back();
  int left_len = 0;
  for (int i = 0, cnt = 0, oid = (penetrate != -1); i < n; ++i) {
    std::stringstream ss;
    ss << name;
    // at output, we make the connectivity because the previous edge would already be there..
    if (len) ss << cnt++;
    // TODO(@were): Do I need to modularize these two clean up segment?
    auto sym = p->symbols.Get(ss.str()); // check in a symbols array?
    if (auto ce = dynamic_cast<dsa::dfg::ConvergeEntry*>(sym)) {
      int num_entries = ce->entries.size();
      DSA_CHECK(num_entries > 0 && num_entries <= 16);
      std::vector<int> es;
      for (auto elem : ce->entries) {

         DSA_LOG(PARSE) << "Output port " << ss.str() << " with stated " << (penetrate != -1) << ", oid " << oid << ", and width " << elem->r;

        p->dfg->edges.emplace_back(p->dfg, elem->nid, elem->vid, out.id(), oid, elem->l, elem->r, elem->l, elem->r);
        es.push_back(p->dfg->edges.back().id);
        oid++;
      }
      out.ops().emplace_back(p->dfg, es, EdgeType::data);
    } else if (auto ve = dynamic_cast<dsa::dfg::ValueEntry*>(sym)) {

      DSA_LOG(PARSE) << "Output port " << ss.str() << " with stated " << (penetrate != -1) << ", oid " << oid << ", and width " << ve->r;

      p->dfg->edges.emplace_back(p->dfg, ve->nid, ve->vid, out.id(), oid, ve->l, ve->r, ve->l, ve->r);
      

      std::vector<int> es{p->dfg->edges.back().id};
      out.ops().emplace_back(p->dfg, es, EdgeType::data);

      oid++;
    }
  }
  out.values.emplace_back(p->dfg, out.id(), 0);

  // Add an edge from this output port to the destination
  if (!$3->destination.empty()) {
    auto pve = p->symbols.Get($3->destination);
    DSA_CHECK(pve) << "Dest port " << $3->destination << " not found.";
    if (auto ve = dynamic_cast<dsa::dfg::ValueEntry*>(pve)) {
      
      // Add Edge from Output Port to Array
      p->dfg->edges.emplace_back(p->dfg, out.id(), ve->vid, ve->nid, 0, ve->l, ve->r);
      
      // Add Values to the OutputPort
      std::vector<int> es{p->dfg->edges.back().id};
      //out.values[0].uses.push_back(p->dfg->edges.back().id);
      
      // Add operand to the Array
      p->dfg->nodes[ve->nid]->ops().emplace_back(p->dfg, es, EdgeType::data);
    }
  }
  p->meta.clear();
  delete $3;
}
| ARRAY ':' array_def eol {
  auto name = $3->name;
  DSA_CHECK(!p->symbols.Has(name)) << "Already has the symbol " << name;
  int size = $3->size;
  if (strcmp($3->type.c_str(), "dma") == 0) {
    using T = dsa::dfg::DMA;
    p->dfg->emplace_back<T>(size, name, p->dfg);
    p->dfg->nodes.back()->values.emplace_back(p->dfg, p->dfg->nodes.back()->id(), 0);
    p->symbols.Set(name, new ValueEntry(p->dfg->nodes.back()->id(), 0));
  } else if (strcmp($3->type.c_str(), "spm") == 0 || strcmp($3->type.c_str(), "scratchpad") == 0) {
    using T = dsa::dfg::Scratchpad;
    p->dfg->emplace_back<T>(size, name, p->dfg);
    p->dfg->nodes.back()->values.emplace_back(p->dfg, p->dfg->nodes.back()->id(), 0);
    p->symbols.Set(name, new ValueEntry(p->dfg->nodes.back()->id(), 0));
  } else if (strcmp($3->type.c_str(), "reg") == 0) {
    using T = dsa::dfg::Register;
    p->dfg->emplace_back<T>(size, name, p->dfg);
    p->dfg->nodes.back()->values.emplace_back(p->dfg, p->dfg->nodes.back()->id(), 0);
    p->symbols.Set(name, new ValueEntry(p->dfg->nodes.back()->id(), 0));
  } else if (strcmp($3->type.c_str(), "gen") == 0) {
    using T = dsa::dfg::Generate;
    p->dfg->emplace_back<T>(size, name, p->dfg);
    p->dfg->nodes.back()->values.emplace_back(p->dfg, p->dfg->nodes.back()->id(), 0);
    p->symbols.Set(name, new ValueEntry(p->dfg->nodes.back()->id(), 0));
  } else if (strcmp($3->type.c_str(), "rec") == 0) {
    using T = dsa::dfg::Recurrance;
    p->dfg->emplace_back<T>(size, name, p->dfg);
    p->dfg->nodes.back()->values.emplace_back(p->dfg, p->dfg->nodes.back()->id(), 0);
    p->symbols.Set(name, new ValueEntry(p->dfg->nodes.back()->id(), 0));
  } else {
    DSA_CHECK(false) << "Unsupported array type: " << $3->type;
  }

  p->meta.clear();
  delete $3;
}
| INDIRECT ':' io_def eol { // create 1 input, 1 output and edge
  auto name = $3->name;
  int len = $3->length;
  int width = $1;
  int n = std::max(1, len);
  int slice = 64 / width;
  
  // add input/output ports
  p->dfg->emplace_back<dsa::dfg::InputPort>(n, width, name, p->dfg, p->meta, false);
  p->dfg->emplace_back<dsa::dfg::OutputPort>(n, width, name, p->dfg, p->meta, -1);
  //, true);
  
  // std::stringstream ss;
  // ss << name;
  // auto sym = p->symbols.Get(ss.str());
  // auto ne = dynamic_cast<dsa::dfg::ValueEntry*>(sym);
  
  int nid = p->dfg->vins.back().id();
  p->symbols.Set(std::string(name), new ValueEntry(nid, 0));
  int vid = 0;
  int oid = 0;
  int iid = p->dfg->vouts.back().id();
  int l=0, r=63;

  p->dfg->edges.emplace_back(
    p->dfg, nid, vid,
    iid, oid, l, r);

  std::vector<int> es{p->dfg->edges.back().id};

  // set ops on this output
  p->dfg->vouts.back().ops().emplace_back(p->dfg, es, EdgeType::data);
  // p->dfg->vins.back().values[0].uses.push_back(p->dfg->edges.back().id);
  // DSA_LOG(PARSE) << p->dfg->vouts.back().name() << " " << p->dfg->vouts.back().id();

  p->meta.clear();
  delete $3;
}
| TASKDEP '[' task_map_id_def map_def ':' '(' task_map_list ')' eol {
  p->dfg->create_new_task_dependence_map($3->first, $3->second); // accessing a pair
  for(unsigned task_param=0; task_param<$4->size(); ++task_param) {
    p->dfg->add_new_task_dependence_characteristic((*$4)[task_param]->first, (*$4)[task_param]->second); // accessing a pair
  }
  auto &args = *$7;
  for(unsigned i=0; i<args.size(); ++i) {
    p->dfg->add_new_task_dependence_map(args[i].first, args[i].second);
  }
  p->meta.clear();
  delete $7;
}
| TASKPROP '[' I_CONST ']' ':' '(' map_def ')' eol { // TASKPROP[0]: (coreMask:0111)
  p->dfg->create_new_task_type($3); // accessing a pair
  for(unsigned task_param=0; task_param<$7->size(); ++task_param) {
    p->dfg->add_new_task_property((*$7)[task_param]->first, (*$7)[task_param]->second); // accessing a pair
  }
  p->meta.clear();
  delete $7;
}
| value_list '=' rhs eol {
  if (auto ne = dynamic_cast<NodeEntry*>($3)) {
    auto node = p->dfg->nodes[ne->nid];
    if (auto op = dynamic_cast<dsa::dfg::Operation*>(node)) {
      for (int i = 0, n = $1->size(); i < n; ++i) {
        op->values.emplace_back(p->dfg, op->id(), i);
      }
    }
    for (int i = 0, n = node->values.size(); i < n; ++i) {
      auto *ve = new ValueEntry(node->id(), i, 0, node->bitwidth() - 1);
      p->symbols.Set((*$1)[i], ve);
      node->values[i].symbol = (*$1)[i];
      auto iter = p->regs.find((*$1)[i]);
      if (iter != p->regs.end()) {
        p->regs.erase(iter);
        DSA_CHECK(sscanf(iter->second.c_str() + 4, "%d", &node->values[i].reg) == 1);
      }
    }
  } else if (dynamic_cast<ConvergeEntry*>($3) || dynamic_cast<ValueEntry*>($3)) {
    // Converge all the space-separated values
    DSA_CHECK($1->size() == 1);
    p->symbols.Set((*$1)[0], $3);
  } else {
    DSA_CHECK(false) << "Unknown type of entry!";
  }
  delete $1;
}
| NEW_DFG eol {
  p->dfg->meta.emplace_back();
}
| PRAGMA IDENT IDENT eol {
  if ($2->find("$Reg") == 0) {
    p->regs[*$3] = *$2;
  } else {
    p->dfg->set_pragma(*$2,*$3);
  }
  delete $2;
  delete $3;
}
| PRAGMA IDENT IDENT I_CONST eol {
  DSA_CHECK(*$2 == "group");
  std::ostringstream oss;
  oss << $4;
  p->dfg->set_pragma(*$3, oss.str());
  delete($2);
  delete($3);
}
| PRAGMA IDENT '=' IDENT eol {
  p->meta.set(*$2, *$4);
}
| PRAGMA IDENT '=' I_CONST eol {
  std::ostringstream oss;
  oss << $4;
  p->meta.set(*$2, oss.str());
}
| PRAGMA IDENT '=' F_CONST eol {
  std::ostringstream oss;
  oss << $4;
  p->meta.set(*$2, oss.str());
}
| eol
;

eol : ';'
	| EOLN
	;

// Array Definition
// Array: NAME SIZE TYPE
array_def: IDENT {
  $$ = new ArrayDef(*$1, 64, "dma");
  delete $1;
}
| IDENT I_CONST {
  $$ = new ArrayDef(*$1, $2, "dma");
  delete $1;
}
| IDENT I_CONST IDENT {
  $$ = new ArrayDef(*$1, $2, *$3);
  delete $1;
  delete $3;
}
| IDENT IDENT {
  $$ = new ArrayDef(*$1, 64, *$2);
  delete $1;
  delete $2;
};


// Input: A[8]
io_def: IDENT {
  $$ = new IODef(*$1, 0, false, "", "", "");
  delete $1;
}
| IDENT '[' I_CONST ']' {
  $$ = new IODef(*$1, $3, false, "", "", "");
  delete $1;
}
| IDENT IDENT {
  DSA_CHECK(*$2 == "stated") << "'stated' expected";
  $$ = new IODef(*$1, 0, true, "", "", "");
  delete $1;
}
| IDENT '[' I_CONST ']' IDENT {
  DSA_CHECK(*$5 == "stated") << "'stated' expected";
  $$ = new IODef(*$1, $3, true, "", "", "");
  delete $1;
}
| IDENT IDENT '=' IDENT {
  if (*$2 == "state") {
    $$ = new IODef(*$1, 0, false, "", "", *$4);
  } else if (*$2 == "source") {
    $$ = new IODef(*$1, 0, false, *$4, "", "");
  } else if (*$2 == "destination") {
    $$ = new IODef(*$1, 0, false, "", *$4, "");
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$2 << "! Expected one of [stated, source, destination]";
  }
  delete $1;
  delete $4;
}
| IDENT '[' I_CONST ']' IDENT '=' IDENT {
  if (*$5 == "stated") {
    $$ = new IODef(*$1, $3, false, "", "", *$7);
  } else if (*$5 == "source") {
    $$ = new IODef(*$1, $3, false, *$7, "", "");
  } else if (*$5 == "destination") {
    $$ = new IODef(*$1, $3, false, "", *$7, "");
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$5 << "! Expected one of [stated, source, destination]";
  }
  delete $1;
  delete $5;
  delete $7;
}
| IDENT '[' I_CONST ']' IDENT '=' IDENT IDENT {
  DSA_CHECK(*$8 == "stated") << "'stated' expected";
  if (*$5 == "source") {
    $$ = new IODef(*$1, $3, true, *$7, "", "");
  } else if (*$5 == "destination") {
    $$ = new IODef(*$1, $3, true, "", *$7, "");
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$5 << "! Expected one of [ source, destination]";
  }
  delete $1;
  delete $5;
  delete $7;
  delete $8;
}
| IDENT IDENT '=' IDENT IDENT {
  DSA_CHECK(*$5 == "stated") << "'stated' expected";
  if (*$2 == "source") {
    $$ = new IODef(*$1, 0, true, *$4, "", "");
  } else if (*$2 == "destination") {
    $$ = new IODef(*$1, 0, true, "", *$4, "");
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$2 << "! Expected one of [source, destination]";
  }
  delete $1;
  delete $2;
  delete $4;
  delete $5;
}
| IDENT '[' I_CONST ']' IDENT IDENT '=' IDENT {
  DSA_CHECK(*$5 == "stated") << "'stated' expected";
  if (*$6 == "source") {
    $$ = new IODef(*$1, $3, true, *$8, "", "");
  } else if (*$6 == "destination") {
    $$ = new IODef(*$1, $3, true, "", *$8, "");
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$6 << "! Expected one of [ source, destination]";
  }
  delete $1;
  delete $5;
  delete $6;
  delete $8;
}
| IDENT IDENT IDENT '=' IDENT {
  DSA_CHECK(*$2 == "stated") << "'stated' expected";
  if (*$3 == "source") {
    $$ = new IODef(*$1, 0, true, *$5, "", "");
  } else if (*$3 == "destination") {
    $$ = new IODef(*$1, 0, true, "", *$5, "");
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$3 << "! Expected one of [source, destination]";
  }
  delete $1;
  delete $2;
  delete $3;
  delete $5;
}
| IDENT IDENT '=' IDENT IDENT '=' IDENT {
  if (*$2 == "stated") {
    if (*$5 == "source") {
      $$ = new IODef(*$1, 0, false, *$7, "", *$4);
    } else if (*$4 == "destination") {
      $$ = new IODef(*$1, 0, false, "", *$7, *$4);
    } else {
      DSA_CHECK(false) << "Unknown IO type: " << *$5 << "! Expected one of [source, destination]";
    }
  } else if (*$2 == "source") {
    if (*$5 == "stated") {
      $$ = new IODef(*$1, 0, false, *$4, "", *$7);
    }  {
      DSA_CHECK(false) << "Unknown IO type: " << *$5 << "! Expected one of [stated]. (Hint can't have both source and dest declared).";
    }
  } else if (*$2 == "destination") {
    if (*$5 == "stated") {
      $$ = new IODef(*$1, 0, false, "", *$4, *$7);
    }  {
      DSA_CHECK(false) << "Unknown IO type: " << *$5 << "! Expected one of [stated]. (Hint can't have both source and dest declared).";
    }
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$2 << "! Expected one of [stated, source, destination]";
  }
  delete $1;
  delete $2;
  delete $4;
  delete $5;
  delete $7;
}
| IDENT '[' I_CONST ']' IDENT '=' IDENT IDENT '=' IDENT {
  if (*$5 == "stated") {
    if (*$8 == "source") {
      $$ = new IODef(*$1, $3, false, *$10, "", *$7);
    } else if (*$8 == "destination") {
      $$ = new IODef(*$1, $3, false, "", *$10, *$7);
    } else {
      DSA_CHECK(false) << "Unknown IO type: " << *$8 << "! Expected one of [source, destination]";
    }
  } else if (*$5 == "source") {
    if (*$8 == "stated") {
      $$ = new IODef(*$1, $3, false, *$10, "", *$7);
    }  {
      DSA_CHECK(false) << "Unknown IO type: " << *$8 << "! Expected one of [stated]. (Hint can't have both source and dest declared).";
    }
  } else if (*$5 == "destination") {
    if (*$8 == "stated") {
      $$ = new IODef(*$1, $3, false, "", *$10, *$7);
    }  {
      DSA_CHECK(false) << "Unknown IO type: " << *$8 << "! Expected one of [stated]. (Hint can't have both source and dest declared).";
    }
  } else {
    DSA_CHECK(false) << "Unknown IO type: " << *$5 << "! Expected one of [stated, source, destination]";
  }
  delete $1;
  delete $5;
  delete $7;
  delete $8;
  delete $10;
}
;

task_map_id_def: I_CONST ':' I_CONST {
  $$ = new map_id_t($1, $3);
}
| task_map_id_def ',' {
  $$ = $1;
}
| task_map_id_def ']' {
  $$ = $1;
};

map_def: IDENT '=' IDENT {
  $$ = new std::vector<map_pair_t*>();
  $$->push_back(new map_pair_t(*$1,*$3));
}
| map_def ',' IDENT '=' IDENT {
  $$ = $1;
  $$->push_back(new map_pair_t(*$3,*$5)); // other entries
}
| map_def ']' {
  $$ = $1;
};

rhs: expr { $$ = $1; };

expr: I_CONST {
  $$ = new ConstDataEntry($1); //expr: what you can assign to a var
}
| I_CONST I_CONST {
  $$ = new ConstDataEntry($1, $2);
}
| I_CONST I_CONST I_CONST I_CONST {
  $$ = new ConstDataEntry($1, $2, $3, $4);
}
| F_CONST {
  $$ = new ConstDataEntry($1);
}
| F_CONST F_CONST {
  $$ = new ConstDataEntry((float) $1, (float) $2);
}
| edge_list {
  if (auto ce = dynamic_cast<ConvergeEntry*>($1)) {
    if (ce->entries.size() == 1) {
      $$ = ce->entries[0];
    }
    delete ce;
  } else {
    $$ = $1;
  }
}
| IDENT '(' arg_list ')' {
  auto &opcode = *$1;
  auto &args = *$3;

  dsa::OpCode op = dsa::inst_from_string(opcode.c_str());
  p->dfg->emplace_back<dsa::dfg::Instruction>(p->dfg, op);
  auto *inst = &p->dfg->type_filter<dsa::dfg::Instruction>().back();
  dsa::dfg::UpdateNodeByArgs(inst, args);
  $$ = new NodeEntry(inst->id());
  delete $1;
  delete $3;
}
| IDENT '<' fu_list '>' '(' arg_list ')' {
  auto &opcode = *$1;
  auto &fu_cnt = *$3;
  auto &args = *$6;
  std::vector<dsa::OpCode> fus;
  std::vector<int> cnts;
  for (int i = 0, n = fu_cnt.size(); i < n; ++i) {
    fus.push_back(fu_cnt[i].first);
    cnts.push_back(fu_cnt[i].second);
  }
  p->dfg->emplace_back<dsa::dfg::Operation>(p->dfg, fus, cnts);
  auto *operation = &p->dfg->type_filter<dsa::dfg::Operation>().back();
  DSA_LOG(PARSE) << operation->name();
  DSA_LOG(PARSE) << p->dfg->nodes[operation->id()]->name();
  dsa::dfg::UpdateNodeByArgs(operation, args);
  $$ = new NodeEntry(operation->id());
  delete $1;
  delete $3;
  delete $6;
};

arg_expr : expr {
  $$ = $1;
}
| IDENT '=' expr {
  $$ = new dsa::dfg::ControlEntry(*$1, $3);
}
| IDENT '=' expr '{' ctrl_list  '}' {
  $$ = new dsa::dfg::ControlEntry(*$1, *$5, $3, 7);
  delete $1;
  delete $5;
}
| IDENT '=' '{' ctrl_list  '}' {
  $$ = new dsa::dfg::ControlEntry(*$1, *$4, nullptr, 7);
  delete $1;
  delete $4;
}
| IDENT '=' expr '&' I_CONST '{' ctrl_list  '}' {
  $$ = new dsa::dfg::ControlEntry(*$1, *$7, $3, $5);
  delete $1;
  delete $7;
}
;

arg_list: arg_expr {
  $$ = new std::vector<dsa::dfg::ParseResult*>();
  $$->push_back($1);
}
| arg_list ',' arg_expr {
  $1->push_back($3);
  $$ = $1;
};

task_map_list: ident_list ':' ident_list {
  $$ = new task_def_t();
  $$->push_back(std::make_pair(*$1, *$3));
  delete $3;
}
| task_map_list ',' ident_list ':' ident_list {
  $$ = $1;
  $$->push_back(std::make_pair(*$3, *$5));
  delete $5;
};

ctrl_list: I_CONST ':' ident_list {
  $$ = new ctrl_def_t();
  (*$$)[$1]=*$3;
  delete $3;
}
| ctrl_list ',' I_CONST ':' ident_list {
  $$ = $1;
  (*$$)[$3] = *$5;
  delete $5;
};

value_list: IDENT {
  $$ = new string_vec_t();
  $$->push_back(std::string(*$1));
}
| value_list ',' IDENT {
  $1->push_back(*$3);
  $$ = $1;
};

ident_list: IDENT {
  $$ = new std::vector<std::string>();
  $$->push_back(std::string(*$1));
}
| ident_list '|' IDENT {
  $1->push_back(*$3);
  $$ = $1;
};

fu_list: IDENT ':' I_CONST {
  $$ = new std::vector<std::pair<dsa::OpCode, int>>();
  $$->emplace_back(dsa::inst_from_string($1->c_str()), $3);
}
| fu_list ',' IDENT ':' I_CONST {
  $1->emplace_back(dsa::inst_from_string($3->c_str()), $5);
  $$ = $1;
}

edge_list: edge {
  auto res = new dsa::dfg::ConvergeEntry(); 
  if (auto ne = dynamic_cast<dsa::dfg::ValueEntry*>($1)) {
    res->entries.push_back(ne);
    $$ = res;
  } else if (auto ce = dynamic_cast<dsa::dfg::ConvergeEntry*>($1)) {
    res->entries = ce->entries;
    $$ = res;
  } else if (auto re = dynamic_cast<dsa::dfg::RegisterEntry*>($1)) {
    $$ = re;
  } else {
    DSA_CHECK(0) << "Not supported edge list type!";
  }
}
| edge_list edge {
  auto res = dynamic_cast<dsa::dfg::ConvergeEntry*>($1);
  DSA_CHECK(res);
  if (auto ne = dynamic_cast<dsa::dfg::ValueEntry*>($2)) {
    res->entries.push_back(ne);
  } else if (auto ce = dynamic_cast<dsa::dfg::ConvergeEntry*>($2)) {
    res->entries.insert(res->entries.end(), ce->entries.begin(), ce->entries.end());
  } else {
    DSA_CHECK(0) << "Not supported edge list type!";
  }
  $$ = res;
};

/*edge is a dfgnode annotated with bitwidth/index information*/
edge: IDENT {
    $$ = p->symbols.Get(*$1);
    delete $1;
  }
| IDENT ':' I_CONST ':' I_CONST {
  auto ne = dynamic_cast<dsa::dfg::ValueEntry*>(p->symbols.Get(*$1));
  DSA_CHECK(ne) << "For now only result values supports slicing!";
  if (ne->l == (int) $3 && ne->r == (int) $5) {
    $$ = ne;
  } else {
    std::ostringstream oss;
    oss << *$1 << ':' << $3 << ':' << $5;
    if (!p->symbols.Has(oss.str())) {
      p->symbols.Set(oss.str(), new dsa::dfg::ValueEntry(ne->nid, ne->vid, $3, $5));
    }
    $$ = p->symbols.Get(oss.str());
  }
  delete $1;
};

%%

int parse_dfg(const char* filename, SSDfg* _dfg) {
  FILE* dfg_file = fopen(filename, "r");
  yylineno = 1;
  if (dfg_file == NULL) {
    perror(" Failed ");
    exit(1);
  }

  struct parse_param p;
  p.dfg = _dfg;

  yyrestart(dfg_file);
  yyparse(&p);
  return 0;
}


static void yyerror(struct parse_param* p, char const* s) {
  fprintf(stderr, "Error parsing DFG at line %d: %s\n", yylineno, s);
  DSA_CHECK(0);
}
