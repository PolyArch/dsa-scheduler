%code requires {

#include <stdint.h>
#include "dsa/dfg/node.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/symbols.h"
#include "dsa/dfg/node.h"
int parse_dfg(const char* dfgfile, SSDfg* dfg);
typedef std::pair<std::string,int> io_pair_t;
typedef std::pair<int,int> map_pair_t;

using ParseResult = dsa::dfg::ParseResult;
using ControlEntry = dsa::dfg::ControlEntry;
using ConvergeEntry = dsa::dfg::ConvergeEntry;
using ConstDataEntry = dsa::dfg::ConstDataEntry;
using ValueEntry = dsa::dfg::ValueEntry;
using EdgeType = dsa::dfg::OperandType;

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
  io_pair_t* io_pair;
  map_pair_t* map_pair;
  std::vector<ParseResult*> *sym_vec;
  dsa::dfg::ParseResult *sym_ent;
  string_vec_t* str_vec;
  ctrl_def_t* ctrl_def;
  task_def_t* task_def;

  YYSTYPE() {}   // this is only okay because sym_ent doesn't need a
  ~YYSTYPE() {}  // real constructor/deconstructuor (but string/vector do)
}

%token	  EOLN NEW_DFG PRAGMA ARROW
%token<s> IDENT STRING_LITERAL
%token<d> F_CONST
%token<i> I_CONST INPUT OUTPUT INDIRECT TASKDEP

%type <io_pair> io_def
%type <map_pair> map_def
%type <sym_vec> arg_list
%type <sym_ent> arg_expr rhs expr edge edge_list
%type <ctrl_def> ctrl_list
%type <str_vec> ident_list value_list
%type <task_def> task_map_list


%start statement_list
%debug
%error-verbose

%%

statement_list: statement
	            | statement_list statement;

statement: INPUT ':' io_def  eol {
  CHECK(!p->symbols.Has($3->first)) << "Already has the symbol " << $3->first;
  auto name = $3->first;
  int len = $3->second;
  int width = $1;
  int n = std::max(1, len);
  int slice = 64 / width;
  p->dfg->emplace_back<SSDfgVecInput>(n, width, name, p->dfg, p->meta);
  LOG(PARSE) << p->dfg->vins.back().id() << " " << p->dfg->vins.back().name();
  int left_len = 0;
  // printf("n: , slice: %d %d", n, slice);
  for (int i = 0, cnt = 0; i < n; i += slice) {
    left_len = slice;
    if (n - i > 0) {
      left_len = std::min(n - i, slice);
    }
    // printf("left_len: , width: %d %d", left_len, width);
    // for each len and width, create a symbol or a new vins
    for (int j = 0; j < left_len * width; j += width) {
      std::stringstream ss;
      ss << name;
      if (len) ss << cnt++;
      // TODO(@were): Do I need to modularize these two clean up segment?
      p->symbols.Set(ss.str(), new ValueEntry(p->dfg->vins.back().id(), i, j, j + width - 1));
    }
  }
  p->meta.clear();
  delete $3;
}
| OUTPUT ':' io_def eol {
  using T = SSDfgVecOutput;
  auto name = $3->first;
  int len = $3->second;
  int width = $1;
  int n = std::max(1, len);
  int slice = 64 / width;
  // int t = ceil(n / float(slice)); -- FIXME: do we need this?
  // I think it's somewhat likely i am breaking decomposability
  p->dfg->emplace_back<T>(n, width, name, p->dfg, p->meta);
  LOG(PARSE) << p->dfg->vouts.back().name() << " " << p->dfg->vouts.back().id();
  int left_len = 0;
  // printf("n: , slice: %d %d", n, slice);
  for (int i = 0, cnt = 0; i < n; i += slice) {
    left_len = slice;
    if (n - i > 0) {
      left_len = std::min(n - i, slice);
    }
    // printf("left_len: , width: %d %d", left_len, width);
    for (int j = 0; j < left_len * width; j += width) {
      std::stringstream ss;
      ss << name;
      // at output, we make the connectivity because the previous edge would already be there..
      if (len) ss << cnt++;
      // TODO(@were): Do I need to modularize these two clean up segment?
      auto sym = p->symbols.Get(ss.str()); // check in a symbols array?
      if (auto ce = dynamic_cast<dsa::dfg::ConvergeEntry*>(sym)) {
        int num_entries = ce->entries.size();
        assert(num_entries > 0 && num_entries <= 16);
        std::vector<int> es;
        for (auto elem : ce->entries) {
          // printf("nid: %d vid: %d l: %d r: %d", elem->nid, elem->vid, elem->l, elem->r);
          p->dfg->edges.emplace_back(
            p->dfg, elem->nid, elem->vid,
            p->dfg->vouts.back().id(), elem->l, elem->r);
          es.push_back(p->dfg->edges.back().id);
        }
        p->dfg->vouts.back().ops().emplace_back(p->dfg, es, EdgeType::data);
      } else if (auto ne = dynamic_cast<dsa::dfg::ValueEntry*>(sym)) {
        // printf("nid: %d vid: %d l: %d r: %d", ne->nid, ne->vid, ne->l, ne->r);
        p->dfg->edges.emplace_back(
          p->dfg, ne->nid, ne->vid,
          p->dfg->vouts.back().id(), ne->l, ne->r);
        std::vector<int> es{p->dfg->edges.back().id};
        p->dfg->vouts.back().ops().emplace_back(p->dfg, es, EdgeType::data);
      }
    }
  }
  p->meta.clear();
  delete $3;
}
| INDIRECT ':' io_def eol { // create 1 input, 1 output and edge
  auto name = $3->first;
  int len = $3->second;
  int width = $1;
  int n = std::max(1, len);
  int slice = 64 / width;
  
  // add input/output ports
  p->dfg->emplace_back<SSDfgVecInput>(n, width, name, p->dfg, p->meta);
  p->dfg->emplace_back<SSDfgVecOutput>(n, width, name, p->dfg, p->meta, true);
  
  // int nid=p->dfg->vins.back().id(); // this creates seg fault...it should get its own id
  // p->symbols.Set(std::string(name), new ValueEntry(nid, 0));
  // std::stringstream ss;
  // ss << name;
  // auto sym = p->symbols.Get(ss.str());
  // auto ne = dynamic_cast<dsa::dfg::ValueEntry*>(sym);
  
  int nid = p->dfg->vins.back().id();
  int vid = 0;
  int iid = p->dfg->vouts.back().id();
  int l=0, r=63;

  p->dfg->edges.emplace_back(
    p->dfg, nid, vid,
    iid, l, r);

  std::vector<int> es{p->dfg->edges.back().id};

  // set ops on this output
  p->dfg->vouts.back().ops().emplace_back(p->dfg, es, EdgeType::data);
  // p->dfg->vins.back().values[0].uses.push_back(p->dfg->edges.back().id);
  // LOG(PARSE) << p->dfg->vouts.back().name() << " " << p->dfg->vouts.back().id();

  p->meta.clear();
  delete $3;
}
| TASKDEP map_def ':' '(' task_map_list ')' eol {
  p->dfg->create_new_task_dependence_map($2->first, $2->second);
  auto &args = *$5;
  for(unsigned i=0; i<args.size(); ++i) {
    p->dfg->add_new_task_dependence_map(args[i].first, args[i].second);
  }
  p->meta.clear();
  delete $5;
}
| value_list '=' rhs eol {
  if (auto ne = dynamic_cast<ValueEntry*>($3)) {
    assert($1->size()==1);
    std::string name = (*$1)[0];
    p->symbols.Set(name, new ValueEntry(ne->nid, ne->vid, ne->l, ne->r));
  } else if (auto ce = dynamic_cast<ConvergeEntry*>($3)) {
    // By definition, converge entries only need one symbol (just def)
    if ($1->size()==1) {
      std::string name = (*$1)[0];
      printf("name of converge entry: %s", name);
      for (auto elem : ce->entries) {
        printf("nid of converge entry is: %d", elem->nid);
        auto node = p->dfg->nodes[elem->nid];
        if(!node->has_name()) node->set_name(name);
      }
      p->symbols.Set(name, $3);
    } else if ($1->size() == ce->entries.size()) {
      auto o = dynamic_cast<ValueEntry*>(ce->entries[0]);
      assert(o && "Assumption check failure, the first element should be a value entry!");
      assert(dynamic_cast<SSDfgInst*>(p->dfg->nodes[o->nid]) && "Should be a instruction!");
      p->symbols.Set((*$1)[0], new ValueEntry(o->nid, o->vid));
      for (int i = 1, n = ce->entries.size(); i < n; ++i) {
        if (auto ve = dynamic_cast<ValueEntry*>(ce->entries[i])) {
          assert(ve->nid == o->nid && "Should all from the same node!");
          p->symbols.Set((*$1)[i], new ValueEntry(ve->nid, 0));
        } else {
          assert(false && "Assumption check failure! All the remaining element should also be a value entry!");
        }
      }
    }
  }
  delete $1;
}
| NEW_DFG eol {
  p->dfg->start_new_dfg_group();
}
| PRAGMA IDENT IDENT eol {
  p->dfg->set_pragma(*$2,*$3);
  delete $2;
  delete $3;
}
| PRAGMA IDENT IDENT I_CONST eol {
  assert(*$2 == "group");
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

// Input: A[8]
io_def: IDENT {
  $$ = new io_pair_t(*$1,0);
  delete $1;
}
| IDENT '[' I_CONST ']' {
    $$ = new io_pair_t(*$1,$3);
    delete $1;
};

map_def: '[' I_CONST ':' I_CONST ']' {
  $$ = new map_pair_t($2,$4);
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
  auto ce = dynamic_cast<ConvergeEntry*>($1);
  assert(ce);
  if (ce->entries.size() == 1) {
    $$ = ce->entries[0];
    delete ce;
  } else {
    $$ = $1; // What does this mean???
  }
}
| IDENT '(' arg_list ')' {
  auto &opcode = *$1;
  auto &args = *$3;


  dsa::OpCode op = dsa::inst_from_string(opcode.c_str());
  p->dfg->emplace_back<SSDfgInst>(p->dfg, op);
  auto *inst = &p->dfg->type_filter<SSDfgInst>().back();
  int iid = inst->id();
  for (unsigned i = 0; i < args.size(); ++i) {
    if (auto data = dynamic_cast<ConstDataEntry*>(args[i])) {
      inst->ops().emplace_back(data->data);
    } else if (auto ne = dynamic_cast<ValueEntry*>(args[i])) {
      p->dfg->edges.emplace_back(p->dfg, ne->nid, ne->vid, iid, ne->l, ne->r);
      std::vector<int> es{p->dfg->edges.back().id};
      inst->ops().emplace_back(p->dfg, es, EdgeType::data);
    } else if (auto ce = dynamic_cast<ConvergeEntry*>(args[i])) {
      std::vector<int> es;
      for (auto elem : ce->entries) {
        if (auto ne = dynamic_cast<ValueEntry*>(elem)) {
          p->dfg->edges.emplace_back(p->dfg, ne->nid, ne->vid, iid, ne->l, ne->r);
          es.push_back(p->dfg->edges.back().id);
        }
      }
      inst->ops().emplace_back(p->dfg, es, EdgeType::data );
    } else if (auto ce = dynamic_cast<ControlEntry*>(args[i])) {
      // External control
      if (ce->controller) {
        auto ne = dynamic_cast<ValueEntry*>(ce->controller);
        p->dfg->edges.emplace_back(p->dfg, ne->nid, ne->vid, iid, ne->l, ne->r);
        inst->predicate = ce->bits;
        std::vector<int> es{p->dfg->edges.back().id};
        inst->ops().emplace_back(p->dfg, es, ce->flag);
      } else {
        // Self control
        inst->self_predicate = ce->bits;
      }
    } else {
      CHECK(false) << "Invalide Node type";
      throw;
    }
  }
  std::vector<ValueEntry*> values(inst->values.size());
  for (int i = 0, n = values.size(); i < n; ++i) {
    values[i] = new ValueEntry(inst->id(), i, 0, inst->bitwidth() - 1);
  }
  if (values.size() == 1) {
    $$ = values[0];
  } else {
    auto res = new ConvergeEntry();
    res->entries = values;
    $$ = res;
  }

  delete $1;
  delete $3;
};

arg_expr : expr {
  $$ = $1;
}
| IDENT '=' expr {
  $$ = new dsa::dfg::ControlEntry(*$1, $3);
}
| IDENT '=' expr '{' ctrl_list  '}' {
  $$ = new dsa::dfg::ControlEntry(*$1, *$5, $3);
  delete $5;
}
| IDENT '=' '{' ctrl_list  '}' {
  $$ = new dsa::dfg::ControlEntry(*$1, *$4, nullptr);
  delete $4;
};

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
  (*$$)[$3]=*$5;
  delete $5;
};

value_list: IDENT {
  $$ = new string_vec_t();
  $$->push_back(std::string(*$1));
}
| value_list ',' IDENT {
  $1->push_back(*$3); $$ = $1;
};

ident_list: IDENT {
  $$ = new string_vec_t();
  $$->push_back(std::string(*$1));
}
| ident_list '|' IDENT {
  $1->push_back(*$3);
  $$ = $1;
};

edge_list: edge {
  auto res = new dsa::dfg::ConvergeEntry(); 
  if (auto ne = dynamic_cast<dsa::dfg::ValueEntry*>($1)) {
    res->entries.push_back(ne);
  } else if (auto ce = dynamic_cast<dsa::dfg::ConvergeEntry*>($1)) {
    res->entries = ce->entries;
  } else {
    assert(0 && "Not supported edge list type!");
  }
  $$ = res;
}
| edge_list edge {
  auto res = dynamic_cast<dsa::dfg::ConvergeEntry*>($1);
  assert(res);
  if (auto ne = dynamic_cast<dsa::dfg::ValueEntry*>($2)) {
    res->entries.insert(res->entries.begin(), ne);
  } else if (auto ce = dynamic_cast<dsa::dfg::ConvergeEntry*>($1)) {
    res->entries.insert(res->entries.begin(), ce->entries.begin(), ce->entries.end());
  } else {
    assert(0 && "Not supported edge list type!");
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
  assert(ne && "For now only result values supports slicing!");
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
  assert(0);
}
