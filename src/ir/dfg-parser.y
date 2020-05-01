%code requires {
  #include <stdint.h>
  #include "dsa/ir/ssdfg.h"
  int parse_dfg(const char* dfgfile, SSDfg* dfg);
  typedef std::pair<std::string,int> io_pair_t;
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
  EntryTable symbols;
  SSDfg* dfg;
  ssdfg::MetaPort meta;
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
    std::vector<ParseResult*> *sym_vec;
    ParseResult *sym_ent;
    string_vec_t* str_vec;
    ctrl_def_t* ctrl_def;

    YYSTYPE() {}   // this is only okay because sym_ent doesn't need a
    ~YYSTYPE() {}  // real constructor/deconstructuor (but string/vector do)
}

%token	  EOLN NEW_DFG PRAGMA ARROW
%token<s> IDENT STRING_LITERAL
%token<d> F_CONST
%token<i> I_CONST INPUT OUTPUT

%type <io_pair> io_def
%type <sym_vec> arg_list
%type <sym_ent> arg_expr rhs expr edge edge_list
%type <ctrl_def> ctrl_list
%type <str_vec> ident_list value_list


%start statement_list
%debug
%error-verbose

%%

statement_list: statement
	            | statement_list statement;

statement
    : INPUT ':' io_def  eol {
        if(p->symbols.has_sym($3->first)) {
          printf("Symbol \"%s\" already exists\n", $3->first.c_str());
          assert(0);
        }
        p->dfg->addVecInput($3->first, $3->second, p->symbols, $1, p->meta);
        p->meta.clear();
        delete $3;
      }
    | OUTPUT ':' io_def eol {
        p->dfg->addVecOutput($3->first.c_str(), $3->second, p->symbols, $1, p->meta);
        p->meta.clear();
        delete $3;
      }
    | value_list '=' rhs eol {
        if (auto ne = dynamic_cast<ValueEntry*>($3)) {
          assert($1->size()==1);
          std::string name = (*$1)[0];
          p->symbols.set(name, new ValueEntry(ne->value, ne->l, ne->r));
        } else if (auto ce = dynamic_cast<ConvergeEntry*>($3)) {
          //By definition, converge entries only need one symbol (just def)
          if ($1->size()==1) {
            std::string name = (*$1)[0];
            for (auto elem : ce->entries) {
              auto node = elem->value->node();
              if(!node->has_name()) node->set_name(name);
            }
            p->symbols.set(name, $3);
          } else if ($1->size() == ce->entries.size()) {
            auto o = dynamic_cast<ValueEntry*>(ce->entries[0]);
            assert(o && "Assumption check failure, the first element should be a value entry!");
            assert(dynamic_cast<SSDfgInst*>(o->value->node()) && "Should be a instruction!");
            p->symbols.set((*$1)[0], o->value);
            for (int i = 1, n = ce->entries.size(); i < n; ++i) {
              if (auto ve = dynamic_cast<ValueEntry*>(ce->entries[i])) {
                assert(ve->value->node() == o->value->node() && "Should all from the same node!");
                p->symbols.set((*$1)[i], ve->value);
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

io_def
	: IDENT {
	    $$ = new io_pair_t(*$1,0);
	    delete $1;
	  }
	| IDENT '[' I_CONST ']' {
	    $$ = new io_pair_t(*$1,$3);
	    delete $1;
	  }
	;

rhs : expr { $$ = $1; } ;

expr
    : I_CONST {
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
          $$ = $1;
        }
      }
    | IDENT '(' arg_list ')' {
        $$ = p->dfg->create_inst(*$1,*$3);
        delete $1;
        delete $3;
      }
    ;


/* Argument expressions can have extra flag arguments as well */
arg_expr 
	: expr {
	    $$ = $1;
	  }
	| IDENT '=' expr {
	    $$ = new ControlEntry(*$1, $3);
	  }
	| IDENT '=' expr '{' ctrl_list  '}' {
            $$ = new ControlEntry(*$1, *$5, $3);
            delete $5;
          }
	| IDENT '=' '{' ctrl_list  '}' {
            $$ = new ControlEntry(*$1, *$4, nullptr);
            delete $4;
          }
    ;

arg_list
    : arg_expr {
        $$ = new std::vector<ParseResult*>();
        $$->push_back($1);
      }
    | arg_list ',' arg_expr {
        $1->push_back($3);
        $$ = $1;
      }
    ;

ctrl_list
    : I_CONST ':' ident_list {
        $$ = new ctrl_def_t();
        (*$$)[$1]=*$3; delete $3;
      }
    | ctrl_list ',' I_CONST ':' ident_list {
        $$ = $1;
        (*$$)[$3]=*$5; delete $5;
      }
    ;

value_list
    : IDENT {
        $$ = new string_vec_t(); $$->push_back(std::string(*$1));
      }
    | value_list ',' IDENT {
        $1->push_back(*$3); $$ = $1;
      }
    ;

ident_list
    : IDENT {
        $$ = new string_vec_t(); $$->push_back(std::string(*$1));
      }
    | ident_list '|' IDENT {
        $1->push_back(*$3); $$ = $1;
      }
    ;

edge_list
    : edge {
        auto res = new ConvergeEntry(); 
        if (auto ne = dynamic_cast<ValueEntry*>($1)) {
          res->entries.push_back(ne);
        } else if (auto ce = dynamic_cast<ConvergeEntry*>($1)) {
          res->entries = ce->entries;
        } else {
          assert(0 && "Not supported edge list type!");
        }
        $$ = res;
      }
    | edge_list edge {
        auto res = dynamic_cast<ConvergeEntry*>($1);
        assert(res);
        if (auto ne = dynamic_cast<ValueEntry*>($2)) {
          res->entries.insert(res->entries.begin(), ne);
        } else if (auto ce = dynamic_cast<ConvergeEntry*>($1)) {
          res->entries.insert(res->entries.begin(), ce->entries.begin(), ce->entries.end());
        } else {
          assert(0 && "Not supported edge list type!");
        }
        $$ = res;
      }
    ;

/*edge is a dfgnode annotated with bitwidth/index information*/
edge
    : IDENT {
        $$ = p->symbols.get_sym(*$1);
        delete $1;
      }
    | IDENT ':' I_CONST ':' I_CONST {
        auto ne = dynamic_cast<ValueEntry*>(p->symbols.get_sym(*$1));
        assert(ne && "For now only result values supports slicing!");
        if (ne->l == (int) $3 && ne->r == (int) $5) {
          $$ = ne;
        } else {
          std::ostringstream oss;
          oss << *$1 << ':' << $3 << ':' << $5;
          if (!p->symbols.has_sym(oss.str())) {
            p->symbols.set(oss.str(), new ValueEntry(ne->value, $3, $5));
          }
          $$ = p->symbols.get_sym(oss.str());
        }
        delete $1;
      }
    ;

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