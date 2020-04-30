/* A Bison parser, made by GNU Bison 3.0.4.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

#ifndef YY_YY_HOME_SIHAO_REPO_SS_STACK_SS_SCHEDULER_SRC_SS_SCHEDULER_DFG_PARSER_TAB_H_INCLUDED
# define YY_YY_HOME_SIHAO_REPO_SS_STACK_SS_SCHEDULER_SRC_SS_SCHEDULER_DFG_PARSER_TAB_H_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 1
#endif
#if YYDEBUG
extern int yydebug;
#endif
/* "%code requires" blocks.  */
#line 1 "dfg-parser.y" /* yacc.c:1909  */

  #include <stdint.h>
  #include "ssdfg.h"
  int parse_dfg(const char* dfgfile, SSDfg* dfg);
  typedef std::pair<std::string,int> io_pair_t;

#line 51 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.h" /* yacc.c:1909  */

/* Token type.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    EOLN = 258,
    NEW_DFG = 259,
    PRAGMA = 260,
    ARROW = 261,
    IDENT = 262,
    STRING_LITERAL = 263,
    F_CONST = 264,
    I_CONST = 265,
    INPUT = 266,
    OUTPUT = 267
  };
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED

union YYSTYPE
{
#line 41 "dfg-parser.y" /* yacc.c:1909  */

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

#line 91 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.h" /* yacc.c:1909  */
};

typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE yylval;

int yyparse (struct parse_param* p);

#endif /* !YY_YY_HOME_SIHAO_REPO_SS_STACK_SS_SCHEDULER_SRC_SS_SCHEDULER_DFG_PARSER_TAB_H_INCLUDED  */
