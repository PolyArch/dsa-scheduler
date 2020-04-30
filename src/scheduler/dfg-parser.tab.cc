/* A Bison parser, made by GNU Bison 3.0.4.  */

/* Bison implementation for Yacc-like parsers in C

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

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "3.0.4"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1




/* Copy the first part of user declarations.  */
#line 8 "dfg-parser.y" /* yacc.c:339  */

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

#line 90 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:339  */

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 1
#endif

/* In a future release of Bison, this section will be replaced
   by #include "dfg-parser.tab.h".  */
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
#line 1 "dfg-parser.y" /* yacc.c:355  */

  #include <stdint.h>
  #include "ssdfg.h"
  int parse_dfg(const char* dfgfile, SSDfg* dfg);
  typedef std::pair<std::string,int> io_pair_t;

#line 127 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:355  */

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
#line 41 "dfg-parser.y" /* yacc.c:355  */

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

#line 167 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:355  */
};

typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE yylval;

int yyparse (struct parse_param* p);

#endif /* !YY_YY_HOME_SIHAO_REPO_SS_STACK_SS_SCHEDULER_SRC_SS_SCHEDULER_DFG_PARSER_TAB_H_INCLUDED  */

/* Copy the second part of user declarations.  */

#line 184 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:358  */

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#else
typedef signed char yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(Msgid) dgettext ("bison-runtime", Msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(Msgid) Msgid
# endif
#endif

#ifndef YY_ATTRIBUTE
# if (defined __GNUC__                                               \
      && (2 < __GNUC__ || (__GNUC__ == 2 && 96 <= __GNUC_MINOR__)))  \
     || defined __SUNPRO_C && 0x5110 <= __SUNPRO_C
#  define YY_ATTRIBUTE(Spec) __attribute__(Spec)
# else
#  define YY_ATTRIBUTE(Spec) /* empty */
# endif
#endif

#ifndef YY_ATTRIBUTE_PURE
# define YY_ATTRIBUTE_PURE   YY_ATTRIBUTE ((__pure__))
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# define YY_ATTRIBUTE_UNUSED YY_ATTRIBUTE ((__unused__))
#endif

#if !defined _Noreturn \
     && (!defined __STDC_VERSION__ || __STDC_VERSION__ < 201112)
# if defined _MSC_VER && 1200 <= _MSC_VER
#  define _Noreturn __declspec (noreturn)
# else
#  define _Noreturn YY_ATTRIBUTE ((__noreturn__))
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(E) ((void) (E))
#else
# define YYUSE(E) /* empty */
#endif

#if defined __GNUC__ && 407 <= __GNUC__ * 100 + __GNUC_MINOR__
/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN \
    _Pragma ("GCC diagnostic push") \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")\
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# define YY_IGNORE_MAYBE_UNINITIALIZED_END \
    _Pragma ("GCC diagnostic pop")
#else
# define YY_INITIAL_VALUE(Value) Value
#endif
#ifndef YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_END
#endif
#ifndef YY_INITIAL_VALUE
# define YY_INITIAL_VALUE(Value) /* Nothing. */
#endif


#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined EXIT_SUCCESS
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
      /* Use EXIT_SUCCESS as a witness for stdlib.h.  */
#     ifndef EXIT_SUCCESS
#      define EXIT_SUCCESS 0
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's 'empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined EXIT_SUCCESS \
       && ! ((defined YYMALLOC || defined malloc) \
             && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef EXIT_SUCCESS
#    define EXIT_SUCCESS 0
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined EXIT_SUCCESS
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined EXIT_SUCCESS
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
         || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

# define YYCOPY_NEEDED 1

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)                           \
    do                                                                  \
      {                                                                 \
        YYSIZE_T yynewbytes;                                            \
        YYCOPY (&yyptr->Stack_alloc, Stack, yysize);                    \
        Stack = &yyptr->Stack_alloc;                                    \
        yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
        yyptr += yynewbytes / sizeof (*yyptr);                          \
      }                                                                 \
    while (0)

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from SRC to DST.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(Dst, Src, Count) \
      __builtin_memcpy (Dst, Src, (Count) * sizeof (*(Src)))
#  else
#   define YYCOPY(Dst, Src, Count)              \
      do                                        \
        {                                       \
          YYSIZE_T yyi;                         \
          for (yyi = 0; yyi < (Count); yyi++)   \
            (Dst)[yyi] = (Src)[yyi];            \
        }                                       \
      while (0)
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  16
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   102

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  24
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  14
/* YYNRULES -- Number of rules.  */
#define YYNRULES  41
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  84

/* YYTRANSLATE[YYX] -- Symbol number corresponding to YYX as returned
   by yylex, with out-of-bounds checking.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   267

#define YYTRANSLATE(YYX)                                                \
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, without out-of-bounds checking.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
      18,    19,     2,     2,    22,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,    13,    15,
       2,    14,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,    16,     2,    17,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    20,    23,    21,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12
};

#if YYDEBUG
  /* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,    71,    71,    72,    76,    85,    90,   123,   126,   131,
     139,   142,   147,   152,   155,   156,   160,   164,   170,   173,
     176,   179,   182,   185,   188,   198,   208,   211,   214,   218,
     225,   229,   236,   240,   247,   250,   256,   259,   265,   276,
     292,   297
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || 1
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "EOLN", "NEW_DFG", "PRAGMA", "ARROW",
  "IDENT", "STRING_LITERAL", "F_CONST", "I_CONST", "INPUT", "OUTPUT",
  "':'", "'='", "';'", "'['", "']'", "'('", "')'", "'{'", "'}'", "','",
  "'|'", "$accept", "statement_list", "statement", "eol", "io_def", "rhs",
  "expr", "arg_expr", "arg_list", "ctrl_list", "value_list", "ident_list",
  "edge_list", "edge", YY_NULLPTR
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[NUM] -- (External) token number corresponding to the
   (internal) symbol number NUM (which must be that of a token).  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,    58,    61,    59,    91,    93,    40,    41,
     123,   125,    44,   124
};
# endif

#define YYPACT_NINF -19

#define yypact_value_is_default(Yystate) \
  (!!((Yystate) == (-19)))

#define YYTABLE_NINF -1

#define yytable_value_is_error(Yytable_value) \
  0

  /* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
     STATE-NUM.  */
static const yytype_int8 yypact[] =
{
      26,   -19,     0,     9,   -19,     7,    13,   -19,     2,   -19,
     -19,   -10,   -19,    11,    17,    17,   -19,   -19,    43,    21,
      36,    47,    24,     0,     0,    29,    39,    45,     0,   -19,
      60,   -19,   -19,     0,   -19,     0,     0,     0,    58,   -19,
     -19,    59,    51,   -19,    61,   -19,    57,   -19,   -19,   -19,
     -19,   -19,    55,    62,    31,   -19,   -19,    40,    63,   -19,
      64,     1,   -19,    51,   -19,   -19,    66,    65,   -19,    67,
      42,    66,    70,   -19,    68,    44,   -19,    56,    69,   -19,
      74,    70,   -19,    56
};

  /* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
     Performed when YYTABLE does not specify something else to do.  Zero
     means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       0,    15,     0,     0,    34,     0,     0,    14,     0,     2,
      13,     0,     7,     0,     0,     0,     1,     3,     0,     0,
       0,     0,    16,     0,     0,    40,    22,    19,     0,    18,
      24,    38,    35,     0,     8,     0,     0,     0,     0,     4,
       5,     0,     0,    23,    20,     6,    40,    39,     9,    10,
      12,    11,     0,     0,    40,    26,    30,     0,     0,    17,
       0,     0,    25,     0,    21,    41,     0,    27,    31,     0,
       0,     0,     0,    29,     0,     0,    36,    32,     0,    28,
       0,     0,    37,    33
};

  /* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
     -19,   -19,    75,    -1,    71,   -19,   -18,    25,   -19,    16,
     -19,     3,   -19,    72
};

  /* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int8 yydefgoto[] =
{
      -1,     8,     9,    10,    23,    28,    55,    56,    57,    70,
      11,    77,    30,    31
};

  /* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
     positive, shift that token.  If negative, reduce the rule whose
     number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_uint8 yytable[] =
{
      29,    12,    16,     1,    18,     1,     2,     3,    25,     4,
      26,    27,    19,     5,     6,     7,    13,     7,    20,    34,
      14,    66,    39,    40,    22,    21,    15,    45,    32,     1,
       2,     3,    48,     4,    49,    50,    51,     5,     6,     1,
      38,     7,    41,    67,    41,    61,    33,    42,    43,    42,
      25,     7,    26,    27,    35,    44,    36,    37,    54,    62,
      26,    27,    63,    73,    74,    79,    74,    46,    52,    53,
      41,    58,    59,    64,    65,    60,    69,    76,    78,    80,
      72,    82,    81,    17,    83,    71,    24,    75,    68,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,    47
};

static const yytype_int8 yycheck[] =
{
      18,     2,     0,     3,    14,     3,     4,     5,     7,     7,
       9,    10,    22,    11,    12,    15,     7,    15,     7,    20,
      13,    20,    23,    24,     7,    14,    13,    28,     7,     3,
       4,     5,    33,     7,    35,    36,    37,    11,    12,     3,
      16,    15,    13,    61,    13,    14,    10,    18,     9,    18,
       7,    15,     9,    10,     7,    10,     9,    10,     7,    19,
       9,    10,    22,    21,    22,    21,    22,     7,    10,    10,
      13,    10,    17,    10,    10,    13,    10,     7,    10,    23,
      13,     7,    13,     8,    81,    20,    15,    71,    63,    -1,
      -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
      -1,    -1,    30
};

  /* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
     symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     3,     4,     5,     7,    11,    12,    15,    25,    26,
      27,    34,    27,     7,    13,    13,     0,    26,    14,    22,
       7,    14,     7,    28,    28,     7,     9,    10,    29,    30,
      36,    37,     7,    10,    27,     7,     9,    10,    16,    27,
      27,    13,    18,     9,    10,    27,     7,    37,    27,    27,
      27,    27,    10,    10,     7,    30,    31,    32,    10,    17,
      13,    14,    19,    22,    10,    10,    20,    30,    31,    10,
      33,    20,    13,    21,    22,    33,     7,    35,    10,    21,
      23,    13,     7,    35
};

  /* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    24,    25,    25,    26,    26,    26,    26,    26,    26,
      26,    26,    26,    26,    27,    27,    28,    28,    29,    30,
      30,    30,    30,    30,    30,    30,    31,    31,    31,    31,
      32,    32,    33,    33,    34,    34,    35,    35,    36,    36,
      37,    37
};

  /* YYR2[YYN] -- Number of symbols on the right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     1,     2,     4,     4,     4,     2,     4,     5,
       5,     5,     5,     1,     1,     1,     1,     4,     1,     1,
       2,     4,     1,     2,     1,     4,     1,     3,     6,     5,
       1,     3,     3,     5,     1,     3,     1,     3,     1,     2,
       1,     5
};


#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (yychar = YYEMPTY)
#define YYEMPTY         (-2)
#define YYEOF           0

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab


#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                                  \
do                                                              \
  if (yychar == YYEMPTY)                                        \
    {                                                           \
      yychar = (Token);                                         \
      yylval = (Value);                                         \
      YYPOPSTACK (yylen);                                       \
      yystate = *yyssp;                                         \
      goto yybackup;                                            \
    }                                                           \
  else                                                          \
    {                                                           \
      yyerror (p, YY_("syntax error: cannot back up")); \
      YYERROR;                                                  \
    }                                                           \
while (0)

/* Error token number */
#define YYTERROR        1
#define YYERRCODE       256



/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)                        \
do {                                            \
  if (yydebug)                                  \
    YYFPRINTF Args;                             \
} while (0)

/* This macro is provided for backward compatibility. */
#ifndef YY_LOCATION_PRINT
# define YY_LOCATION_PRINT(File, Loc) ((void) 0)
#endif


# define YY_SYMBOL_PRINT(Title, Type, Value, Location)                    \
do {                                                                      \
  if (yydebug)                                                            \
    {                                                                     \
      YYFPRINTF (stderr, "%s ", Title);                                   \
      yy_symbol_print (stderr,                                            \
                  Type, Value, p); \
      YYFPRINTF (stderr, "\n");                                           \
    }                                                                     \
} while (0)


/*----------------------------------------.
| Print this symbol's value on YYOUTPUT.  |
`----------------------------------------*/

static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep, struct parse_param* p)
{
  FILE *yyo = yyoutput;
  YYUSE (yyo);
  YYUSE (p);
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# endif
  YYUSE (yytype);
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep, struct parse_param* p)
{
  YYFPRINTF (yyoutput, "%s %s (",
             yytype < YYNTOKENS ? "token" : "nterm", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep, p);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

static void
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)                            \
do {                                                            \
  if (yydebug)                                                  \
    yy_stack_print ((Bottom), (Top));                           \
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

static void
yy_reduce_print (yytype_int16 *yyssp, YYSTYPE *yyvsp, int yyrule, struct parse_param* p)
{
  unsigned long int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       yystos[yyssp[yyi + 1 - yynrhs]],
                       &(yyvsp[(yyi + 1) - (yynrhs)])
                                              , p);
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)          \
do {                                    \
  if (yydebug)                          \
    yy_reduce_print (yyssp, yyvsp, Rule, p); \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif


#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
static YYSIZE_T
yystrlen (const char *yystr)
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
static char *
yystpcpy (char *yydest, const char *yysrc)
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
        switch (*++yyp)
          {
          case '\'':
          case ',':
            goto do_not_strip_quotes;

          case '\\':
            if (*++yyp != '\\')
              goto do_not_strip_quotes;
            /* Fall through.  */
          default:
            if (yyres)
              yyres[yyn] = *yyp;
            yyn++;
            break;

          case '"':
            if (yyres)
              yyres[yyn] = '\0';
            return yyn;
          }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into *YYMSG, which is of size *YYMSG_ALLOC, an error message
   about the unexpected token YYTOKEN for the state stack whose top is
   YYSSP.

   Return 0 if *YYMSG was successfully written.  Return 1 if *YYMSG is
   not large enough to hold the message.  In that case, also set
   *YYMSG_ALLOC to the required number of bytes.  Return 2 if the
   required number of bytes is too large to store.  */
static int
yysyntax_error (YYSIZE_T *yymsg_alloc, char **yymsg,
                yytype_int16 *yyssp, int yytoken)
{
  YYSIZE_T yysize0 = yytnamerr (YY_NULLPTR, yytname[yytoken]);
  YYSIZE_T yysize = yysize0;
  enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
  /* Internationalized format string. */
  const char *yyformat = YY_NULLPTR;
  /* Arguments of yyformat. */
  char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
  /* Number of reported tokens (one for the "unexpected", one per
     "expected"). */
  int yycount = 0;

  /* There are many possibilities here to consider:
     - If this state is a consistent state with a default action, then
       the only way this function was invoked is if the default action
       is an error action.  In that case, don't check for expected
       tokens because there are none.
     - The only way there can be no lookahead present (in yychar) is if
       this state is a consistent state with a default action.  Thus,
       detecting the absence of a lookahead is sufficient to determine
       that there is no unexpected or expected token to report.  In that
       case, just report a simple "syntax error".
     - Don't assume there isn't a lookahead just because this state is a
       consistent state with a default action.  There might have been a
       previous inconsistent state, consistent state with a non-default
       action, or user semantic action that manipulated yychar.
     - Of course, the expected token list depends on states to have
       correct lookahead information, and it depends on the parser not
       to perform extra reductions after fetching a lookahead from the
       scanner and before detecting a syntax error.  Thus, state merging
       (from LALR or IELR) and default reductions corrupt the expected
       token list.  However, the list is correct for canonical LR with
       one exception: it will still contain any token that will not be
       accepted due to an error action in a later state.
  */
  if (yytoken != YYEMPTY)
    {
      int yyn = yypact[*yyssp];
      yyarg[yycount++] = yytname[yytoken];
      if (!yypact_value_is_default (yyn))
        {
          /* Start YYX at -YYN if negative to avoid negative indexes in
             YYCHECK.  In other words, skip the first -YYN actions for
             this state because they are default actions.  */
          int yyxbegin = yyn < 0 ? -yyn : 0;
          /* Stay within bounds of both yycheck and yytname.  */
          int yychecklim = YYLAST - yyn + 1;
          int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
          int yyx;

          for (yyx = yyxbegin; yyx < yyxend; ++yyx)
            if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR
                && !yytable_value_is_error (yytable[yyx + yyn]))
              {
                if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                  {
                    yycount = 1;
                    yysize = yysize0;
                    break;
                  }
                yyarg[yycount++] = yytname[yyx];
                {
                  YYSIZE_T yysize1 = yysize + yytnamerr (YY_NULLPTR, yytname[yyx]);
                  if (! (yysize <= yysize1
                         && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
                    return 2;
                  yysize = yysize1;
                }
              }
        }
    }

  switch (yycount)
    {
# define YYCASE_(N, S)                      \
      case N:                               \
        yyformat = S;                       \
      break
      YYCASE_(0, YY_("syntax error"));
      YYCASE_(1, YY_("syntax error, unexpected %s"));
      YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
      YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
      YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
      YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
# undef YYCASE_
    }

  {
    YYSIZE_T yysize1 = yysize + yystrlen (yyformat);
    if (! (yysize <= yysize1 && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
      return 2;
    yysize = yysize1;
  }

  if (*yymsg_alloc < yysize)
    {
      *yymsg_alloc = 2 * yysize;
      if (! (yysize <= *yymsg_alloc
             && *yymsg_alloc <= YYSTACK_ALLOC_MAXIMUM))
        *yymsg_alloc = YYSTACK_ALLOC_MAXIMUM;
      return 1;
    }

  /* Avoid sprintf, as that infringes on the user's name space.
     Don't have undefined behavior even if the translation
     produced a string with the wrong number of "%s"s.  */
  {
    char *yyp = *yymsg;
    int yyi = 0;
    while ((*yyp = *yyformat) != '\0')
      if (*yyp == '%' && yyformat[1] == 's' && yyi < yycount)
        {
          yyp += yytnamerr (yyp, yyarg[yyi++]);
          yyformat += 2;
        }
      else
        {
          yyp++;
          yyformat++;
        }
  }
  return 0;
}
#endif /* YYERROR_VERBOSE */

/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep, struct parse_param* p)
{
  YYUSE (yyvaluep);
  YYUSE (p);
  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YYUSE (yytype);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}




/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;
/* Number of syntax errors so far.  */
int yynerrs;


/*----------.
| yyparse.  |
`----------*/

int
yyparse (struct parse_param* p)
{
    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       'yyss': related to states.
       'yyvs': related to semantic values.

       Refer to the stacks through separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken = 0;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yyssp = yyss = yyssa;
  yyvsp = yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */
  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
        /* Give user a chance to reallocate the stack.  Use copies of
           these so that the &'s don't force the real ones into
           memory.  */
        YYSTYPE *yyvs1 = yyvs;
        yytype_int16 *yyss1 = yyss;

        /* Each stack pointer address is followed by the size of the
           data in use in that stack, in bytes.  This used to be a
           conditional around just the two extra args, but that might
           be undefined if yyoverflow is a macro.  */
        yyoverflow (YY_("memory exhausted"),
                    &yyss1, yysize * sizeof (*yyssp),
                    &yyvs1, yysize * sizeof (*yyvsp),
                    &yystacksize);

        yyss = yyss1;
        yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
        goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
        yystacksize = YYMAXDEPTH;

      {
        yytype_int16 *yyss1 = yyss;
        union yyalloc *yyptr =
          (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
        if (! yyptr)
          goto yyexhaustedlab;
        YYSTACK_RELOCATE (yyss_alloc, yyss);
        YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
        if (yyss1 != yyssa)
          YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
                  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
        YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = yylex ();
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
        goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     '$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 4:
#line 76 "dfg-parser.y" /* yacc.c:1646  */
    {
        if(p->symbols.has_sym((yyvsp[-1].io_pair)->first)) {
          printf("Symbol \"%s\" already exists\n", (yyvsp[-1].io_pair)->first.c_str());
          assert(0);
        }
        p->dfg->addVecInput((yyvsp[-1].io_pair)->first, (yyvsp[-1].io_pair)->second, p->symbols, (yyvsp[-3].i), p->meta);
        p->meta.clear();
        delete (yyvsp[-1].io_pair);
      }
#line 1324 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 5:
#line 85 "dfg-parser.y" /* yacc.c:1646  */
    {
        p->dfg->addVecOutput((yyvsp[-1].io_pair)->first.c_str(), (yyvsp[-1].io_pair)->second, p->symbols, (yyvsp[-3].i), p->meta);
        p->meta.clear();
        delete (yyvsp[-1].io_pair);
      }
#line 1334 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 6:
#line 90 "dfg-parser.y" /* yacc.c:1646  */
    {
        ParseResult *s = (yyvsp[-1].sym_ent);
        if (auto ne = dynamic_cast<ValueEntry*>((yyvsp[-1].sym_ent))) {
          SSDfgNode* node = ne->value->node();
          assert((yyvsp[-3].str_vec)->size()==1);
          std::string name = (*(yyvsp[-3].str_vec))[0];
          p->symbols.set(name, new ValueEntry(ne->value, ne->l, ne->r));
        } else if (auto ce = dynamic_cast<ConvergeEntry*>((yyvsp[-1].sym_ent))) {
          //By definition, converge entries only need one symbol (just def)
          if ((yyvsp[-3].str_vec)->size()==1) {
            std::string name = (*(yyvsp[-3].str_vec))[0];
            for (auto elem : ce->entries) {
              auto node = elem->value->node();
              if(!node->has_name()) node->set_name(name);
            }
            p->symbols.set(name, (yyvsp[-1].sym_ent));
          } else if ((yyvsp[-3].str_vec)->size() == ce->entries.size()) {
            auto o = dynamic_cast<ValueEntry*>(ce->entries[0]);
            assert(o && "Assumption check failure, the first element should be a value entry!");
            assert(dynamic_cast<SSDfgInst*>(o->value->node()) && "Should be a instruction!");
            p->symbols.set((*(yyvsp[-3].str_vec))[0], o->value);
            for (int i = 1; i < ce->entries.size(); ++i) {
              if (auto ve = dynamic_cast<ValueEntry*>(ce->entries[i])) {
                assert(ve->value->node() == o->value->node() && "Should all from the same node!");
                p->symbols.set((*(yyvsp[-3].str_vec))[i], ve->value);
              } else {
                assert(false && "Assumption check failure! All the remaining element should also be a value entry!");
              }
            }
          }
        }
        delete (yyvsp[-3].str_vec);
      }
#line 1372 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 7:
#line 123 "dfg-parser.y" /* yacc.c:1646  */
    {
        p->dfg->start_new_dfg_group();
      }
#line 1380 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 8:
#line 126 "dfg-parser.y" /* yacc.c:1646  */
    {
        p->dfg->set_pragma(*(yyvsp[-2].s),*(yyvsp[-1].s));
        delete (yyvsp[-2].s);
        delete (yyvsp[-1].s);
      }
#line 1390 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 9:
#line 131 "dfg-parser.y" /* yacc.c:1646  */
    {
        assert(*(yyvsp[-3].s) == "group");
        std::ostringstream oss;
        oss << (yyvsp[-1].i);
        p->dfg->set_pragma(*(yyvsp[-2].s), oss.str());
        delete((yyvsp[-3].s));
        delete((yyvsp[-2].s));
      }
#line 1403 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 10:
#line 139 "dfg-parser.y" /* yacc.c:1646  */
    {
        p->meta.set(*(yyvsp[-3].s), *(yyvsp[-1].s));
      }
#line 1411 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 11:
#line 142 "dfg-parser.y" /* yacc.c:1646  */
    {
        std::ostringstream oss;
        oss << (yyvsp[-1].i);
        p->meta.set(*(yyvsp[-3].s), oss.str());
      }
#line 1421 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 12:
#line 147 "dfg-parser.y" /* yacc.c:1646  */
    {
        std::ostringstream oss;
        oss << (yyvsp[-1].d);
        p->meta.set(*(yyvsp[-3].s), oss.str());
      }
#line 1431 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 16:
#line 160 "dfg-parser.y" /* yacc.c:1646  */
    {
	    (yyval.io_pair) = new io_pair_t(*(yyvsp[0].s),0);
	    delete (yyvsp[0].s);
	  }
#line 1440 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 17:
#line 164 "dfg-parser.y" /* yacc.c:1646  */
    {
	    (yyval.io_pair) = new io_pair_t(*(yyvsp[-3].s),(yyvsp[-1].i));
	    delete (yyvsp[-3].s);
	  }
#line 1449 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 18:
#line 170 "dfg-parser.y" /* yacc.c:1646  */
    { (yyval.sym_ent) = (yyvsp[0].sym_ent); }
#line 1455 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 19:
#line 173 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_ent) = new ConstDataEntry((yyvsp[0].i)); //expr: what you can assign to a var
      }
#line 1463 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 20:
#line 176 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_ent) = new ConstDataEntry((yyvsp[-1].i), (yyvsp[0].i));
      }
#line 1471 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 21:
#line 179 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_ent) = new ConstDataEntry((yyvsp[-3].i), (yyvsp[-2].i), (yyvsp[-1].i), (yyvsp[0].i));
      }
#line 1479 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 22:
#line 182 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_ent) = new ConstDataEntry((yyvsp[0].d));
      }
#line 1487 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 23:
#line 185 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_ent) = new ConstDataEntry((float) (yyvsp[-1].d), (float) (yyvsp[0].d));
      }
#line 1495 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 24:
#line 188 "dfg-parser.y" /* yacc.c:1646  */
    {
        auto ce = dynamic_cast<ConvergeEntry*>((yyvsp[0].sym_ent));
        assert(ce);
        if (ce->entries.size() == 1) {
          (yyval.sym_ent) = ce->entries[0];
          delete ce;
        } else {
          (yyval.sym_ent) = (yyvsp[0].sym_ent);
        }
      }
#line 1510 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 25:
#line 198 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_ent) = p->dfg->create_inst(*(yyvsp[-3].s),*(yyvsp[-1].sym_vec));
        delete (yyvsp[-3].s);
        delete (yyvsp[-1].sym_vec);
      }
#line 1520 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 26:
#line 208 "dfg-parser.y" /* yacc.c:1646  */
    {
	    (yyval.sym_ent) = (yyvsp[0].sym_ent);
	  }
#line 1528 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 27:
#line 211 "dfg-parser.y" /* yacc.c:1646  */
    {
	    (yyval.sym_ent) = new ControlEntry(*(yyvsp[-2].s), (yyvsp[0].sym_ent));
	  }
#line 1536 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 28:
#line 214 "dfg-parser.y" /* yacc.c:1646  */
    {
            (yyval.sym_ent) = new ControlEntry(*(yyvsp[-5].s), *(yyvsp[-1].ctrl_def), (yyvsp[-3].sym_ent));
            delete (yyvsp[-1].ctrl_def);
          }
#line 1545 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 29:
#line 218 "dfg-parser.y" /* yacc.c:1646  */
    {
            (yyval.sym_ent) = new ControlEntry(*(yyvsp[-4].s), *(yyvsp[-1].ctrl_def), nullptr);
            delete (yyvsp[-1].ctrl_def);
          }
#line 1554 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 30:
#line 225 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_vec) = new std::vector<ParseResult*>();
        (yyval.sym_vec)->push_back((yyvsp[0].sym_ent));
      }
#line 1563 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 31:
#line 229 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyvsp[-2].sym_vec)->push_back((yyvsp[0].sym_ent));
        (yyval.sym_vec) = (yyvsp[-2].sym_vec);
      }
#line 1572 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 32:
#line 236 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.ctrl_def) = new ctrl_def_t();
        (*(yyval.ctrl_def))[(yyvsp[-2].i)]=*(yyvsp[0].str_vec); delete (yyvsp[0].str_vec);
      }
#line 1581 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 33:
#line 240 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.ctrl_def) = (yyvsp[-4].ctrl_def);
        (*(yyval.ctrl_def))[(yyvsp[-2].i)]=*(yyvsp[0].str_vec); delete (yyvsp[0].str_vec);
      }
#line 1590 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 34:
#line 247 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.str_vec) = new string_vec_t(); (yyval.str_vec)->push_back(std::string(*(yyvsp[0].s)));
      }
#line 1598 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 35:
#line 250 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyvsp[-2].str_vec)->push_back(*(yyvsp[0].s)); (yyval.str_vec) = (yyvsp[-2].str_vec);
      }
#line 1606 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 36:
#line 256 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.str_vec) = new string_vec_t(); (yyval.str_vec)->push_back(std::string(*(yyvsp[0].s)));
      }
#line 1614 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 37:
#line 259 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyvsp[-2].str_vec)->push_back(*(yyvsp[0].s)); (yyval.str_vec) = (yyvsp[-2].str_vec);
      }
#line 1622 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 38:
#line 265 "dfg-parser.y" /* yacc.c:1646  */
    {
        auto res = new ConvergeEntry(); 
        if (auto ne = dynamic_cast<ValueEntry*>((yyvsp[0].sym_ent))) {
          res->entries.push_back(ne);
        } else if (auto ce = dynamic_cast<ConvergeEntry*>((yyvsp[0].sym_ent))) {
          res->entries = ce->entries;
        } else {
          assert(0 && "Not supported edge list type!");
        }
        (yyval.sym_ent) = res;
      }
#line 1638 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 39:
#line 276 "dfg-parser.y" /* yacc.c:1646  */
    {
        auto res = dynamic_cast<ConvergeEntry*>((yyvsp[-1].sym_ent));
        assert(res);
        if (auto ne = dynamic_cast<ValueEntry*>((yyvsp[0].sym_ent))) {
          res->entries.insert(res->entries.begin(), ne);
        } else if (auto ce = dynamic_cast<ConvergeEntry*>((yyvsp[-1].sym_ent))) {
          res->entries.insert(res->entries.begin(), ce->entries.begin(), ce->entries.end());
        } else {
          assert(0 && "Not supported edge list type!");
        }
        (yyval.sym_ent) = res;
      }
#line 1655 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 40:
#line 292 "dfg-parser.y" /* yacc.c:1646  */
    {
        (yyval.sym_ent) = p->symbols.get_sym(*(yyvsp[0].s));
        auto res = p->symbols.get_sym(*(yyvsp[0].s));
        delete (yyvsp[0].s);
      }
#line 1665 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;

  case 41:
#line 297 "dfg-parser.y" /* yacc.c:1646  */
    {
        auto ne = dynamic_cast<ValueEntry*>(p->symbols.get_sym(*(yyvsp[-4].s)));
        assert(ne && "For now only result values supports slicing!");
        if (ne->l == (yyvsp[-2].i) && ne->r == (yyvsp[0].i)) {
          (yyval.sym_ent) = ne;
        } else {
          std::ostringstream oss;
          oss << *(yyvsp[-4].s) << ':' << (yyvsp[-2].i) << ':' << (yyvsp[0].i);
          if (!p->symbols.has_sym(oss.str())) {
            p->symbols.set(oss.str(), new ValueEntry(ne->value, (yyvsp[-2].i), (yyvsp[0].i)));
          }
          (yyval.sym_ent) = p->symbols.get_sym(oss.str());
        }
        delete (yyvsp[-4].s);
      }
#line 1685 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
    break;


#line 1689 "/home/sihao/repo/ss-stack/ss-scheduler/src/ss-scheduler/dfg-parser.tab.cc" /* yacc.c:1646  */
      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;

  /* Now 'shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*--------------------------------------.
| yyerrlab -- here on detecting error.  |
`--------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYEMPTY : YYTRANSLATE (yychar);

  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (p, YY_("syntax error"));
#else
# define YYSYNTAX_ERROR yysyntax_error (&yymsg_alloc, &yymsg, \
                                        yyssp, yytoken)
      {
        char const *yymsgp = YY_("syntax error");
        int yysyntax_error_status;
        yysyntax_error_status = YYSYNTAX_ERROR;
        if (yysyntax_error_status == 0)
          yymsgp = yymsg;
        else if (yysyntax_error_status == 1)
          {
            if (yymsg != yymsgbuf)
              YYSTACK_FREE (yymsg);
            yymsg = (char *) YYSTACK_ALLOC (yymsg_alloc);
            if (!yymsg)
              {
                yymsg = yymsgbuf;
                yymsg_alloc = sizeof yymsgbuf;
                yysyntax_error_status = 2;
              }
            else
              {
                yysyntax_error_status = YYSYNTAX_ERROR;
                yymsgp = yymsg;
              }
          }
        yyerror (p, yymsgp);
        if (yysyntax_error_status == 2)
          goto yyexhaustedlab;
      }
# undef YYSYNTAX_ERROR
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
         error, discard it.  */

      if (yychar <= YYEOF)
        {
          /* Return failure if at end of input.  */
          if (yychar == YYEOF)
            YYABORT;
        }
      else
        {
          yydestruct ("Error: discarding",
                      yytoken, &yylval, p);
          yychar = YYEMPTY;
        }
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule whose action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;      /* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
        {
          yyn += YYTERROR;
          if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
            {
              yyn = yytable[yyn];
              if (0 < yyn)
                break;
            }
        }

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
        YYABORT;


      yydestruct ("Error: popping",
                  yystos[yystate], yyvsp, p);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined yyoverflow || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (p, YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval, p);
    }
  /* Do not reclaim the symbols of the rule whose action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
                  yystos[*yyssp], yyvsp, p);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  return yyresult;
}
#line 314 "dfg-parser.y" /* yacc.c:1906  */




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
