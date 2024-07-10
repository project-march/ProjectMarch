/* A Bison parser, made by GNU Bison 3.8.2.  */

/* Bison implementation for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018-2021 Free Software Foundation,
   Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.  */

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

/* DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
   especially those whose name start with YY_ or yy_.  They are
   private implementation details that can be changed or removed.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output, and Bison version.  */
#define YYBISON 30802

/* Bison version string.  */
#define YYBISON_VERSION "3.8.2"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1




/* First part of user prologue.  */
#line 29 "ginsh_parser.ypp"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#ifdef HAVE_RUSAGE
#include <sys/resource.h>
#else
#include <ctime>
#endif

#ifdef HAVE_UNISTD_H
#include <sys/types.h>
#include <unistd.h>
#endif

#include <stdexcept>

#include "ginsh.h"

using namespace std;
using namespace GiNaC;

#define YYERROR_VERBOSE 1

#ifdef HAVE_LIBREADLINE
// Original readline settings
static int orig_completion_append_character;
static const char *orig_basic_word_break_characters;

#if (RL_VERSION_MAJOR >= 5)
#define GINAC_RL_COMPLETER_CAST(a) const_cast<char *>((a))
#else
#define GINAC_RL_COMPLETER_CAST(a) (a)
#endif
#endif // HAVE_LIBREADLINE

// Expression stack for %, %% and %%%
static void push(const ex &e);
static ex exstack[3];
// Assigned symbols
static exmap assigned_symbol_table;

// Start and end time for the time() function
#ifdef HAVE_RUSAGE
static struct rusage start_time, end_time;
#define START_TIMER getrusage(RUSAGE_SELF, &start_time);
#define STOP_TIMER getrusage(RUSAGE_SELF, &end_time);
#define PRINT_TIME_USED cout << \
   (end_time.ru_utime.tv_sec - start_time.ru_utime.tv_sec) + \
       (end_time.ru_stime.tv_sec - start_time.ru_stime.tv_sec) + \
       double(end_time.ru_utime.tv_usec - start_time.ru_utime.tv_usec) / 1e6 + \
       double(end_time.ru_stime.tv_usec - start_time.ru_stime.tv_usec) / 1e6 \
                       << 's' << endl;
#else
static std::clock_t start_time, end_time;
#define START_TIMER start_time = std::clock();
#define STOP_TIMER end_time = std::clock();
#define PRINT_TIME_USED \
  cout << double(end_time - start_time)/CLOCKS_PER_SEC << 's' << endl;
#endif

// Table of functions (a multimap, because one function may appear with different
// numbers of parameters)
typedef ex (*fcnp)(const exprseq &e);
typedef ex (*fcnp2)(const exprseq &e, int serial);

struct fcn_desc {
	fcn_desc() : p(nullptr), num_params(0), is_ginac(false), serial(0) {}
	fcn_desc(fcnp func, int num) : p(func), num_params(num), is_ginac(false), serial(0) {}
	fcn_desc(fcnp2 func, int num, int ser) : p((fcnp)func), num_params(num), is_ginac(true), serial(ser) {}

	fcnp p;		// Pointer to function
	int num_params;	// Number of parameters (0 = arbitrary)
	bool is_ginac;	// Flag: function is GiNaC function
	int serial;	// GiNaC function serial number (if is_ginac == true)
};

typedef multimap<string, fcn_desc> fcn_tab;
static fcn_tab fcns;

static fcn_tab::const_iterator find_function(const ex &sym, int req_params);

// Table to map help topics to help strings
typedef multimap<string, string> help_tab;
static help_tab help;

static void insert_fcn_help(const char *name, const char *str);
static void print_help(const string &topic);
static void print_help_topics(void);

#line 162 "ginsh_parser.cpp"

# ifndef YY_CAST
#  ifdef __cplusplus
#   define YY_CAST(Type, Val) static_cast<Type> (Val)
#   define YY_REINTERPRET_CAST(Type, Val) reinterpret_cast<Type> (Val)
#  else
#   define YY_CAST(Type, Val) ((Type) (Val))
#   define YY_REINTERPRET_CAST(Type, Val) ((Type) (Val))
#  endif
# endif
# ifndef YY_NULLPTR
#  if defined __cplusplus
#   if 201103L <= __cplusplus
#    define YY_NULLPTR nullptr
#   else
#    define YY_NULLPTR 0
#   endif
#  else
#   define YY_NULLPTR ((void*)0)
#  endif
# endif

#include "ginsh_parser.hpp"
/* Symbol kind.  */
enum yysymbol_kind_t
{
  YYSYMBOL_YYEMPTY = -2,
  YYSYMBOL_YYEOF = 0,                      /* "end of file"  */
  YYSYMBOL_YYerror = 1,                    /* error  */
  YYSYMBOL_YYUNDEF = 2,                    /* "invalid token"  */
  YYSYMBOL_T_NUMBER = 3,                   /* T_NUMBER  */
  YYSYMBOL_T_SYMBOL = 4,                   /* T_SYMBOL  */
  YYSYMBOL_T_LITERAL = 5,                  /* T_LITERAL  */
  YYSYMBOL_T_DIGITS = 6,                   /* T_DIGITS  */
  YYSYMBOL_T_QUOTE = 7,                    /* T_QUOTE  */
  YYSYMBOL_T_QUOTE2 = 8,                   /* T_QUOTE2  */
  YYSYMBOL_T_QUOTE3 = 9,                   /* T_QUOTE3  */
  YYSYMBOL_T_EQUAL = 10,                   /* T_EQUAL  */
  YYSYMBOL_T_NOTEQ = 11,                   /* T_NOTEQ  */
  YYSYMBOL_T_LESSEQ = 12,                  /* T_LESSEQ  */
  YYSYMBOL_T_GREATEREQ = 13,               /* T_GREATEREQ  */
  YYSYMBOL_T_QUIT = 14,                    /* T_QUIT  */
  YYSYMBOL_T_WARRANTY = 15,                /* T_WARRANTY  */
  YYSYMBOL_T_PRINT = 16,                   /* T_PRINT  */
  YYSYMBOL_T_IPRINT = 17,                  /* T_IPRINT  */
  YYSYMBOL_T_PRINTLATEX = 18,              /* T_PRINTLATEX  */
  YYSYMBOL_T_PRINTCSRC = 19,               /* T_PRINTCSRC  */
  YYSYMBOL_T_TIME = 20,                    /* T_TIME  */
  YYSYMBOL_T_XYZZY = 21,                   /* T_XYZZY  */
  YYSYMBOL_T_INVENTORY = 22,               /* T_INVENTORY  */
  YYSYMBOL_T_LOOK = 23,                    /* T_LOOK  */
  YYSYMBOL_T_SCORE = 24,                   /* T_SCORE  */
  YYSYMBOL_T_COMPLEX_SYMBOLS = 25,         /* T_COMPLEX_SYMBOLS  */
  YYSYMBOL_T_REAL_SYMBOLS = 26,            /* T_REAL_SYMBOLS  */
  YYSYMBOL_27_ = 27,                       /* '='  */
  YYSYMBOL_28_ = 28,                       /* '<'  */
  YYSYMBOL_29_ = 29,                       /* '>'  */
  YYSYMBOL_30_ = 30,                       /* '+'  */
  YYSYMBOL_31_ = 31,                       /* '-'  */
  YYSYMBOL_32_ = 32,                       /* '*'  */
  YYSYMBOL_33_ = 33,                       /* '/'  */
  YYSYMBOL_NEG = 34,                       /* NEG  */
  YYSYMBOL_35_ = 35,                       /* '^'  */
  YYSYMBOL_36_ = 36,                       /* '!'  */
  YYSYMBOL_37_ = 37,                       /* ';'  */
  YYSYMBOL_38_ = 38,                       /* ':'  */
  YYSYMBOL_39_ = 39,                       /* '('  */
  YYSYMBOL_40_ = 40,                       /* ')'  */
  YYSYMBOL_41_ = 41,                       /* '?'  */
  YYSYMBOL_42_ = 42,                       /* '\''  */
  YYSYMBOL_43_ = 43,                       /* '{'  */
  YYSYMBOL_44_ = 44,                       /* '}'  */
  YYSYMBOL_45_ = 45,                       /* '['  */
  YYSYMBOL_46_ = 46,                       /* ']'  */
  YYSYMBOL_47_ = 47,                       /* ','  */
  YYSYMBOL_YYACCEPT = 48,                  /* $accept  */
  YYSYMBOL_input = 49,                     /* input  */
  YYSYMBOL_line = 50,                      /* line  */
  YYSYMBOL_51_1 = 51,                      /* $@1  */
  YYSYMBOL_exp = 52,                       /* exp  */
  YYSYMBOL_exprseq = 53,                   /* exprseq  */
  YYSYMBOL_list_or_empty = 54,             /* list_or_empty  */
  YYSYMBOL_list = 55,                      /* list  */
  YYSYMBOL_matrix = 56,                    /* matrix  */
  YYSYMBOL_row = 57                        /* row  */
};
typedef enum yysymbol_kind_t yysymbol_kind_t;




#ifdef short
# undef short
#endif

/* On compilers that do not define __PTRDIFF_MAX__ etc., make sure
   <limits.h> and (if available) <stdint.h> are included
   so that the code can choose integer types of a good width.  */

#ifndef __PTRDIFF_MAX__
# include <limits.h> /* INFRINGES ON USER NAME SPACE */
# if defined __STDC_VERSION__ && 199901 <= __STDC_VERSION__
#  include <stdint.h> /* INFRINGES ON USER NAME SPACE */
#  define YY_STDINT_H
# endif
#endif

/* Narrow types that promote to a signed type and that can represent a
   signed or unsigned integer of at least N bits.  In tables they can
   save space and decrease cache pressure.  Promoting to a signed type
   helps avoid bugs in integer arithmetic.  */

#ifdef __INT_LEAST8_MAX__
typedef __INT_LEAST8_TYPE__ yytype_int8;
#elif defined YY_STDINT_H
typedef int_least8_t yytype_int8;
#else
typedef signed char yytype_int8;
#endif

#ifdef __INT_LEAST16_MAX__
typedef __INT_LEAST16_TYPE__ yytype_int16;
#elif defined YY_STDINT_H
typedef int_least16_t yytype_int16;
#else
typedef short yytype_int16;
#endif

/* Work around bug in HP-UX 11.23, which defines these macros
   incorrectly for preprocessor constants.  This workaround can likely
   be removed in 2023, as HPE has promised support for HP-UX 11.23
   (aka HP-UX 11i v2) only through the end of 2022; see Table 2 of
   <https://h20195.www2.hpe.com/V2/getpdf.aspx/4AA4-7673ENW.pdf>.  */
#ifdef __hpux
# undef UINT_LEAST8_MAX
# undef UINT_LEAST16_MAX
# define UINT_LEAST8_MAX 255
# define UINT_LEAST16_MAX 65535
#endif

#if defined __UINT_LEAST8_MAX__ && __UINT_LEAST8_MAX__ <= __INT_MAX__
typedef __UINT_LEAST8_TYPE__ yytype_uint8;
#elif (!defined __UINT_LEAST8_MAX__ && defined YY_STDINT_H \
       && UINT_LEAST8_MAX <= INT_MAX)
typedef uint_least8_t yytype_uint8;
#elif !defined __UINT_LEAST8_MAX__ && UCHAR_MAX <= INT_MAX
typedef unsigned char yytype_uint8;
#else
typedef short yytype_uint8;
#endif

#if defined __UINT_LEAST16_MAX__ && __UINT_LEAST16_MAX__ <= __INT_MAX__
typedef __UINT_LEAST16_TYPE__ yytype_uint16;
#elif (!defined __UINT_LEAST16_MAX__ && defined YY_STDINT_H \
       && UINT_LEAST16_MAX <= INT_MAX)
typedef uint_least16_t yytype_uint16;
#elif !defined __UINT_LEAST16_MAX__ && USHRT_MAX <= INT_MAX
typedef unsigned short yytype_uint16;
#else
typedef int yytype_uint16;
#endif

#ifndef YYPTRDIFF_T
# if defined __PTRDIFF_TYPE__ && defined __PTRDIFF_MAX__
#  define YYPTRDIFF_T __PTRDIFF_TYPE__
#  define YYPTRDIFF_MAXIMUM __PTRDIFF_MAX__
# elif defined PTRDIFF_MAX
#  ifndef ptrdiff_t
#   include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  endif
#  define YYPTRDIFF_T ptrdiff_t
#  define YYPTRDIFF_MAXIMUM PTRDIFF_MAX
# else
#  define YYPTRDIFF_T long
#  define YYPTRDIFF_MAXIMUM LONG_MAX
# endif
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif defined __STDC_VERSION__ && 199901 <= __STDC_VERSION__
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned
# endif
#endif

#define YYSIZE_MAXIMUM                                  \
  YY_CAST (YYPTRDIFF_T,                                 \
           (YYPTRDIFF_MAXIMUM < YY_CAST (YYSIZE_T, -1)  \
            ? YYPTRDIFF_MAXIMUM                         \
            : YY_CAST (YYSIZE_T, -1)))

#define YYSIZEOF(X) YY_CAST (YYPTRDIFF_T, sizeof (X))


/* Stored state numbers (used for stacks). */
typedef yytype_int8 yy_state_t;

/* State numbers in computations.  */
typedef int yy_state_fast_t;

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


#ifndef YY_ATTRIBUTE_PURE
# if defined __GNUC__ && 2 < __GNUC__ + (96 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_PURE __attribute__ ((__pure__))
# else
#  define YY_ATTRIBUTE_PURE
# endif
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# if defined __GNUC__ && 2 < __GNUC__ + (7 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_UNUSED __attribute__ ((__unused__))
# else
#  define YY_ATTRIBUTE_UNUSED
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YY_USE(E) ((void) (E))
#else
# define YY_USE(E) /* empty */
#endif

/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
#if defined __GNUC__ && ! defined __ICC && 406 <= __GNUC__ * 100 + __GNUC_MINOR__
# if __GNUC__ * 100 + __GNUC_MINOR__ < 407
#  define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN                           \
    _Pragma ("GCC diagnostic push")                                     \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")
# else
#  define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN                           \
    _Pragma ("GCC diagnostic push")                                     \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")              \
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# endif
# define YY_IGNORE_MAYBE_UNINITIALIZED_END      \
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

#if defined __cplusplus && defined __GNUC__ && ! defined __ICC && 6 <= __GNUC__
# define YY_IGNORE_USELESS_CAST_BEGIN                          \
    _Pragma ("GCC diagnostic push")                            \
    _Pragma ("GCC diagnostic ignored \"-Wuseless-cast\"")
# define YY_IGNORE_USELESS_CAST_END            \
    _Pragma ("GCC diagnostic pop")
#endif
#ifndef YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_END
#endif


#define YY_ASSERT(E) ((void) (0 && (E)))

#if !defined yyoverflow

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
#endif /* !defined yyoverflow */

#if (! defined yyoverflow \
     && (! defined __cplusplus \
         || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yy_state_t yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (YYSIZEOF (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (YYSIZEOF (yy_state_t) + YYSIZEOF (YYSTYPE)) \
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
        YYPTRDIFF_T yynewbytes;                                         \
        YYCOPY (&yyptr->Stack_alloc, Stack, yysize);                    \
        Stack = &yyptr->Stack_alloc;                                    \
        yynewbytes = yystacksize * YYSIZEOF (*Stack) + YYSTACK_GAP_MAXIMUM; \
        yyptr += yynewbytes / YYSIZEOF (*yyptr);                        \
      }                                                                 \
    while (0)

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from SRC to DST.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(Dst, Src, Count) \
      __builtin_memcpy (Dst, Src, YY_CAST (YYSIZE_T, (Count)) * sizeof (*(Src)))
#  else
#   define YYCOPY(Dst, Src, Count)              \
      do                                        \
        {                                       \
          YYPTRDIFF_T yyi;                      \
          for (yyi = 0; yyi < (Count); yyi++)   \
            (Dst)[yyi] = (Src)[yyi];            \
        }                                       \
      while (0)
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  2
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   300

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  48
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  10
/* YYNRULES -- Number of rules.  */
#define YYNRULES  67
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  122

/* YYMAXUTOK -- Last valid token kind.  */
#define YYMAXUTOK   282


/* YYTRANSLATE(TOKEN-NUM) -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, with out-of-bounds checking.  */
#define YYTRANSLATE(YYX)                                \
  (0 <= (YYX) && (YYX) <= YYMAXUTOK                     \
   ? YY_CAST (yysymbol_kind_t, yytranslate[YYX])        \
   : YYSYMBOL_YYUNDEF)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex.  */
static const yytype_int8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    36,     2,     2,     2,     2,     2,    42,
      39,    40,    32,    30,    47,    31,     2,    33,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,    38,    37,
      28,    27,    29,    41,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,    45,     2,    46,    35,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    43,     2,    44,     2,     2,     2,     2,
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
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    34
};

#if YYDEBUG
/* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_int16 yyrline[] =
{
       0,   146,   146,   147,   150,   151,   160,   168,   176,   190,
     198,   206,   207,   208,   209,   210,   211,   212,   213,   214,
     227,   228,   229,   230,   235,   236,   237,   237,   238,   239,
     242,   243,   250,   251,   252,   253,   254,   255,   256,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   285,   286,
     289,   290,   293,   294,   297,   298,   301,   302
};
#endif

/** Accessing symbol of state STATE.  */
#define YY_ACCESSING_SYMBOL(State) YY_CAST (yysymbol_kind_t, yystos[State])

#if YYDEBUG || 0
/* The user-facing name of the symbol whose (internal) number is
   YYSYMBOL.  No bounds checking.  */
static const char *yysymbol_name (yysymbol_kind_t yysymbol) YY_ATTRIBUTE_UNUSED;

/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "\"end of file\"", "error", "\"invalid token\"", "T_NUMBER", "T_SYMBOL",
  "T_LITERAL", "T_DIGITS", "T_QUOTE", "T_QUOTE2", "T_QUOTE3", "T_EQUAL",
  "T_NOTEQ", "T_LESSEQ", "T_GREATEREQ", "T_QUIT", "T_WARRANTY", "T_PRINT",
  "T_IPRINT", "T_PRINTLATEX", "T_PRINTCSRC", "T_TIME", "T_XYZZY",
  "T_INVENTORY", "T_LOOK", "T_SCORE", "T_COMPLEX_SYMBOLS",
  "T_REAL_SYMBOLS", "'='", "'<'", "'>'", "'+'", "'-'", "'*'", "'/'", "NEG",
  "'^'", "'!'", "';'", "':'", "'('", "')'", "'?'", "'\\''", "'{'", "'}'",
  "'['", "']'", "','", "$accept", "input", "line", "$@1", "exp", "exprseq",
  "list_or_empty", "list", "matrix", "row", YY_NULLPTR
};

static const char *
yysymbol_name (yysymbol_kind_t yysymbol)
{
  return yytname[yysymbol];
}
#endif

#define YYPACT_NINF (-36)

#define yypact_value_is_default(Yyn) \
  ((Yyn) == YYPACT_NINF)

#define YYTABLE_NINF (-1)

#define yytable_value_is_error(Yyn) \
  0

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
static const yytype_int16 yypact[] =
{
     -36,   100,   -36,   -35,   -36,   -26,   -36,   -13,   -36,   -36,
     -36,   -36,   -36,   -21,   -18,   -14,    -9,   -36,   -36,   -36,
     -36,   -36,   -36,   -36,     3,     3,   -36,     3,    33,    43,
       3,     9,   -36,    60,   -36,   -36,     3,     3,    52,     3,
       3,     3,     3,    17,   -31,   -31,   122,   -36,   -36,   -36,
     -36,   -36,   -36,   -36,    15,   255,    14,    28,     3,   -19,
       3,     3,     3,     3,     3,     3,     3,     3,     3,     3,
       3,   -36,   -36,   -36,   255,   255,   -25,   -36,   136,   167,
     180,   211,     3,   -36,   -36,   -36,     3,   255,   -15,   -36,
      31,   264,   264,    47,    47,    47,    47,   -16,   -16,   -31,
     -31,   -31,   -36,     3,    49,    50,    57,    62,   224,   255,
     -36,     3,     3,   255,   -36,   -36,   -36,   -36,   -36,   255,
      -3,   -36
};

/* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
   Performed when YYTABLE does not specify something else to do.  Zero
   means the default is an error.  */
static const yytype_int8 yydefact[] =
{
       2,     0,     1,     0,    30,    31,    33,    34,    35,    36,
      37,    18,    19,     0,     0,     0,     0,    26,    20,    21,
      22,    23,    25,    24,     0,     0,     4,     0,     0,     0,
      60,     0,     3,     0,    28,    29,     0,     0,     0,     0,
       0,     0,     0,     0,    52,    51,     0,    11,    13,    14,
      15,    16,    12,    17,     0,    62,     0,    61,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,    54,     5,     6,    40,    58,     0,    39,     0,     0,
       0,     0,     0,    55,    32,    56,     0,    66,     0,    57,
       0,    41,    42,    44,    46,    43,    45,    47,    48,    49,
      50,    53,    38,     0,     0,     0,     0,     0,     0,    63,
      64,     0,     0,    59,     7,     8,     9,    10,    27,    67,
       0,    65
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
     -36,   -36,   -36,   -36,    -1,   -36,   -36,   -36,   -36,   -28
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int8 yydefgoto[] =
{
       0,     1,    32,    43,    87,    76,    56,    57,    59,    88
};

/* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule whose
   number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_int8 yytable[] =
{
      33,    36,    34,    35,    70,    71,     4,     5,     6,     7,
       8,     9,    10,    37,    38,   102,    68,    69,    39,    70,
      71,    40,   103,    44,    45,    41,    46,    89,    90,    55,
      42,   110,   111,    24,    25,    74,    75,    47,    78,    79,
      80,    81,    27,   121,   111,    29,    30,    54,    31,    48,
      49,    50,    51,    52,    58,    77,    82,    84,    85,    91,
      92,    93,    94,    95,    96,    97,    98,    99,   100,   101,
      60,    61,    62,    63,    53,    86,   112,    66,    67,    68,
      69,   108,    70,    71,   120,   109,   114,   115,    64,    65,
      66,    67,    68,    69,   116,    70,    71,    72,    73,   117,
       2,     3,   113,     4,     5,     6,     7,     8,     9,    10,
     119,     0,     0,     0,    11,    12,    13,    14,    15,    16,
      17,    18,    19,    20,    21,    22,    23,     0,     0,     0,
      24,    25,    60,    61,    62,    63,     0,    26,     0,    27,
       0,    28,    29,    30,     0,    31,    60,    61,    62,    63,
      64,    65,    66,    67,    68,    69,     0,    70,    71,     0,
       0,     0,    83,     0,    64,    65,    66,    67,    68,    69,
       0,    70,    71,     0,     0,     0,   104,    60,    61,    62,
      63,     0,     0,     0,     0,     0,     0,     0,     0,     0,
      60,    61,    62,    63,     0,    64,    65,    66,    67,    68,
      69,     0,    70,    71,     0,     0,     0,   105,    64,    65,
      66,    67,    68,    69,     0,    70,    71,     0,     0,     0,
     106,    60,    61,    62,    63,     0,     0,     0,     0,     0,
       0,     0,     0,     0,    60,    61,    62,    63,     0,    64,
      65,    66,    67,    68,    69,     0,    70,    71,     0,     0,
       0,   107,    64,    65,    66,    67,    68,    69,     0,    70,
      71,     0,     0,     0,   118,    60,    61,    62,    63,     0,
       0,     0,     0,     0,     0,     0,    62,    63,     0,     0,
       0,     0,     0,    64,    65,    66,    67,    68,    69,     0,
      70,    71,    64,    65,    66,    67,    68,    69,     0,    70,
      71
};

static const yytype_int8 yycheck[] =
{
       1,    27,    37,    38,    35,    36,     3,     4,     5,     6,
       7,     8,     9,    39,    27,    40,    32,    33,    39,    35,
      36,    39,    47,    24,    25,    39,    27,    46,    47,    30,
      39,    46,    47,    30,    31,    36,    37,     4,    39,    40,
      41,    42,    39,    46,    47,    42,    43,     4,    45,    16,
      17,    18,    19,    20,    45,     3,    39,    42,    44,    60,
      61,    62,    63,    64,    65,    66,    67,    68,    69,    70,
      10,    11,    12,    13,    41,    47,    45,    30,    31,    32,
      33,    82,    35,    36,   112,    86,    37,    37,    28,    29,
      30,    31,    32,    33,    37,    35,    36,    37,    38,    37,
       0,     1,   103,     3,     4,     5,     6,     7,     8,     9,
     111,    -1,    -1,    -1,    14,    15,    16,    17,    18,    19,
      20,    21,    22,    23,    24,    25,    26,    -1,    -1,    -1,
      30,    31,    10,    11,    12,    13,    -1,    37,    -1,    39,
      -1,    41,    42,    43,    -1,    45,    10,    11,    12,    13,
      28,    29,    30,    31,    32,    33,    -1,    35,    36,    -1,
      -1,    -1,    40,    -1,    28,    29,    30,    31,    32,    33,
      -1,    35,    36,    -1,    -1,    -1,    40,    10,    11,    12,
      13,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
      10,    11,    12,    13,    -1,    28,    29,    30,    31,    32,
      33,    -1,    35,    36,    -1,    -1,    -1,    40,    28,    29,
      30,    31,    32,    33,    -1,    35,    36,    -1,    -1,    -1,
      40,    10,    11,    12,    13,    -1,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    -1,    10,    11,    12,    13,    -1,    28,
      29,    30,    31,    32,    33,    -1,    35,    36,    -1,    -1,
      -1,    40,    28,    29,    30,    31,    32,    33,    -1,    35,
      36,    -1,    -1,    -1,    40,    10,    11,    12,    13,    -1,
      -1,    -1,    -1,    -1,    -1,    -1,    12,    13,    -1,    -1,
      -1,    -1,    -1,    28,    29,    30,    31,    32,    33,    -1,
      35,    36,    28,    29,    30,    31,    32,    33,    -1,    35,
      36
};

/* YYSTOS[STATE-NUM] -- The symbol kind of the accessing symbol of
   state STATE-NUM.  */
static const yytype_int8 yystos[] =
{
       0,    49,     0,     1,     3,     4,     5,     6,     7,     8,
       9,    14,    15,    16,    17,    18,    19,    20,    21,    22,
      23,    24,    25,    26,    30,    31,    37,    39,    41,    42,
      43,    45,    50,    52,    37,    38,    27,    39,    27,    39,
      39,    39,    39,    51,    52,    52,    52,     4,    16,    17,
      18,    19,    20,    41,     4,    52,    54,    55,    45,    56,
      10,    11,    12,    13,    28,    29,    30,    31,    32,    33,
      35,    36,    37,    38,    52,    52,    53,     3,    52,    52,
      52,    52,    39,    40,    42,    44,    47,    52,    57,    46,
      47,    52,    52,    52,    52,    52,    52,    52,    52,    52,
      52,    52,    40,    47,    40,    40,    40,    40,    52,    52,
      46,    47,    45,    52,    37,    37,    37,    37,    40,    52,
      57,    46
};

/* YYR1[RULE-NUM] -- Symbol kind of the left-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr1[] =
{
       0,    48,    49,    49,    50,    50,    50,    50,    50,    50,
      50,    50,    50,    50,    50,    50,    50,    50,    50,    50,
      50,    50,    50,    50,    50,    50,    51,    50,    50,    50,
      52,    52,    52,    52,    52,    52,    52,    52,    52,    52,
      52,    52,    52,    52,    52,    52,    52,    52,    52,    52,
      52,    52,    52,    52,    52,    52,    52,    52,    53,    53,
      54,    54,    55,    55,    56,    56,    57,    57
};

/* YYR2[RULE-NUM] -- Number of symbols on the right-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr2[] =
{
       0,     2,     0,     2,     1,     2,     2,     5,     5,     5,
       5,     2,     2,     2,     2,     2,     2,     2,     1,     1,
       1,     1,     1,     1,     1,     1,     0,     5,     2,     2,
       1,     1,     3,     1,     1,     1,     1,     1,     4,     3,
       3,     3,     3,     3,     3,     3,     3,     3,     3,     3,
       3,     2,     2,     3,     2,     3,     3,     3,     1,     3,
       0,     1,     1,     3,     3,     5,     1,     3
};


enum { YYENOMEM = -2 };

#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (yychar = YYEMPTY)

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYNOMEM         goto yyexhaustedlab


#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                                    \
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
        yyerror (YY_("syntax error: cannot back up")); \
        YYERROR;                                                  \
      }                                                           \
  while (0)

/* Backward compatibility with an undocumented macro.
   Use YYerror or YYUNDEF. */
#define YYERRCODE YYUNDEF


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




# define YY_SYMBOL_PRINT(Title, Kind, Value, Location)                    \
do {                                                                      \
  if (yydebug)                                                            \
    {                                                                     \
      YYFPRINTF (stderr, "%s ", Title);                                   \
      yy_symbol_print (stderr,                                            \
                  Kind, Value); \
      YYFPRINTF (stderr, "\n");                                           \
    }                                                                     \
} while (0)


/*-----------------------------------.
| Print this symbol's value on YYO.  |
`-----------------------------------*/

static void
yy_symbol_value_print (FILE *yyo,
                       yysymbol_kind_t yykind, YYSTYPE const * const yyvaluep)
{
  FILE *yyoutput = yyo;
  YY_USE (yyoutput);
  if (!yyvaluep)
    return;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YY_USE (yykind);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}


/*---------------------------.
| Print this symbol on YYO.  |
`---------------------------*/

static void
yy_symbol_print (FILE *yyo,
                 yysymbol_kind_t yykind, YYSTYPE const * const yyvaluep)
{
  YYFPRINTF (yyo, "%s %s (",
             yykind < YYNTOKENS ? "token" : "nterm", yysymbol_name (yykind));

  yy_symbol_value_print (yyo, yykind, yyvaluep);
  YYFPRINTF (yyo, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

static void
yy_stack_print (yy_state_t *yybottom, yy_state_t *yytop)
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
yy_reduce_print (yy_state_t *yyssp, YYSTYPE *yyvsp,
                 int yyrule)
{
  int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %d):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       YY_ACCESSING_SYMBOL (+yyssp[yyi + 1 - yynrhs]),
                       &yyvsp[(yyi + 1) - (yynrhs)]);
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)          \
do {                                    \
  if (yydebug)                          \
    yy_reduce_print (yyssp, yyvsp, Rule); \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args) ((void) 0)
# define YY_SYMBOL_PRINT(Title, Kind, Value, Location)
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






/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

static void
yydestruct (const char *yymsg,
            yysymbol_kind_t yykind, YYSTYPE *yyvaluep)
{
  YY_USE (yyvaluep);
  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yykind, yyvaluep, yylocationp);

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YY_USE (yykind);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}


/* Lookahead token kind.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;
/* Number of syntax errors so far.  */
int yynerrs;




/*----------.
| yyparse.  |
`----------*/

int
yyparse (void)
{
    yy_state_fast_t yystate = 0;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus = 0;

    /* Refer to the stacks through separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* Their size.  */
    YYPTRDIFF_T yystacksize = YYINITDEPTH;

    /* The state stack: array, bottom, top.  */
    yy_state_t yyssa[YYINITDEPTH];
    yy_state_t *yyss = yyssa;
    yy_state_t *yyssp = yyss;

    /* The semantic value stack: array, bottom, top.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs = yyvsa;
    YYSTYPE *yyvsp = yyvs;

  int yyn;
  /* The return value of yyparse.  */
  int yyresult;
  /* Lookahead symbol kind.  */
  yysymbol_kind_t yytoken = YYSYMBOL_YYEMPTY;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yychar = YYEMPTY; /* Cause a token to be read.  */

  goto yysetstate;


/*------------------------------------------------------------.
| yynewstate -- push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;


/*--------------------------------------------------------------------.
| yysetstate -- set current state (the top of the stack) to yystate.  |
`--------------------------------------------------------------------*/
yysetstate:
  YYDPRINTF ((stderr, "Entering state %d\n", yystate));
  YY_ASSERT (0 <= yystate && yystate < YYNSTATES);
  YY_IGNORE_USELESS_CAST_BEGIN
  *yyssp = YY_CAST (yy_state_t, yystate);
  YY_IGNORE_USELESS_CAST_END
  YY_STACK_PRINT (yyss, yyssp);

  if (yyss + yystacksize - 1 <= yyssp)
#if !defined yyoverflow && !defined YYSTACK_RELOCATE
    YYNOMEM;
#else
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYPTRDIFF_T yysize = yyssp - yyss + 1;

# if defined yyoverflow
      {
        /* Give user a chance to reallocate the stack.  Use copies of
           these so that the &'s don't force the real ones into
           memory.  */
        yy_state_t *yyss1 = yyss;
        YYSTYPE *yyvs1 = yyvs;

        /* Each stack pointer address is followed by the size of the
           data in use in that stack, in bytes.  This used to be a
           conditional around just the two extra args, but that might
           be undefined if yyoverflow is a macro.  */
        yyoverflow (YY_("memory exhausted"),
                    &yyss1, yysize * YYSIZEOF (*yyssp),
                    &yyvs1, yysize * YYSIZEOF (*yyvsp),
                    &yystacksize);
        yyss = yyss1;
        yyvs = yyvs1;
      }
# else /* defined YYSTACK_RELOCATE */
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
        YYNOMEM;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
        yystacksize = YYMAXDEPTH;

      {
        yy_state_t *yyss1 = yyss;
        union yyalloc *yyptr =
          YY_CAST (union yyalloc *,
                   YYSTACK_ALLOC (YY_CAST (YYSIZE_T, YYSTACK_BYTES (yystacksize))));
        if (! yyptr)
          YYNOMEM;
        YYSTACK_RELOCATE (yyss_alloc, yyss);
        YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
        if (yyss1 != yyssa)
          YYSTACK_FREE (yyss1);
      }
# endif

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YY_IGNORE_USELESS_CAST_BEGIN
      YYDPRINTF ((stderr, "Stack size increased to %ld\n",
                  YY_CAST (long, yystacksize)));
      YY_IGNORE_USELESS_CAST_END

      if (yyss + yystacksize - 1 <= yyssp)
        YYABORT;
    }
#endif /* !defined yyoverflow && !defined YYSTACK_RELOCATE */


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

  /* YYCHAR is either empty, or end-of-input, or a valid lookahead.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token\n"));
      yychar = yylex ();
    }

  if (yychar <= YYEOF)
    {
      yychar = YYEOF;
      yytoken = YYSYMBOL_YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else if (yychar == YYerror)
    {
      /* The scanner already issued an error message, process directly
         to error recovery.  But do not keep the error token as
         lookahead, it is too special and may lead us to an endless
         loop in error recovery. */
      yychar = YYUNDEF;
      yytoken = YYSYMBOL_YYerror;
      goto yyerrlab1;
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
  yystate = yyn;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END

  /* Discard the shifted token.  */
  yychar = YYEMPTY;
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
| yyreduce -- do a reduction.  |
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
  case 5: /* line: exp ';'  */
#line 151 "ginsh_parser.ypp"
                  {
		try {
			cout << yyvsp[-1] << endl;
			push(yyvsp[-1]);
		} catch (exception &e) {
			cerr << e.what() << endl;
			YYERROR;
		}
	}
#line 1324 "ginsh_parser.cpp"
    break;

  case 6: /* line: exp ':'  */
#line 160 "ginsh_parser.ypp"
                  {
		try {
			push(yyvsp[-1]);
		} catch (exception &e) {
			std::cerr << e.what() << endl;
			YYERROR;
		}
	}
#line 1337 "ginsh_parser.cpp"
    break;

  case 7: /* line: T_PRINT '(' exp ')' ';'  */
#line 168 "ginsh_parser.ypp"
                                  {
		try {
			yyvsp[-2].print(print_tree(std::cout));
		} catch (exception &e) {
			std::cerr << e.what() << endl;
			YYERROR;
		}
	}
#line 1350 "ginsh_parser.cpp"
    break;

  case 8: /* line: T_IPRINT '(' exp ')' ';'  */
#line 176 "ginsh_parser.ypp"
                                   {
		try {
			ex e = yyvsp[-2];
			if (!e.info(info_flags::integer))
				throw (std::invalid_argument("argument to iprint() must be an integer"));
			long i = ex_to<numeric>(e).to_long();
			cout << i << endl;
			cout << "#o" << oct << i << endl;
			cout << "#x" << hex << i << dec << endl;
		} catch (exception &e) {
			cerr << e.what() << endl;
			YYERROR;
		}
	}
#line 1369 "ginsh_parser.cpp"
    break;

  case 9: /* line: T_PRINTLATEX '(' exp ')' ';'  */
#line 190 "ginsh_parser.ypp"
                                       {
		try {
			yyvsp[-2].print(print_latex(std::cout)); cout << endl;
		} catch (exception &e) {
			std::cerr << e.what() << endl;
			YYERROR;
		}
	}
#line 1382 "ginsh_parser.cpp"
    break;

  case 10: /* line: T_PRINTCSRC '(' exp ')' ';'  */
#line 198 "ginsh_parser.ypp"
                                      {
		try {
			yyvsp[-2].print(print_csrc_double(std::cout)); cout << endl;
		} catch (exception &e) {
			std::cerr << e.what() << endl;
			YYERROR;
		}
	}
#line 1395 "ginsh_parser.cpp"
    break;

  case 11: /* line: '?' T_SYMBOL  */
#line 206 "ginsh_parser.ypp"
                                {print_help(ex_to<symbol>(yyvsp[0]).get_name());}
#line 1401 "ginsh_parser.cpp"
    break;

  case 12: /* line: '?' T_TIME  */
#line 207 "ginsh_parser.ypp"
                                {print_help("time");}
#line 1407 "ginsh_parser.cpp"
    break;

  case 13: /* line: '?' T_PRINT  */
#line 208 "ginsh_parser.ypp"
                                {print_help("print");}
#line 1413 "ginsh_parser.cpp"
    break;

  case 14: /* line: '?' T_IPRINT  */
#line 209 "ginsh_parser.ypp"
                                {print_help("iprint");}
#line 1419 "ginsh_parser.cpp"
    break;

  case 15: /* line: '?' T_PRINTLATEX  */
#line 210 "ginsh_parser.ypp"
                                {print_help("print_latex");}
#line 1425 "ginsh_parser.cpp"
    break;

  case 16: /* line: '?' T_PRINTCSRC  */
#line 211 "ginsh_parser.ypp"
                                {print_help("print_csrc");}
#line 1431 "ginsh_parser.cpp"
    break;

  case 17: /* line: '?' '?'  */
#line 212 "ginsh_parser.ypp"
                                {print_help_topics();}
#line 1437 "ginsh_parser.cpp"
    break;

  case 18: /* line: T_QUIT  */
#line 213 "ginsh_parser.ypp"
                                {YYACCEPT;}
#line 1443 "ginsh_parser.cpp"
    break;

  case 19: /* line: T_WARRANTY  */
#line 214 "ginsh_parser.ypp"
                     {
		cout << "This program is free software; you can redistribute it and/or modify it under\n";
		cout << "the terms of the GNU General Public License as published by the Free Software\n";
		cout << "Foundation; either version 2 of the License, or (at your option) any later\n";
		cout << "version.\n";
		cout << "This program is distributed in the hope that it will be useful, but WITHOUT\n";
		cout << "ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS\n";
		cout << "FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more\n";
		cout << "details.\n";
		cout << "You should have received a copy of the GNU General Public License along with\n";
		cout << "this program. If not, write to the Free Software Foundation,\n";
		cout << "51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.\n";
	}
#line 1461 "ginsh_parser.cpp"
    break;

  case 20: /* line: T_XYZZY  */
#line 227 "ginsh_parser.ypp"
                                {cout << "Nothing happens.\n";}
#line 1467 "ginsh_parser.cpp"
    break;

  case 21: /* line: T_INVENTORY  */
#line 228 "ginsh_parser.ypp"
                                {cout << "You're not carrying anything.\n";}
#line 1473 "ginsh_parser.cpp"
    break;

  case 22: /* line: T_LOOK  */
#line 229 "ginsh_parser.ypp"
                                {cout << "You're in a twisty little maze of passages, all alike.\n";}
#line 1479 "ginsh_parser.cpp"
    break;

  case 23: /* line: T_SCORE  */
#line 230 "ginsh_parser.ypp"
                  {
		cout << "If you were to quit now, you would score ";
		cout << (syms.size() > 350 ? 350 : syms.size());
		cout << " out of a possible 350.\n";
	}
#line 1489 "ginsh_parser.cpp"
    break;

  case 24: /* line: T_REAL_SYMBOLS  */
#line 235 "ginsh_parser.ypp"
                         { symboltype = domain::real; }
#line 1495 "ginsh_parser.cpp"
    break;

  case 25: /* line: T_COMPLEX_SYMBOLS  */
#line 236 "ginsh_parser.ypp"
                            { symboltype = domain::complex; }
#line 1501 "ginsh_parser.cpp"
    break;

  case 26: /* $@1: %empty  */
#line 237 "ginsh_parser.ypp"
                 { START_TIMER }
#line 1507 "ginsh_parser.cpp"
    break;

  case 27: /* line: T_TIME $@1 '(' exp ')'  */
#line 237 "ginsh_parser.ypp"
                                             { STOP_TIMER PRINT_TIME_USED }
#line 1513 "ginsh_parser.cpp"
    break;

  case 28: /* line: error ';'  */
#line 238 "ginsh_parser.ypp"
                                {yyclearin; yyerrok;}
#line 1519 "ginsh_parser.cpp"
    break;

  case 29: /* line: error ':'  */
#line 239 "ginsh_parser.ypp"
                                {yyclearin; yyerrok;}
#line 1525 "ginsh_parser.cpp"
    break;

  case 30: /* exp: T_NUMBER  */
#line 242 "ginsh_parser.ypp"
                                {yyval = yyvsp[0];}
#line 1531 "ginsh_parser.cpp"
    break;

  case 31: /* exp: T_SYMBOL  */
#line 243 "ginsh_parser.ypp"
                                {
		auto i = assigned_symbol_table.find(yyvsp[0]);
		if (i == assigned_symbol_table.end())
			yyval = yyvsp[0];
		else
			yyval = i->second;
	}
#line 1543 "ginsh_parser.cpp"
    break;

  case 32: /* exp: '\'' T_SYMBOL '\''  */
#line 250 "ginsh_parser.ypp"
                                {yyval = yyvsp[-1];}
#line 1549 "ginsh_parser.cpp"
    break;

  case 33: /* exp: T_LITERAL  */
#line 251 "ginsh_parser.ypp"
                                {yyval = yyvsp[0];}
#line 1555 "ginsh_parser.cpp"
    break;

  case 34: /* exp: T_DIGITS  */
#line 252 "ginsh_parser.ypp"
                                {yyval = yyvsp[0];}
#line 1561 "ginsh_parser.cpp"
    break;

  case 35: /* exp: T_QUOTE  */
#line 253 "ginsh_parser.ypp"
                                {yyval = exstack[0];}
#line 1567 "ginsh_parser.cpp"
    break;

  case 36: /* exp: T_QUOTE2  */
#line 254 "ginsh_parser.ypp"
                                {yyval = exstack[1];}
#line 1573 "ginsh_parser.cpp"
    break;

  case 37: /* exp: T_QUOTE3  */
#line 255 "ginsh_parser.ypp"
                                {yyval = exstack[2];}
#line 1579 "ginsh_parser.cpp"
    break;

  case 38: /* exp: T_SYMBOL '(' exprseq ')'  */
#line 256 "ginsh_parser.ypp"
                                   {
		auto i = find_function(yyvsp[-3], yyvsp[-1].nops());
		if (i->second.is_ginac) {
			yyval = ((fcnp2)(i->second.p))(ex_to<exprseq>(yyvsp[-1]), i->second.serial);
		} else {
			yyval = (i->second.p)(ex_to<exprseq>(yyvsp[-1]));
		}
	}
#line 1592 "ginsh_parser.cpp"
    break;

  case 39: /* exp: T_DIGITS '=' T_NUMBER  */
#line 264 "ginsh_parser.ypp"
                                {yyval = yyvsp[0]; Digits = ex_to<numeric>(yyvsp[0]).to_int();}
#line 1598 "ginsh_parser.cpp"
    break;

  case 40: /* exp: T_SYMBOL '=' exp  */
#line 265 "ginsh_parser.ypp"
                                {yyval = yyvsp[0]; assigned_symbol_table[yyvsp[-2]] = yyvsp[0]; }
#line 1604 "ginsh_parser.cpp"
    break;

  case 41: /* exp: exp T_EQUAL exp  */
#line 266 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] == yyvsp[0];}
#line 1610 "ginsh_parser.cpp"
    break;

  case 42: /* exp: exp T_NOTEQ exp  */
#line 267 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] != yyvsp[0];}
#line 1616 "ginsh_parser.cpp"
    break;

  case 43: /* exp: exp '<' exp  */
#line 268 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] < yyvsp[0];}
#line 1622 "ginsh_parser.cpp"
    break;

  case 44: /* exp: exp T_LESSEQ exp  */
#line 269 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] <= yyvsp[0];}
#line 1628 "ginsh_parser.cpp"
    break;

  case 45: /* exp: exp '>' exp  */
#line 270 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] > yyvsp[0];}
#line 1634 "ginsh_parser.cpp"
    break;

  case 46: /* exp: exp T_GREATEREQ exp  */
#line 271 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] >= yyvsp[0];}
#line 1640 "ginsh_parser.cpp"
    break;

  case 47: /* exp: exp '+' exp  */
#line 272 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] + yyvsp[0];}
#line 1646 "ginsh_parser.cpp"
    break;

  case 48: /* exp: exp '-' exp  */
#line 273 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] - yyvsp[0];}
#line 1652 "ginsh_parser.cpp"
    break;

  case 49: /* exp: exp '*' exp  */
#line 274 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] * yyvsp[0];}
#line 1658 "ginsh_parser.cpp"
    break;

  case 50: /* exp: exp '/' exp  */
#line 275 "ginsh_parser.ypp"
                                {yyval = yyvsp[-2] / yyvsp[0];}
#line 1664 "ginsh_parser.cpp"
    break;

  case 51: /* exp: '-' exp  */
#line 276 "ginsh_parser.ypp"
                                {yyval = -yyvsp[0];}
#line 1670 "ginsh_parser.cpp"
    break;

  case 52: /* exp: '+' exp  */
#line 277 "ginsh_parser.ypp"
                                {yyval = yyvsp[0];}
#line 1676 "ginsh_parser.cpp"
    break;

  case 53: /* exp: exp '^' exp  */
#line 278 "ginsh_parser.ypp"
                                {yyval = power(yyvsp[-2], yyvsp[0]);}
#line 1682 "ginsh_parser.cpp"
    break;

  case 54: /* exp: exp '!'  */
#line 279 "ginsh_parser.ypp"
                                {yyval = factorial(yyvsp[-1]);}
#line 1688 "ginsh_parser.cpp"
    break;

  case 55: /* exp: '(' exp ')'  */
#line 280 "ginsh_parser.ypp"
                                {yyval = yyvsp[-1];}
#line 1694 "ginsh_parser.cpp"
    break;

  case 56: /* exp: '{' list_or_empty '}'  */
#line 281 "ginsh_parser.ypp"
                                {yyval = yyvsp[-1];}
#line 1700 "ginsh_parser.cpp"
    break;

  case 57: /* exp: '[' matrix ']'  */
#line 282 "ginsh_parser.ypp"
                                {yyval = lst_to_matrix(ex_to<lst>(yyvsp[-1]));}
#line 1706 "ginsh_parser.cpp"
    break;

  case 58: /* exprseq: exp  */
#line 285 "ginsh_parser.ypp"
                                {yyval = exprseq{yyvsp[0]};}
#line 1712 "ginsh_parser.cpp"
    break;

  case 59: /* exprseq: exprseq ',' exp  */
#line 286 "ginsh_parser.ypp"
                                {exprseq es(ex_to<exprseq>(yyvsp[-2])); yyval = es.append(yyvsp[0]);}
#line 1718 "ginsh_parser.cpp"
    break;

  case 60: /* list_or_empty: %empty  */
#line 289 "ginsh_parser.ypp"
                                {yyval = *new lst;}
#line 1724 "ginsh_parser.cpp"
    break;

  case 61: /* list_or_empty: list  */
#line 290 "ginsh_parser.ypp"
                                {yyval = yyvsp[0];}
#line 1730 "ginsh_parser.cpp"
    break;

  case 62: /* list: exp  */
#line 293 "ginsh_parser.ypp"
                                {yyval = lst{yyvsp[0]};}
#line 1736 "ginsh_parser.cpp"
    break;

  case 63: /* list: list ',' exp  */
#line 294 "ginsh_parser.ypp"
                                {lst l(ex_to<lst>(yyvsp[-2])); yyval = l.append(yyvsp[0]);}
#line 1742 "ginsh_parser.cpp"
    break;

  case 64: /* matrix: '[' row ']'  */
#line 297 "ginsh_parser.ypp"
                                {yyval = lst{yyvsp[-1]};}
#line 1748 "ginsh_parser.cpp"
    break;

  case 65: /* matrix: matrix ',' '[' row ']'  */
#line 298 "ginsh_parser.ypp"
                                 {lst l(ex_to<lst>(yyvsp[-4])); yyval = l.append(yyvsp[-1]);}
#line 1754 "ginsh_parser.cpp"
    break;

  case 66: /* row: exp  */
#line 301 "ginsh_parser.ypp"
                                {yyval = lst{yyvsp[0]};}
#line 1760 "ginsh_parser.cpp"
    break;

  case 67: /* row: row ',' exp  */
#line 302 "ginsh_parser.ypp"
                                {lst l(ex_to<lst>(yyvsp[-2])); yyval = l.append(yyvsp[0]);}
#line 1766 "ginsh_parser.cpp"
    break;


#line 1770 "ginsh_parser.cpp"

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
  YY_SYMBOL_PRINT ("-> $$ =", YY_CAST (yysymbol_kind_t, yyr1[yyn]), &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;

  *++yyvsp = yyval;

  /* Now 'shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */
  {
    const int yylhs = yyr1[yyn] - YYNTOKENS;
    const int yyi = yypgoto[yylhs] + *yyssp;
    yystate = (0 <= yyi && yyi <= YYLAST && yycheck[yyi] == *yyssp
               ? yytable[yyi]
               : yydefgoto[yylhs]);
  }

  goto yynewstate;


/*--------------------------------------.
| yyerrlab -- here on detecting error.  |
`--------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYSYMBOL_YYEMPTY : YYTRANSLATE (yychar);
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
      yyerror (YY_("syntax error"));
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
                      yytoken, &yylval);
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
  /* Pacify compilers when the user code never invokes YYERROR and the
     label yyerrorlab therefore never appears in user code.  */
  if (0)
    YYERROR;
  ++yynerrs;

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

  /* Pop stack until we find a state that shifts the error token.  */
  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
        {
          yyn += YYSYMBOL_YYerror;
          if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYSYMBOL_YYerror)
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
                  YY_ACCESSING_SYMBOL (yystate), yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", YY_ACCESSING_SYMBOL (yyn), yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturnlab;


/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturnlab;


/*-----------------------------------------------------------.
| yyexhaustedlab -- YYNOMEM (memory exhaustion) comes here.  |
`-----------------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  goto yyreturnlab;


/*----------------------------------------------------------.
| yyreturnlab -- parsing is finished, clean up and return.  |
`----------------------------------------------------------*/
yyreturnlab:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval);
    }
  /* Do not reclaim the symbols of the rule whose action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
                  YY_ACCESSING_SYMBOL (+*yyssp), yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif

  return yyresult;
}

#line 310 "ginsh_parser.ypp"

// Error print routine
int yyerror(const char *s)
{
	cerr << s << " at " << yytext << endl;
	return 0;
}

// Push expression "e" onto the expression stack (for ", "" and """)
static void push(const ex &e)
{
	exstack[2] = exstack[1];
	exstack[1] = exstack[0];
	exstack[0] = e;
}


/*
 *  Built-in functions
 */

static ex f_collect(const exprseq &e) {return e[0].collect(e[1]);}
static ex f_collect_distributed(const exprseq &e) {return e[0].collect(e[1], true);}
static ex f_collect_common_factors(const exprseq &e) {return collect_common_factors(e[0]);}
static ex f_convert_H_to_Li(const exprseq &e) {return convert_H_to_Li(e[0], e[1]);}
static ex f_degree(const exprseq &e) {return e[0].degree(e[1]);}
static ex f_denom(const exprseq &e) {return e[0].denom();}
static ex f_evalf(const exprseq &e) {return e[0].evalf();}
static ex f_evalm(const exprseq &e) {return e[0].evalm();}
static ex f_eval_integ(const exprseq &e) {return e[0].eval_integ();}
static ex f_expand(const exprseq &e) {return e[0].expand();}
static ex f_factor(const exprseq &e) {return factor(e[0]);}
static ex f_gcd(const exprseq &e) {return gcd(e[0], e[1]);}
static ex f_has(const exprseq &e) {return e[0].has(e[1]) ? ex(1) : ex(0);}
static ex f_lcm(const exprseq &e) {return lcm(e[0], e[1]);}
static ex f_lcoeff(const exprseq &e) {return e[0].lcoeff(e[1]);}
static ex f_ldegree(const exprseq &e) {return e[0].ldegree(e[1]);}
static ex f_lsolve(const exprseq &e) {return lsolve(e[0], e[1]);}
static ex f_nops(const exprseq &e) {return e[0].nops();}
static ex f_normal(const exprseq &e) {return e[0].normal();}
static ex f_numer(const exprseq &e) {return e[0].numer();}
static ex f_numer_denom(const exprseq &e) {return e[0].numer_denom();}
static ex f_pow(const exprseq &e) {return pow(e[0], e[1]);}
static ex f_sqrt(const exprseq &e) {return sqrt(e[0]);}
static ex f_sqrfree1(const exprseq &e) {return sqrfree(e[0]);}
static ex f_subs2(const exprseq &e) {return e[0].subs(e[1]);}
static ex f_tcoeff(const exprseq &e) {return e[0].tcoeff(e[1]);}

#define CHECK_ARG(num, type, fcn) if (!is_a<type>(e[num])) throw(std::invalid_argument("argument " #num " to " #fcn "() must be a " #type))

static ex f_charpoly(const exprseq &e)
{
	CHECK_ARG(0, matrix, charpoly);
	return ex_to<matrix>(e[0]).charpoly(e[1]);
}

static ex f_coeff(const exprseq &e)
{
	CHECK_ARG(2, numeric, coeff);
	return e[0].coeff(e[1], ex_to<numeric>(e[2]).to_int());
}

static ex f_content(const exprseq &e)
{
	return e[0].content(e[1]);
}

static ex f_decomp_rational(const exprseq &e)
{
	return decomp_rational(e[0], e[1]);
}

static ex f_determinant(const exprseq &e)
{
	CHECK_ARG(0, matrix, determinant);
	return ex_to<matrix>(e[0]).determinant();
}

static ex f_diag(const exprseq &e)
{
	size_t dim = e.nops();
	matrix &m = *new matrix(dim, dim);
	for (size_t i=0; i<dim; i++)
		m.set(i, i, e.op(i));
	return m;
}

static ex f_diff2(const exprseq &e)
{
	CHECK_ARG(1, symbol, diff);
	return e[0].diff(ex_to<symbol>(e[1]));
}

static ex f_diff3(const exprseq &e)
{
	CHECK_ARG(1, symbol, diff);
	CHECK_ARG(2, numeric, diff);
	return e[0].diff(ex_to<symbol>(e[1]), ex_to<numeric>(e[2]).to_int());
}

static ex f_divide(const exprseq &e)
{
	ex q;
	if (divide(e[0], e[1], q))
		return q;
	else
		return fail();
}

static ex f_find(const exprseq &e)
{
	exset found;
	e[0].find(e[1], found);
	lst l;
	for (auto & i : found)
		l.append(i);
	return l;
}

static ex f_fsolve(const exprseq &e)
{
	CHECK_ARG(1, symbol, fsolve);
	CHECK_ARG(2, numeric, fsolve);
	CHECK_ARG(3, numeric, fsolve);
	return fsolve(e[0], ex_to<symbol>(e[1]), ex_to<numeric>(e[2]), ex_to<numeric>(e[3]));
}

static ex f_integer_content(const exprseq &e)
{
	return e[0].expand().integer_content();
}

static ex f_integral(const exprseq &e)
{
	CHECK_ARG(0, symbol, integral);
	return GiNaC::integral(e[0], e[1], e[2], e[3]);
}

static ex f_inverse(const exprseq &e)
{
	CHECK_ARG(0, matrix, inverse);
	return ex_to<matrix>(e[0]).inverse();
}

static ex f_is(const exprseq &e)
{
	CHECK_ARG(0, relational, is);
	return (bool)ex_to<relational>(e[0]) ? ex(1) : ex(0);
}

class apply_map_function : public map_function {
	ex apply;
public:
	apply_map_function(const ex & a) : apply(a) {}
	virtual ~apply_map_function() {}
	ex operator()(const ex & e) override { return apply.subs(wild() == e, true); }
};

static ex f_map(const exprseq &e)
{
	apply_map_function fcn(e[1]);
	return e[0].map(fcn);
}

static ex f_match(const exprseq &e)
{
	exmap repls;
	if (e[0].match(e[1], repls)) {
		lst repl_lst;
		for (auto & i : repls)
			repl_lst.append(relational(i.first, i.second, relational::equal));
		return repl_lst;
	}
	throw std::runtime_error("FAIL");
}

static ex f_op(const exprseq &e)
{
	CHECK_ARG(1, numeric, op);
	int n = ex_to<numeric>(e[1]).to_int();
	if (n < 0 || n >= (int)e[0].nops())
		throw(std::out_of_range("second argument to op() is out of range"));
	return e[0].op(n);
}

static ex f_prem(const exprseq &e)
{
	return prem(e[0], e[1], e[2]);
}

static ex f_primpart(const exprseq &e)
{
	return e[0].primpart(e[1]);
}

static ex f_quo(const exprseq &e)
{
	return quo(e[0], e[1], e[2]);
}

static ex f_rank(const exprseq &e)
{
	CHECK_ARG(0, matrix, rank);
	return ex_to<matrix>(e[0]).rank();
}

static ex f_rem(const exprseq &e)
{
	return rem(e[0], e[1], e[2]);
}

static ex f_resultant(const exprseq &e)
{
	CHECK_ARG(2, symbol, resultant);
	return resultant(e[0], e[1], ex_to<symbol>(e[2]));
}

static ex f_series(const exprseq &e)
{
	CHECK_ARG(2, numeric, series);
	return e[0].series(e[1], ex_to<numeric>(e[2]).to_int());
}

static ex f_series_to_poly(const exprseq &e)
{
	CHECK_ARG(0, pseries, series_to_poly);
	return series_to_poly(ex_to<pseries>(e[0]));
}

static ex f_sprem(const exprseq &e)
{
	return sprem(e[0], e[1], e[2]);
}

static ex f_sqrfree2(const exprseq &e)
{
	CHECK_ARG(1, lst, sqrfree);
	return sqrfree(e[0], ex_to<lst>(e[1]));
}

static ex f_sqrfree_parfrac(const exprseq &e)
{
	return sqrfree_parfrac(e[0], ex_to<symbol>(e[1]));
}

static ex f_subs3(const exprseq &e)
{
	CHECK_ARG(1, lst, subs);
	CHECK_ARG(2, lst, subs);
	return e[0].subs(ex_to<lst>(e[1]), ex_to<lst>(e[2]));
}

static ex f_trace(const exprseq &e)
{
	CHECK_ARG(0, matrix, trace);
	return ex_to<matrix>(e[0]).trace();
}

static ex f_transpose(const exprseq &e)
{
	CHECK_ARG(0, matrix, transpose);
	return ex_to<matrix>(e[0]).transpose();
}

static ex f_unassign(const exprseq &e)
{
	CHECK_ARG(0, symbol, unassign);
	exmap::iterator i = assigned_symbol_table.find(e[0]);
	if (i != assigned_symbol_table.end())
		assigned_symbol_table.erase(i);
	return e[0];
}

static ex f_unit(const exprseq &e)
{
	return e[0].unit(e[1]);
}

static ex f_basic_log_kernel(const exprseq &e)
{
	return basic_log_kernel();	
}

static ex f_multiple_polylog_kernel(const exprseq &e)
{
	return multiple_polylog_kernel(e[0]);	
}

static ex f_ELi_kernel(const exprseq &e)
{
	return ELi_kernel(e[0],e[1],e[2],e[3]);	
}

static ex f_Ebar_kernel(const exprseq &e)
{
	return Ebar_kernel(e[0],e[1],e[2],e[3]);	
}

static ex f_Kronecker_dtau_kernel_4(const exprseq &e)
{
	return Kronecker_dtau_kernel(e[0],e[1],e[2],e[3]);	
}

static ex f_Kronecker_dtau_kernel_3(const exprseq &e)
{
	return Kronecker_dtau_kernel(e[0],e[1],e[2]);	
}

static ex f_Kronecker_dtau_kernel_2(const exprseq &e)
{
	return Kronecker_dtau_kernel(e[0],e[1]);	
}

static ex f_Kronecker_dz_kernel_5(const exprseq &e)
{
	return Kronecker_dz_kernel(e[0],e[1],e[2],e[3],e[4]);	
}

static ex f_Kronecker_dz_kernel_4(const exprseq &e)
{
	return Kronecker_dz_kernel(e[0],e[1],e[2],e[3]);	
}

static ex f_Kronecker_dz_kernel_3(const exprseq &e)
{
	return Kronecker_dz_kernel(e[0],e[1],e[2]);	
}

static ex f_Eisenstein_kernel_6(const exprseq &e)
{
	return Eisenstein_kernel(e[0],e[1],e[2],e[3],e[4],e[5]);	
}

static ex f_Eisenstein_kernel_5(const exprseq &e)
{
	return Eisenstein_kernel(e[0],e[1],e[2],e[3],e[4]);	
}

static ex f_Eisenstein_h_kernel_5(const exprseq &e)
{
	return Eisenstein_h_kernel(e[0],e[1],e[2],e[3],e[4]);	
}

static ex f_Eisenstein_h_kernel_4(const exprseq &e)
{
	return Eisenstein_h_kernel(e[0],e[1],e[2],e[3]);	
}

static ex f_modular_form_kernel_3(const exprseq &e)
{
	return modular_form_kernel(e[0],e[1],e[2]);	
}

static ex f_modular_form_kernel_2(const exprseq &e)
{
	return modular_form_kernel(e[0],e[1]);	
}

static ex f_user_defined_kernel(const exprseq &e)
{
	return user_defined_kernel(e[0],e[1]);	
}

static ex f_q_expansion_modular_form(const exprseq &e)
{
	if ( is_a<Eisenstein_kernel>(e[0]) ) {
		return ex_to<Eisenstein_kernel>(e[0]).q_expansion_modular_form(e[1], ex_to<numeric>(e[2]).to_int());
	}	
	if ( is_a<Eisenstein_h_kernel>(e[0]) ) {
		return ex_to<Eisenstein_h_kernel>(e[0]).q_expansion_modular_form(e[1], ex_to<numeric>(e[2]).to_int());
	}	
	if ( is_a<modular_form_kernel>(e[0]) ) {
		return ex_to<modular_form_kernel>(e[0]).q_expansion_modular_form(e[1], ex_to<numeric>(e[2]).to_int());
	}	
	throw(std::invalid_argument("first argument must be a modular form"));
}

static ex f_dummy(const exprseq &e)
{
	throw(std::logic_error("dummy function called (shouldn't happen)"));
}

// Tables for initializing the "fcns" map and the function help topics
struct fcn_init {
	const char *name;
	fcnp p;
	int num_params;
};

static const fcn_init builtin_fcns[] = {
	{"charpoly", f_charpoly, 2},
	{"coeff", f_coeff, 3},
	{"collect", f_collect, 2},
	{"collect_common_factors", f_collect_common_factors, 1},
	{"collect_distributed", f_collect_distributed, 2},
	{"content", f_content, 2},
	{"convert_H_to_Li", f_convert_H_to_Li, 2},
	{"decomp_rational", f_decomp_rational, 2},
	{"degree", f_degree, 2},
	{"denom", f_denom, 1},
	{"determinant", f_determinant, 1},
	{"diag", f_diag, 0},
	{"diff", f_diff2, 2},
	{"diff", f_diff3, 3},
	{"divide", f_divide, 2},
	{"evalf", f_evalf, 1},
	{"evalm", f_evalm, 1},
	{"eval_integ", f_eval_integ, 1},
	{"expand", f_expand, 1},
	{"factor", f_factor, 1},
	{"find", f_find, 2},
	{"fsolve", f_fsolve, 4},
	{"gcd", f_gcd, 2},
	{"has", f_has, 2},
	{"integer_content", f_integer_content, 1},
	{"integral", f_integral, 4},
	{"inverse", f_inverse, 1},
	{"iprint", f_dummy, 0},      // for Tab-completion
	{"is", f_is, 1},
	{"lcm", f_lcm, 2},
	{"lcoeff", f_lcoeff, 2},
	{"ldegree", f_ldegree, 2},
	{"lsolve", f_lsolve, 2},
	{"map", f_map, 2},
	{"match", f_match, 2},
	{"nops", f_nops, 1},
	{"normal", f_normal, 1},
	{"numer", f_numer, 1},
	{"numer_denom", f_numer_denom, 1},
	{"op", f_op, 2},
	{"pow", f_pow, 2},
	{"prem", f_prem, 3},
	{"primpart", f_primpart, 2},
	{"print", f_dummy, 0},       // for Tab-completion
	{"print_csrc", f_dummy, 0},  // for Tab-completion
	{"print_latex", f_dummy, 0}, // for Tab-completion
	{"quo", f_quo, 3},
	{"rank", f_rank, 1},
	{"rem", f_rem, 3},
	{"resultant", f_resultant, 3},
	{"series", f_series, 3},
	{"series_to_poly", f_series_to_poly, 1},
	{"sprem", f_sprem, 3},
	{"sqrfree", f_sqrfree1, 1},
	{"sqrfree", f_sqrfree2, 2},
	{"sqrfree_parfrac", f_sqrfree_parfrac, 2},
	{"sqrt", f_sqrt, 1},
	{"subs", f_subs2, 2},
	{"subs", f_subs3, 3},
	{"tcoeff", f_tcoeff, 2},
	{"time", f_dummy, 0},        // for Tab-completion
	{"trace", f_trace, 1},
	{"transpose", f_transpose, 1},
	{"unassign", f_unassign, 1},
	{"unit", f_unit, 2},
	{"basic_log_kernel", f_basic_log_kernel, 0},
	{"multiple_polylog_kernel", f_multiple_polylog_kernel, 1},
	{"ELi_kernel", f_ELi_kernel, 4},
	{"Ebar_kernel", f_Ebar_kernel, 4},
	{"Kronecker_dtau_kernel", f_Kronecker_dtau_kernel_4, 4},
	{"Kronecker_dtau_kernel", f_Kronecker_dtau_kernel_3, 3},
	{"Kronecker_dtau_kernel", f_Kronecker_dtau_kernel_2, 2},
	{"Kronecker_dz_kernel", f_Kronecker_dz_kernel_5, 5},
	{"Kronecker_dz_kernel", f_Kronecker_dz_kernel_4, 4},
	{"Kronecker_dz_kernel", f_Kronecker_dz_kernel_3, 3},
	{"Eisenstein_kernel", f_Eisenstein_kernel_6, 6},
	{"Eisenstein_kernel", f_Eisenstein_kernel_5, 5},
	{"Eisenstein_h_kernel", f_Eisenstein_h_kernel_5, 5},
	{"Eisenstein_h_kernel", f_Eisenstein_h_kernel_4, 4},
	{"modular_form_kernel", f_modular_form_kernel_3, 3},
	{"modular_form_kernel", f_modular_form_kernel_2, 2},
	{"user_defined_kernel", f_user_defined_kernel, 2},
	{"q_expansion_modular_form", f_q_expansion_modular_form, 3},
	{nullptr, f_dummy, 0}        // End marker
};

struct fcn_help_init {
	const char *name;
	const char *help;
};

static const fcn_help_init builtin_help[] = {
	{"acos", "inverse cosine function"},
	{"acosh", "inverse hyperbolic cosine function"},
	{"asin", "inverse sine function"},
	{"asinh", "inverse hyperbolic sine function"},
	{"atan", "inverse tangent function"},
	{"atan2", "inverse tangent function with two arguments"},
	{"atanh", "inverse hyperbolic tangent function"},
	{"beta", "Beta function"},
	{"binomial", "binomial function"},
	{"cos", "cosine function"},
	{"cosh", "hyperbolic cosine function"},
	{"exp", "exponential function"},
	{"factorial", "factorial function"},
	{"lgamma", "natural logarithm of Gamma function"},
	{"tgamma", "Gamma function"},
	{"log", "natural logarithm"},
	{"psi", "psi function\npsi(x) is the digamma function, psi(n,x) the nth polygamma function"},
	{"sin", "sine function"},
	{"sinh", "hyperbolic sine function"},
	{"tan", "tangent function"},
	{"tanh", "hyperbolic tangent function"},
	{"zeta", "zeta function\nzeta(x) is Riemann's zeta function, zetaderiv(n,x) its nth derivative.\nIf x is a GiNaC::lst, it is a multiple zeta value\nzeta(x,s) is an alternating Euler sum"},
	{"G", "multiple polylogarithm (integral representation)"},
	{"Li2", "dilogarithm"},
	{"Li3", "trilogarithm"},
	{"Li", "(multiple) polylogarithm"},
	{"S", "Nielsen's generalized polylogarithm"},
	{"H", "harmonic polylogarithm"},
	{"EllipticK", "complete elliptic integral of the first kind"},
	{"EllipticE", "complete elliptic integral of the second kind"},
	{"iterated_integral", "iterated integral"},
	{"Order", "order term function (for truncated power series)"},
	{"Derivative", "inert differential operator"},
	{nullptr, nullptr}  // End marker
};

#include "ginsh_extensions.h"


/*
 *  Add functions to ginsh
 */

// Functions from fcn_init array
static void insert_fcns(const fcn_init *p)
{
	while (p->name) {
		fcns.insert(make_pair(string(p->name), fcn_desc(p->p, p->num_params)));
		p++;
	}
}

static ex f_ginac_function(const exprseq &es, int serial)
{
	return GiNaC::function(serial, es);
}

// All registered GiNaC functions
namespace GiNaC {
static void ginsh_get_ginac_functions(void)
{
	unsigned serial = 0;
	for (auto & i : function::get_registered_functions()) {
		fcns.insert(make_pair(i.get_name(), fcn_desc(f_ginac_function, i.get_nparams(), serial)));
		serial++;
	}
}
}


/*
 *  Find a function given a name and number of parameters. Throw exceptions on error.
 */

static fcn_tab::const_iterator find_function(const ex &sym, int req_params)
{
	const string &name = ex_to<symbol>(sym).get_name();
	typedef fcn_tab::const_iterator I;
	pair<I, I> b = fcns.equal_range(name);
	if (b.first == b.second)
		throw(std::logic_error("unknown function '" + name + "'"));
	else {
		for (I i=b.first; i!=b.second; i++)
			if ((i->second.num_params == 0) || (i->second.num_params == req_params))
				return i;
	}
	throw(std::logic_error("invalid number of arguments to " + name + "()"));
}


/*
 *  Insert help strings
 */

// Normal help string
static void insert_help(const char *topic, const char *str)
{
	help.insert(make_pair(string(topic), string(str)));
}

// Help string for functions, automatically generates synopsis
static void insert_fcn_help(const char *name, const char *str)
{
	typedef fcn_tab::const_iterator I;
	pair<I, I> b = fcns.equal_range(name);
	if (b.first != b.second) {
		string help_str = string(name) + "(";
		for (int i=0; i<b.first->second.num_params; i++) {
			if (i)
				help_str += ", ";
			help_str += "expression";
		}
		help_str += ") - ";
		help_str += str;
		help.insert(make_pair(string(name), help_str));
	}
}

// Help strings for functions from fcn_help_init array
static void insert_help(const fcn_help_init *p)
{
	while (p->name) {
		insert_fcn_help(p->name, p->help);
		p++;
	}
}


/*
 *  Print help to cout
 */

// Help for a given topic
static void print_help(const string &topic)
{
	typedef help_tab::const_iterator I;
	pair<I, I> b = help.equal_range(topic);
	if (b.first == b.second)
		cout << "no help for '" << topic << "'\n";
	else {
		for (I i=b.first; i!=b.second; i++)
			cout << i->second << endl;
	}
}

// List of help topics
static void print_help_topics(void)
{
	cout << "Available help topics:\n";
	help_tab::const_iterator i;
	string last_name = string("*");
	int num = 0;
	for (i=help.begin(); i!=help.end(); i++) {
		// Don't print duplicates
		if (i->first != last_name) {
			if (num)
				cout << ", ";
			num++;
			cout << i->first;
			last_name = i->first;
		}
	}
	cout << "\nTo get help for a certain topic, type ?topic\n";
}


/*
 *  Function name completion functions for readline
 */

static char *fcn_generator(const char *text, int state)
{
	static int len;				// Length of word to complete
	static fcn_tab::const_iterator index;	// Iterator to function being currently considered

	// If this is a new word to complete, initialize now
	if (state == 0) {
		index = fcns.begin();
		len = strlen(text);
	}

	// Return the next function which partially matches
	while (index != fcns.end()) {
		const char *fcn_name = index->first.c_str();
		++index;
		if (strncmp(fcn_name, text, len) == 0)
			return strdup(fcn_name);
	}
	return nullptr;
}

#ifdef HAVE_LIBREADLINE
static char **fcn_completion(const char *text, int start, int end)
{
	if (rl_line_buffer[0] == '!') {
		// For shell commands, revert back to filename completion
		rl_completion_append_character = orig_completion_append_character;
		rl_basic_word_break_characters = orig_basic_word_break_characters;
		rl_completer_word_break_characters = GINAC_RL_COMPLETER_CAST(rl_basic_word_break_characters);
		return rl_completion_matches(text, rl_filename_completion_function);
	} else {
		// Otherwise, complete function names
		rl_completion_append_character = '(';
		rl_basic_word_break_characters = " \t\n\"#$%&'()*+,-./:;<=>?@[\\]^`{|}~";
		rl_completer_word_break_characters = GINAC_RL_COMPLETER_CAST(rl_basic_word_break_characters);
		return rl_completion_matches(text, fcn_generator);
	}
}
#endif // HAVE_LIBREADLINE

static void ginsh_readline_init(char* name)
{
#ifdef HAVE_LIBREADLINE
	// Init readline completer
	rl_readline_name = name;
	rl_attempted_completion_function = fcn_completion;
	orig_completion_append_character = rl_completion_append_character;
	orig_basic_word_break_characters = rl_basic_word_break_characters;
#endif // HAVE_LIBREADLINE
}

void greeting(void)
{
    cout << "ginsh - GiNaC Interactive Shell (GiNaC V" << GINACLIB_VERSION << ")" << endl;
    cout << "  __,  _______  Copyright (C) 1999-2023 Johannes Gutenberg University Mainz,\n"
         << " (__) *       | Germany.  This is free software with ABSOLUTELY NO WARRANTY.\n"
         << "  ._) i N a C | You are welcome to redistribute it under certain conditions.\n"
         << "<-------------' For details type `warranty;'.\n" << endl;
    cout << "Type ?? for a list of help topics." << endl;
}

/*
 *  Main program
 */

int main(int argc, char **argv)
{
	// Print banner in interactive mode
	if (isatty(0)) 
		greeting();
	assigned_symbol_table = exmap();

	// Init function table
	insert_fcns(builtin_fcns);
	insert_fcns(extended_fcns);
	ginsh_get_ginac_functions();

	// Init help for operators (automatically generated from man page)
	insert_help("operators", "Operators in falling order of precedence:");
#include "ginsh_op_help.h"

	// Init help for built-in functions (automatically generated from man page)
#include "ginsh_fcn_help.h"

	// Help for GiNaC functions is added manually
	insert_help(builtin_help);
	insert_help(extended_help);

	// Help for other keywords
	insert_help("print", "print(expression) - dumps the internal structure of the given expression (for debugging)");
	insert_help("iprint", "iprint(expression) - prints the given integer expression in decimal, octal, and hexadecimal bases");
	insert_help("print_latex", "print_latex(expression) - prints a LaTeX representation of the given expression");
	insert_help("print_csrc", "print_csrc(expression) - prints a C source code representation of the given expression");

	ginsh_readline_init(argv[0]);

	// Init input file list, open first file
	num_files = argc - 1;
	file_list = argv + 1;
	if (num_files) {
		yyin = fopen(*file_list, "r");
		if (yyin == nullptr) {
			cerr << "Can't open " << *file_list << endl;
			exit(1);
		}
		num_files--;
		file_list++;
	}

	// Parse input, catch all remaining exceptions
	int result;
again:	try {
		result = yyparse();
	} catch (exception &e) {
		cerr << e.what() << endl;
		goto again;
	}
	return result;
}
