dnl Copyright (C) 1993-2002 Free Software Foundation, Inc.
dnl This file is free software, distributed under the terms of the GNU
dnl General Public License.  As a special exception to the GNU General
dnl Public License, this file may be distributed as part of a program
dnl that contains a configuration script generated by Autoconf, under
dnl the same distribution terms as the rest of that program.

dnl From Bruno Haible, Marcus Daniels.

AC_PREREQ([2.13])

dnl CL_PROTO(IDENTIFIER, ACTION-IF-NOT-FOUND, FINAL-PROTOTYPE)
AC_DEFUN([CL_PROTO],
[AC_MSG_CHECKING([for $1 declaration])
AC_CACHE_VAL(cl_cv_proto_[$1], [$2
cl_cv_proto_$1="$3"])
cl_cv_proto_$1=`echo "[$]cl_cv_proto_$1" | tr -s ' ' | sed -e 's/( /(/'`
AC_MSG_RESULT([$]{ac_t:-
         }[$]cl_cv_proto_$1)
])

dnl CL_PROTO_TRY(INCLUDES, DECL, ACTION-IF-OK, ACTION-IF-FAILS)
AC_DEFUN([CL_PROTO_TRY],
[AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[$1
]AC_LANG_EXTERN[
$2
]], [[]])], [$3], [$4])
])