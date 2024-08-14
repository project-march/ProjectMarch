## Process this file with automake to produce Makefile.in

EXFILES = archive1.cpp compile1.cpp compile2.cpp compile3.cpp lanczos.cpp \
 mystring.cpp derivative.cpp

TEXI = ginac-examples.texi

info_TEXINFOS = $(TEXI)

TXT = ginac-examples.txt
HTML = ginac-examples.html

all: $(EXFILES)
	${MAKEINFO} --no-split --no-headers $(srcdir)/$(TEXI) > $(TXT)
	${MAKEINFO} --no-split --html $(srcdir)/$(TEXI)

CLEANFILES = $(TXT) $(HTML)

EXTRA_DIST = $(EXFILES) CMakeLists.txt

AM_CPPFLAGS = -I$(srcdir)/../../ginac -I../../ginac -DIN_GINAC

noinst_PROGRAMS = archive1 \
		  compile1 \
		  compile3 \
		  mystring \
		  derivative

archive1_SOURCES = archive1.cpp
archive1_LDADD = ../../ginac/libginac.la

compile1_SOURCES = compile1.cpp
compile1_LDADD = ../../ginac/libginac.la

# compile2 example uses the Cuba library: (http://www.feynarts.de)
# compile2_SOURCES = compile2.cpp
# compile2_LDADD = ../../ginac/libginac.la

compile3_SOURCES = compile3.cpp
compile3_LDADD = ../../ginac/libginac.la

mystring_SOURCES = mystring.cpp
mystring_LDADD = ../../ginac/libginac.la

derivative_SOURCES = derivative.cpp
derivative_LDADD = ../../ginac/libginac.la
