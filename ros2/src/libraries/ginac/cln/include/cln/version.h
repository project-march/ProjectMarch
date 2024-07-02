/* CLN version information */

#ifndef _CL_VERSION_H
#define _CL_VERSION_H

/* Major version number of CLN */
#define CL_VERSION_MAJOR 1
/* Minor version number of CLN */
#define CL_VERSION_MINOR 3
/* Patchlevel version number of CLN */
#define CL_VERSION_PATCHLEVEL 7

/**
 * Libtool's library version information for CLN.
 * (Not to be confused with CLN's release version.)
 * Making new releases:
 * - increment cl_lt_revision,
 * - if any interfaces have been added, removed, or changed, then increment
 *   cl_lt_current and set cl_lt_revision to 0,
 * - if any interfaces have been added, then increment cl_lt_age,
 * - if any interfaces have been removed, set cl_lt_age to 0.
 * (On many systems, $(cl_lt_current):$(cl_lt_revision):$(cl_lt_age) results in
 *  libcln.so.$(cl_lt_current)-$(cl_lt_age).)
 */
#define CL_LT_CURRENT 6
#define CL_LT_AGE 0
#define CL_LT_REVISION 7

#define CL_STR_HELPER(x) #x
#define CL_STR(x) CL_STR_HELPER(x)

#define CL_VERSION \
	CL_STR(CL_VERSION_MAJOR) "." \
	CL_STR(CL_VERSION_MINOR) "." \
	CL_STR(CL_VERSION_PATCHLEVEL)

namespace cln {

extern const int version_major;
extern const int version_minor;
extern const int version_patchlevel;

}  // namespace cln

#endif /* _CL_VERSION_H */

