/*	WARNING: COPYRIGHT (C) 2016 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#ifndef XSTYPEDEFS_H
#define XSTYPEDEFS_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifndef XSENS_SINGLE_PRECISION
#include <stddef.h>
typedef double XsReal;	//!< Defines the floating point type used by the Xsens libraries
typedef size_t XsSize;	//!< XsSize must be unsigned number!
# ifndef PRINTF_SIZET_MODIFIER
#  if defined(XSENS_64BIT)
#	 if defined(__APPLE__)
#      define PRINTF_SIZET_MODIFIER "l"
#    else
#	   define PRINTF_SIZET_MODIFIER PRINTF_INT64_MODIFIER
#    endif
#  else
#    define PRINTF_SIZET_MODIFIER PRINTF_INT32_MODIFIER
#  endif
# endif // PRINTF_SIZET_MODIFIER
#else
typedef float XsReal;			//!< Defines the floating point type used by the Xsens libraries
typedef unsigned int XsSize;	//!< XsSize must be unsigned number!
#endif // XSENS_SINGLE_PRECISION


/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief These flags define the behaviour of data contained by Xsens data structures
	\details Normally, the user should never need to use these directly.
*/
enum XsDataFlags {
	 XSDF_None = 0				//!< No flag set
	,XSDF_Managed = 1			//!< The contained data should be managed (freed) by the object, when false, the object assumes the memory is freed by some other process after its destruction
	,XSDF_FixedSize = 2			//!< The contained data points to a fixed-size buffer, this allows creation of dynamic objects on the stack without malloc/free overhead.
	,XSDF_Empty = 4				//!< The object contains undefined data / should be considered empty. Usually only relevant when XSDF_FixedSize is also set, as otherwise the data pointer will be NULL and empty-ness is implicit.
};
/*! @} */
typedef enum XsDataFlags XsDataFlags;

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API const char *XsDataFlags_toString(XsDataFlags f);

#ifdef __cplusplus
} // extern "C"
/*! \brief \copybrief XsDataFlags_toString \sa XsDataFlags_toString */
inline const char *toString(XsDataFlags s)
{
	return XsDataFlags_toString(s);
}
#else
// define BOOL, TRUE and FALSE
#ifndef BOOL
typedef int BOOL;
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif
#endif // __cplusplus

#define XS_ENUM_TO_STR_CASE(value) case value: return #value;

#endif // file guard
