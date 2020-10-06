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

#ifndef XSBAUD_H
#define XSBAUD_H

#include "xstypesconfig.h"


/*!	\addtogroup enums Global enumerations
	@{
*/

#include "xsbaudcode.h"
#include "xsbaudrate.h"

/*! @} */

typedef enum XsBaudCode XsBaudCode;
typedef enum XsBaudRate XsBaudRate;

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API XsBaudRate XsBaud_codeToRate(XsBaudCode baudcode);
XSTYPES_DLL_API XsBaudCode XsBaud_rateToCode(XsBaudRate baudrate);
XSTYPES_DLL_API int XsBaud_rateToNumeric(XsBaudRate baudrate);
XSTYPES_DLL_API XsBaudRate XsBaud_numericToRate(int numeric);

#ifdef __cplusplus
} // extern "C"

/*! \namespace XsBaud
	\brief Namespace for Baud rate and Baud code constants and conversions
*/
namespace XsBaud {
	/*! \copydoc XsBaud_codeToRate */
	inline XsBaudRate codeToRate(XsBaudCode baudcode)
	{
		return XsBaud_codeToRate(baudcode);
	}
	/*! \copydoc XsBaud_rateToCode */
	inline XsBaudCode rateToCode(XsBaudRate baudrate)
	{
		return XsBaud_rateToCode(baudrate);
	}
	/*! \copydoc XsBaud_rateToNumeric */
	inline int rateToNumeric(XsBaudRate baudrate)
	{
		return XsBaud_rateToNumeric(baudrate);
	}
	/*! \copydoc XsBaud_numericToRate*/
	inline XsBaudRate numericToRate(int numeric)
	{
		return XsBaud_numericToRate(numeric);
	}
}

#ifndef XSENS_NO_STL
#include <ostream>

namespace std {
template<typename _CharT, typename _Traits>
basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsBaudRate const& xd)
{
	return (o << XsBaud::rateToNumeric(xd));
}
}
#endif

#endif

#endif // file guard
