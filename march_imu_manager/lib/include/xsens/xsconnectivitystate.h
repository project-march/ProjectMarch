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

#ifndef XSCONNECTIVITYSTATE_H
#define XSCONNECTIVITYSTATE_H

#include "xdaconfig.h"

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief XsDevice connectivity state identifiers */
enum XsConnectivityState {
	XCS_Disconnected,		/*!< Device has disconnected, only limited informational functionality is available. */
	XCS_Rejected,			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
	XCS_PluggedIn,			/*!< Device is connected through a cable. */
	XCS_Wireless,			/*!< Device is connected wirelessly. */
	XCS_File,				/*!< Device is reading from a file. */
	XCS_Unknown,			/*!< Device is in an unknown state. */
};

/*! @} */
typedef enum XsConnectivityState XsConnectivityState;

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Convert the device state to a human readable string */
XDA_DLL_API const char *XsConnectivityState_toString(XsConnectivityState s);

#ifdef __cplusplus
} // extern "C"

#ifndef XSENS_NO_STL
namespace std {
template<typename _CharT, typename _Traits>
basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsConnectivityState const& xs)
{
	return (o << XsConnectivityState_toString(xs));
}
}
#endif
#endif

#endif	// file guard
