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

#ifndef XSCOMCALLBACKOPTIONS_H
#define XSCOMCALLBACKOPTIONS_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Xda options, used to control the callback enabled in the COM object
	\details These options are used to specify whether the COM object should call 
	a specific callback. All the callbacks are disabled by default in order 
	to prevent memory leakage from unflushed buffers.
*/
enum XsComCallbackOptions {
		XSC_None					= 0			//!< all calbacks are disabled
		, XSC_LivePacket			= 0x0001	//!< live packet callback enable
		, XSC_LivePackets			= 0x0002	//!< live packets callback enable
		, XSC_RecordingPacket		= 0x0004	//!< recording packet callback enable
		, XSC_RecordingPackets		= 0x0008	//!< recording packets callback enable
		, XSC_All					= 0x001F	//!< all calbacks are enabled
};
/*! @} */
typedef enum XsComCallbackOptions XsComCallbackOptions;

#ifdef __cplusplus
//! \brief Logical OR operator for XsComCallbackOptions values
inline XsComCallbackOptions operator | (XsComCallbackOptions a, XsComCallbackOptions b)
{
	return (XsComCallbackOptions) ((int)a | (int)b);
}
//! \brief Logical AND operator for XsComCallbackOptions values
inline XsComCallbackOptions operator & (XsComCallbackOptions a, XsComCallbackOptions b)
{
	return (XsComCallbackOptions) ((int)a & (int)b);
}
//! \brief Logical XOR operator for XsComCallbackOptions values
inline XsComCallbackOptions operator ^ (XsComCallbackOptions a, XsComCallbackOptions b)
{
	return (XsComCallbackOptions) ((int)a ^ (int)b);
}
//! \brief Logical NEG operator for XsComCallbackOptions values
inline XsComCallbackOptions operator ~ (XsComCallbackOptions a)
{
	return (XsComCallbackOptions) (~(int)a);
}

#endif
#endif