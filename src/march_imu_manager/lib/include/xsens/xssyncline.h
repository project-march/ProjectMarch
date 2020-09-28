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

#ifndef XSSYNCLINE_H
#define XSSYNCLINE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Synchronization line identifiers */
enum XsSyncLine
{
	XSL_Inputs,
	XSL_In1 = XSL_Inputs,				/*!< \brief Sync In 1 \remark Applies to Awinda Station and Mt */
	XSL_In2,							/*!< \brief Sync In 2 \remark Applies to Awinda Station */
	XSL_Bi1In,							/*!< \brief Bidirectional Sync 1 In \remark Applies to Xbus Master */
	XSL_ClockIn,						/*!< \brief Clock synchronisation input \remark Applies to Mk4 */
	XSL_CtsIn,							/*!< \brief RS232 CTS sync in \remark Applies to Xbus Master */
	XSL_GnssClockIn,					/*!< \brief Clock synchronisation input line attached to internal GPS unit \remark Applies to Mk4*/
	XSL_GpsClockIn = XSL_GnssClockIn,	/*!< \brief Clock synchronisation input line attached to internal GPS unit \remark Applies to Mk4 \deprecated */
	XSL_ExtTimepulseIn,					/*!< \brief External time pulse input (e.g. for external GNSS unit) \remark Applies to Mk4 with external device*/
	XSL_ReqData,						/*!< \brief Serial data sync option, use \a XMID_ReqData message id for this \remark Applies to Mk4*/
	XSL_Outputs,
	XSL_Out1 = XSL_Outputs,				/*!< \brief Sync Out 1 \remark Applies to Awinda Station and Mt */
	XSL_Out2,							/*!< \brief Sync Out 2 \remark Applies to Awinda Station */
	XSL_Bi1Out,							/*!< \brief Bidirectional Sync 1 Out \remark Applies to Xbus Master */
	XSL_RtsOut,							/*!< \brief RS232 RTS sync out \remark Applies to Xbus Master */

	XSL_Invalid
};
/*! @} */
typedef enum XsSyncLine XsSyncLine;

#endif // file guard
