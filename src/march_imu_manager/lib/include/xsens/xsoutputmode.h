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

#ifndef XSOUTPUTMODE_H
#define XSOUTPUTMODE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
//! Bit values for legacy output mode
enum XsOutputMode {
	XOM_None			= 0x0000,
	XOM_Temperature		= 0x0001,
	XOM_Calibrated		= 0x0002,
	XOM_Orientation		= 0x0004,
	XOM_Auxiliary		= 0x0008,
	XOM_Position		= 0x0010,
	XOM_Velocity		= 0x0020,
	XOM_Sdi				= 0x0200,
	XOM_Status			= 0x0800,
	XOM_GpsPvt_Pressure	= 0x1000,
	XOM_Reserved		= 0x2000,
	XOM_Raw				= 0x4000,
	XOM_Mt9				= 0x8000
};
/*! @} */
typedef enum XsOutputMode XsOutputMode;

// Extended (analog) Output Modes
#define XS_EXTOUTPUTMODE_DISABLED			0x0000
#define XS_EXTOUTPUTMODE_EULER				0x0001

#define XS_DEFAULT_OUTPUT_MODE			XOM_Orientation

#ifdef __cplusplus
/*! \brief Allow logical or of XsOutputMode to be a valid XsOutputMode value */
inline XsOutputMode operator | (XsOutputMode a, XsOutputMode b)
{
	return (XsOutputMode) ((unsigned short) a | (unsigned short) b);
}

/*! \brief Allow logical and of XsOutputMode to be a valid XsOutputMode value */
inline XsOutputMode operator & (XsOutputMode a, XsOutputMode b)
{
	return (XsOutputMode) ((unsigned short) a & (unsigned short) b);
}

/*! \brief Allow logical inversion of XsOutputMode to be a valid XsOutputMode value */
inline XsOutputMode operator ~ (XsOutputMode a)
{
	return (XsOutputMode) ~((unsigned short)a);
}
#endif

#endif // XSOUTPUTMODE_H
