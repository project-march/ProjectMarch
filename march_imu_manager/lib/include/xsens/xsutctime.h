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

#ifndef XSUTCTIME_H
#define XSUTCTIME_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C"
{
#else
#define XSUTCTIME_INITIALIZER	{ 0, 0 ,0, 0, 0, 0, 0, 0}
#endif
struct XsUtcTime;

XSTYPES_DLL_API void XsUtcTime_currentTime(struct XsUtcTime * now);

#ifdef __cplusplus
} // extern "C"
#endif

/*! \brief A structure for storing UTC Time values. */
struct XsUtcTime {
	uint32_t	m_nano;		//!< \brief Nanosecond part of the time
	uint16_t	m_year;		//!< \brief The year (if date is valid)
	uint8_t		m_month;	//!< \brief The month (if date is valid)
	uint8_t		m_day;  	//!< \brief The day of the month (if date is valid)
	uint8_t		m_hour;		//!< \brief The hour (if time is valid)
	uint8_t		m_minute;	//!< \brief The minute (if time is valid)
	uint8_t		m_second;	//!< \brief The second (if time is valid)
	uint8_t		m_valid;	//!< \brief Validity indicator \details When set to 1, the time is valid, when set to 2, the time part is valid, but the date may not be valid. when set to 0, the time is invalid and should be ignored.

#ifdef __cplusplus
	/*! \copydoc XsUtcTime_currentTime
	   \return The current UTC Time
	*/
	inline static XsUtcTime currentTime()
	{
		XsUtcTime tmp;
		XsUtcTime_currentTime(&tmp);
		return tmp;
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsUtcTime& other) const
	{
		return m_nano	== other.m_nano &&
			   m_year	== other.m_year &&
			   m_month	== other.m_month &&
			   m_day  	== other.m_day &&
			   m_hour	== other.m_hour &&
			   m_minute	== other.m_minute &&
			   m_second	== other.m_second &&
			   m_valid	== other.m_valid;
	}
#endif
};
typedef struct XsUtcTime XsUtcTime;

#endif // file guard
