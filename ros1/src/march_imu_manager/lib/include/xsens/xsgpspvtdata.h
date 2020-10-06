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

#ifndef XSGPSPVTDATA_H
#define XSGPSPVTDATA_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C" {
#else
#define XSGPSPVTDATA_INITIALIZER	{ 0,255,0,0,0,0,0,0,0,0,0,0,255 }
#endif

struct XsGpsPvtData;

XSTYPES_DLL_API void XsGpsPvtData_destruct(struct XsGpsPvtData* thisPtr);
XSTYPES_DLL_API int XsGpsPvtData_empty(const struct XsGpsPvtData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief Data from the GPS unit of a legacy MTi-G.*/
struct XsGpsPvtData {
	uint16_t	m_pressure;		//!< The pressure measurement in units of 2 Pascal, only valid if m_pressureAge is not 255
	uint8_t		m_pressureAge;	//!< The age of the pressure measurement in packets. When it decreases relative to the previous packet, it indicates that new data is available.
	uint32_t	m_itow;			//!< Integer time of week in ms
	int32_t		m_latitude;		//!< Latitude in 1e-7 degrees
	int32_t		m_longitude;	//!< Longitude in 1e-7 degrees
	int32_t		m_height;		//!< Height in mm
	int32_t		m_veln;			//!< Velocity towards north in cm/s
	int32_t		m_vele;			//!< Velocity towards east in cm/s
	int32_t		m_veld;			//!< Velocity towards down in cm/s
	uint32_t	m_hacc;			//!< Horizontal accuracy estimate, expected error standard deviation in mm
	uint32_t	m_vacc;			//!< Vertical accuracy estimate, expected error standard deviation in mm
	uint32_t	m_sacc;			//!< Speed accuracy estimate, expected error standard deviation in cm/s
	uint8_t		m_gpsAge;		//!< The age of the GPS measurement in packets. When it decreases relative to the previous packet, it indicates that new data is available.

#ifdef __cplusplus
	/*! \brief \copybrief XsGpsPvtData_destruct */
	inline void clear()
	{
		XsGpsPvtData_destruct(this);
	}

	/*! \brief \copybrief XsGpsPvtData_empty */
	inline bool empty() const
	{
		return 0 != XsGpsPvtData_empty(this);
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsGpsPvtData& other) const
	{
		return	m_pressure		== other.m_pressure &&
				m_pressureAge	== other.m_pressureAge &&
				m_itow			== other.m_itow &&
				m_latitude		== other.m_latitude &&
				m_longitude		== other.m_longitude &&
				m_height		== other.m_height &&
				m_veln			== other.m_veln &&
				m_vele			== other.m_vele &&
				m_veld			== other.m_veld &&
				m_hacc			== other.m_hacc &&
				m_vacc			== other.m_vacc &&
				m_sacc			== other.m_sacc &&
				m_gpsAge		== other.m_gpsAge;
	}
#endif
};

typedef struct XsGpsPvtData XsGpsPvtData;

#endif // file guard
