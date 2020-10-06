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

#ifndef XSRAWGNSSPVTDATA_H
#define XSRAWGNSSPVTDATA_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

#ifndef __cplusplus
#define XSRAWGNSSPVTDATA_INITIALIZER { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#endif

/*! \brief A container for GNSS Position Velocity and Time data
*/
struct XsRawGnssPvtData
{
	uint32_t	m_itow;		//!< GPS time of week (ms)
	uint16_t	m_year;		//!< Year (UTC)
	uint8_t		m_month;	//!< Month (UTC)
	uint8_t		m_day;		//!< Day of Month (UTC)
	uint8_t		m_hour;		//!< Hour of day 0..23 (UTC)
	uint8_t		m_min;		//!< Minute of hour 0..59 (UTC)
	uint8_t		m_sec;		//!< Seconds of minute 0..60 (UTC)
	uint8_t		m_valid;	/*!< Validity Flags
								bit(0) = Set if UTC Date is valid
								bit(1) = Set if UTC Time of Day if valid
								bit(2) = Set if UTC Time of Day has been fully resolved (no seconds uncertainty)
							*/
	uint32_t	m_tAcc;		//!< Time accuracy estimate (ns) (UTC)
	int32_t		m_nano;		//!< Fraction of second (ns) -1e9..1e9 (UTC)
	uint8_t		m_fixType;	/*!< GNSSfix Type, range 0..5
								0x00 = No Fix
								0x01 = Dead Reckoning only
								0x02 = 2D-Fix
								0x03 = 3D-Fix
								0x04 = GNSS + dead reckoning combined
								0x05 = Time only fix
								0x06..0xff: reserved
							*/
	uint8_t		m_flags;	/*!< Fix Status Flags
								bit(0) = Set if there is a valid fix (i.e. within DOP & accuracy masks)
								bit(1) = Set if differential corrections were applied
								bit(2..4) = Reserved (Ignore)
								bit(5) = Set if heading of vehicle is valid
								*/

	uint8_t		m_numSv;	//!< Number of satellites used in Nav Solution
	uint8_t		m_res1;		//!< Reserved for future use (1)
	int32_t		m_lon;		//!< Longitude (deg) (scaling 1e-7)
	int32_t		m_lat;		//!< Latitude (deg) (scaling 1e-7)
	int32_t		m_height;	//!< Height above ellipsoid (mm)
	int32_t		m_hMsl;		//!< Height above mean sea level (mm)

	uint32_t	m_hAcc;		//!< Horizontal accuracy estimate (mm)
	uint32_t	m_vAcc;		//!< Vertical accuracy estimate (mm)

	int32_t		m_velN;		//!< NED north velocity (mm/s)
	int32_t		m_velE;		//!< NED east velocity (mm/s)
	int32_t		m_velD;		//!< NED down velocity (mm/s)
	int32_t		m_gSpeed;	//!< 2D Ground Speed (mm/s)
	int32_t		m_headMot;	//!< 2D Heading of motion (deg) (scaling 1e-5)

	uint32_t	m_sAcc;		//!< Speed accuracy estimate (mm/s)
	uint32_t	m_headAcc;	//!< Heading accuracy estimate (both motion and vehicle) (deg) (scaling 1-e5)
	int32_t		m_headVeh;	//!< 2D Heading of vehicle (deg) (scaling 1e-5)

	uint16_t	m_gdop;		//!< Geometric DOP (scaling 0.01)
	uint16_t	m_pdop;		//!< Position DOP (scaling 0.01)
	uint16_t	m_tdop;		//!< Time DOP (scaling 0.01)
	uint16_t	m_vdop;		//!< Vertical DOP (scaling 0.01)
	uint16_t	m_hdop;		//!< Horizontal DOP (scaling 0.01)
	uint16_t	m_ndop;		//!< Northing DOP (scaling 0.01)
	uint16_t	m_edop;		//!< Easting DOP (scaling 0.01)

#ifdef __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsRawGnssPvtData& other) const
	{
		// direct memory comparison is allowed because all fields line up properly
		return memcmp(this, &other, sizeof(XsRawGnssPvtData)) == 0;
	}
#endif

};
typedef struct XsRawGnssPvtData XsRawGnssPvtData;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif // file guard
