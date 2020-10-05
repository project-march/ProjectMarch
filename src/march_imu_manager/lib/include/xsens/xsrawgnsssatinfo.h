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

#ifndef XSRAWGNSSSATINFO_H
#define XSRAWGNSSSATINFO_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

#ifndef __cplusplus
#define XSSATINFO_INITIALIZER	{ 0, 0, 0, 0 }
#define XSRAWGNSSSATINFO_INITIALIZER { 0, 0, 0, 0, 0,  \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER }
#endif

/*! \addtogroup enums Global enumerations
   @{
*/
/*! \brief Xsens Satellite Info Flags
*/
enum XsSatInfoFlags
{
	XSIF_SignalQualityIndicator_Mask				= 0x7,
	XSIF_SignalQualityIndicator_NoSignal			= 0x0,
	XSIF_SignalQualityIndicator_Searching			= 0x1,
	XSIF_SignalQualityIndicator_Acquired			= 0x2,
	XSIF_SignalQualityIndicator_Unusable			= 0x3,
	XSIF_SignalQualityIndicator_CodeTimeOk			= 0x4,
	XSIF_SignalQualityIndicator_CodeCarrierTimeOk1	= 0x5,
	XSIF_SignalQualityIndicator_CodeCarrierTimeOk2	= 0x6,
	XSIF_SignalQualityIndicator_CodeCarrierTimeOk3	= 0x7,
	XSIF_UsedForNavigation_Mask						= 0x8,
	XSIF_UsedForNavigation_Used						= 0x8,
	XSIF_HealthFlag_Mask							= 0x30,
	XSIF_HealthFlag_Unknown							= 0x00,
	XSIF_HealthFlag_Healthy							= 0x10,
	XSIF_HealthFlag_Unhealthy						= 0x20,
	XSIF_Differential_Mask							= 0x40,
	XSIF_Differential_Available						= 0x40
};
/*! @} */
typedef enum XsSatInfoFlags XsSatInfoFlags;

//! \brief A container for information of one GNSS satellite
struct XsSatInfo
{
		uint8_t		m_gnssId;	//!< GNSS identifier
		uint8_t		m_svId;		//!< Satellite identifier
		uint8_t		m_cno;		//!< Carrier to noise ratio (signals strength)
		uint8_t		m_flags;	/*!<
									bits[0..2] : Signal quality indicator
										0 = No signal
										1 = Searching signal
										2 = Signal acquired
										3 = Signal detected but unusable
										4 = Code locked and time synchronized
										5, 6, 7 = Code and carrier locked and time synchronized
									bits[3] : Is set to 1 when the SV is being used for navigation
									bits[4..5] : SV health flag
										0 = unknown
										1 = healthy
										2 = unhealthy
									bits[6] : Is set to 1 when differential correction data is available
									bits[7] : reserved
									*/
};
typedef struct XsSatInfo XsSatInfo;

/*! \brief A container for GNSS Satellite Information
*/
struct XsRawGnssSatInfo
{
	uint32_t	m_itow;			//!< GPS time of week (ms)
	uint8_t		m_numSvs;		//!< Number of satellites
	uint8_t		m_res1;			//!< Reserved for future use (1)
	uint8_t		m_res2;			//!< Reserved for future use (2)
	uint8_t		m_res3;			//!< Reserved for future use (3)

	XsSatInfo m_satInfos[60]; //!< The information of all satellites, maximum 60

#ifdef __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsRawGnssSatInfo& b) const
	{
		if (m_itow		!= b.m_itow ||
			m_numSvs	!= b.m_numSvs ||
			m_res1		!= b.m_res1 ||
			m_res2		!= b.m_res2 ||
			m_res3		!= b.m_res3)
			return false;
		for (uint8_t i = 0; i < m_numSvs; ++i)
			if (m_satInfos[i].m_gnssId	!= b.m_satInfos[i].m_gnssId ||
				m_satInfos[i].m_svId	!= b.m_satInfos[i].m_svId ||
				m_satInfos[i].m_cno		!= b.m_satInfos[i].m_cno ||
				m_satInfos[i].m_flags	!= b.m_satInfos[i].m_flags)
				return false;
		return true;
	}
#endif
};
typedef struct XsRawGnssSatInfo XsRawGnssSatInfo;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif // file guard
