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

#ifndef XSRAWGPSDOP_H
#define XSRAWGPSDOP_H

#include "pstdint.h"

#pragma pack(push, 1)

#ifndef __cplusplus
#define XSRAWGPSDOP_INITIALIZER { 0, 0, 0, 0, 0, 0, 0, 0 }
#endif

/*! \brief A container for NAV-DOP data
	\details DOP values are dimensionless.
	All dop values are scaled by a factor of 100. that is, if the unit transmits a value of e.g. 156,
	it means that the DOP value is 1.56.
	\deprecated
*/
struct XsRawGpsDop
{
	uint32_t	m_itow;		//!< Gps time of week (ms)
	uint16_t	m_gdop;		//!< Geometric DOP
	uint16_t	m_pdop;		//!< Position DOP
	uint16_t	m_tdop;		//!< Time DOP
	uint16_t	m_vdop;		//!< Vertical DOP
	uint16_t	m_hdop;		//!< Horizontal DOP
	uint16_t	m_ndop;		//!< Northing DOP
	uint16_t	m_edop;		//!< Easting DOP
};
typedef struct XsRawGpsDop XsRawGpsDop;

#pragma pack(pop)

#ifdef __cplusplus
inline bool operator == (XsRawGpsDop const& a, XsRawGpsDop const& b)
{
	return memcmp(&a, &b, sizeof(XsRawGpsDop)) == 0;
}
#endif

#endif // file guard
