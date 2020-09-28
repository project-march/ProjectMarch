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

#ifndef XSCALIBRATEDDATA_H
#define XSCALIBRATEDDATA_H

#include "xstypesconfig.h"
#include "xsvector3.h"

struct XsCalibratedData;

#ifdef __cplusplus
extern "C" {
#else
#define XSCALIBRATEDDATA_INITIALIZER {XSVECTOR3_INITIALIZER, XSVECTOR3_INITIALIZER, XSVECTOR3_INITIALIZER}
#endif

XSTYPES_DLL_API void XsCalibratedData_construct(struct XsCalibratedData* thisPtr, const XsReal* acc, const XsReal* gyr, const XsReal* mag);
XSTYPES_DLL_API void XsCalibratedData_destruct(struct XsCalibratedData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsCalibratedData
{
	XsVector3 m_acc;	//!< Accelerometer data
	XsVector3 m_gyr;	//!< Gyroscope data
	XsVector3 m_mag;	//!< Magnetometer data

#ifdef __cplusplus
	//! \brief Constructor \sa XsCalibratedData_construct
	inline XsCalibratedData()
	{}

	//! \brief Copy constructor, copies the values from \a other to this
	inline XsCalibratedData(const XsCalibratedData& other)
		: m_acc(other.m_acc)
		, m_gyr(other.m_gyr)
		, m_mag(other.m_mag)
	{
	}

	//! \brief Destructor
	inline ~XsCalibratedData()
	{}

	//! \brief Assignment operator, copies the values from \a other to this
	inline const XsCalibratedData& operator = (const XsCalibratedData& other)
	{
		if (this != &other)
		{
			m_acc = other.m_acc;
			m_gyr = other.m_gyr;
			m_mag = other.m_mag;
		}
		return *this;
	}
#endif
};
typedef struct XsCalibratedData XsCalibratedData;

#endif // file guard
