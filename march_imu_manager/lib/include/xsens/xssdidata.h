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

#ifndef XSSDIDATA_H
#define XSSDIDATA_H

#include "xstypesconfig.h"
#include "xsvector3.h"
#include "xsquaternion.h"

struct XsSdiData;

#ifdef __cplusplus
extern "C" {
#else
#define XSSDIDATA_INITIALIZER {XSQUATERNION_INITIALIZER, XSVECTOR3_INITIALIZER}
#endif

XSTYPES_DLL_API void XsSdiData_construct(struct XsSdiData* thisPtr, const XsReal* orientationIncrement, const XsReal* velocityIncrement);
XSTYPES_DLL_API void XsSdiData_destruct(struct XsSdiData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsSdiData
{
#ifdef __cplusplus
	//! \brief Construct an empty object
	inline XsSdiData()
	{
	}

	//! \brief Construct an initialized object
	inline XsSdiData(const XsQuaternion& dq, const XsVector& dv)
		: m_orientationIncrement(dq)
		, m_velocityIncrement(dv)
	{
	}

	//! \brief Copy constructor
	inline XsSdiData(const XsSdiData& other)
		: m_orientationIncrement(other.m_orientationIncrement)
		, m_velocityIncrement(other.m_velocityIncrement)
	{
	}

	//! \brief Assignment operator
	inline const XsSdiData& operator=(const XsSdiData& other)
	{
		if (this != &other)
		{
			m_orientationIncrement = other.m_orientationIncrement;
			m_velocityIncrement = other.m_velocityIncrement;
		}
		return *this;
	}

	//! \brief Clear the object so it contains unity data
	inline void zero()
	{
		m_orientationIncrement = XsQuaternion::identity();
		m_velocityIncrement.zero();
	}

	//! \brief Returns the contained orientation increment
	inline const XsQuaternion& orientationIncrement() const
	{
		return m_orientationIncrement;
	}

	//! \brief Update the contained orientation increment
	inline void setOrientationIncrement(const XsQuaternion& dq)
	{
		m_orientationIncrement = dq;
	}

	//! \brief Returns the contained velocity increment
	inline const XsVector3& velocityIncrement() const
	{
		return m_velocityIncrement;
	}

	//! \brief Update the contained velocity increment
	inline void setVelocityIncrement(const XsVector& dv)
	{
		m_velocityIncrement = dv;
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsSdiData& other) const
	{
		return	m_orientationIncrement == other.m_orientationIncrement &&
				m_velocityIncrement == other.m_velocityIncrement;
	}

private:
#endif

	XsQuaternion m_orientationIncrement;	//!< The orientation increment
	XsVector3    m_velocityIncrement;		//!< The velocity increment
};

typedef struct XsSdiData XsSdiData;

#endif	// file guard
