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

#ifndef XSSELFTESTRESULT_H
#define XSSELFTESTRESULT_H

#include <xsens/pstdint.h>

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Enumeration of bits that describe whether the various self-tests succeeded
	\see XsSelfTestResult
*/
enum XsSelfTestFlag {
	 XSTF_AccX	= 1<<0
	,XSTF_AccY	= 1<<1
	,XSTF_AccZ	= 1<<2
	,XSTF_GyrX	= 1<<3
	,XSTF_GyrY	= 1<<4
	,XSTF_GyrZ	= 1<<5
	,XSTF_MagX	= 1<<6
	,XSTF_MagY	= 1<<7
	,XSTF_MagZ	= 1<<8
};
/*! @} */
typedef enum XsSelfTestFlag XsSelfTestFlag;

/*! \brief Contains the results of a self-test performed by an Xsens device
*/
struct XsSelfTestResult {
	uint16_t m_flags;		//!< Flags that specify which tests have passed

#ifdef __cplusplus
	//! \brief Create a new %XsSelfTestResult from supplied flags
	static inline XsSelfTestResult create(uint16_t resultFlags)
	{
		XsSelfTestResult tmp = { resultFlags };
		return tmp;
	}

	//! \brief Returns whether the accelerometer x-axis passed (true) or failed (false) its self-test
	inline bool accX() const
	{
		return (m_flags & XSTF_AccX) != 0;
	}

	//! \brief Returns whether the accelerometer y-axis passed (true) or failed (false) its self-test
	inline bool accY() const
	{
		return (m_flags & XSTF_AccY) != 0;
	}

	//! \brief Returns whether the accelerometer z-axis passed (true) or failed (false) its self-test
	inline bool accZ() const
	{
		return (m_flags & XSTF_AccZ) != 0;
	}

	//! \brief Returns whether the gyroscope x-axis passed (true) or failed (false) its self-test
	inline bool gyrX() const
	{
		return (m_flags & XSTF_GyrX) != 0;
	}

	//! \brief Returns whether the gyroscope y-axis passed (true) or failed (false) its self-test
	inline bool gyrY() const
	{
		return (m_flags & XSTF_GyrY) != 0;
	}

	//! \brief Returns whether the gyroscope z-axis passed (true) or failed (false) its self-test
	inline bool gyrZ() const
	{
		return (m_flags & XSTF_GyrZ) != 0;
	}

	//! \brief Returns whether the magnetometer x-axis passed (true) or failed (false) its self-test
	inline bool magX() const
	{
		return (m_flags & XSTF_MagX) != 0;
	}

	//! \brief Returns whether the magnetometer y-axis passed (true) or failed (false) its self-test
	inline bool magY() const
	{
		return (m_flags & XSTF_MagY) != 0;
	}

	//! \brief Returns whether the magnetometer z-axis passed (true) or failed (false) its self-test
	inline bool magZ() const
	{
		return (m_flags & XSTF_MagZ) != 0;
	}

#endif
};

typedef struct XsSelfTestResult XsSelfTestResult;

#endif	// file guard
