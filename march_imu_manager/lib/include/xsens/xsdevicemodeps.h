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

#ifndef XSDEVICEMODEPS_H
#define XSDEVICEMODEPS_H

#include "xdaconfig.h"
#include <xsens/pstdint.h>
#include <xsens/xsoutputmode.h>
#include <xsens/xsoutputsettings.h>

#define XS_DEFAULT_PERIOD			1152
#define XS_DEFAULT_SKIP				0

struct XsDeviceModePS;

#ifdef __cplusplus
extern "C" {
#endif

XDA_DLL_API int XsDeviceModePS_updateRate(const struct XsDeviceModePS* thisPtr);
XDA_DLL_API void XsDeviceModePS_setUpdateRate(struct XsDeviceModePS* thisPtr, int rate);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsDeviceModePS {
	XsOutputMode		m_outputMode;		//!< The output mode
	XsOutputSettings	m_outputSettings;	//!< The output settings
	uint16_t			m_period;			//!< The sample period in 1/115200 seconds
	uint16_t			m_skip;				//!< The output skip factor

#ifdef __cplusplus
	//! default constructor, initializes to the given (default) MT settings
	explicit XsDeviceModePS(	const XsOutputMode mode = XS_DEFAULT_OUTPUT_MODE,
					const XsOutputSettings settings = XS_DEFAULT_OUTPUT_SETTINGS,
					const uint16_t period = XS_DEFAULT_PERIOD,
					const uint16_t skip = XS_DEFAULT_SKIP)
	: m_outputMode(mode), m_outputSettings(settings), m_period(period), m_skip(skip)
	{}

	/*! \copydoc XsDeviceModePS_updateRate */
	inline int updateRate(void) const
	{
		return XsDeviceModePS_updateRate(this);
	}

	/*! \copydoc XsDeviceModePS_setUpdateRate */
	inline void setUpdateRate(int rate)
	{
		XsDeviceModePS_setUpdateRate(this, rate);
	}

	//! \brief Check if all fields of the two structures are equal
	inline bool operator == (const XsDeviceModePS& dev) const
	{
		return	m_outputMode == dev.m_outputMode &&
				m_outputSettings == dev.m_outputSettings &&
				m_period == dev.m_period &&
				m_skip == dev.m_skip;
	}
#endif
};
typedef struct XsDeviceModePS XsDeviceModePS;

#endif // file guard
