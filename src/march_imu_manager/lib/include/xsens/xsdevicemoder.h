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

#ifndef XSDEVICEMODER_H
#define XSDEVICEMODER_H

#include "xdaconfig.h"
#include <xsens/pstdint.h>
#include <xsens/xsdataformat.h>

#define XS_DEFAULT_UPDATE_RATE			100

struct XsDeviceModeR;

#ifdef __cplusplus
extern "C" {
#endif

XDA_DLL_API int XsDeviceModeR_updateRate(const struct XsDeviceModeR* thisPtr);
XDA_DLL_API void XsDeviceModeR_getPeriodAndSkipFactor(const struct XsDeviceModeR* thisPtr, uint16_t* period, uint16_t* skip);
XDA_DLL_API void XsDeviceModeR_setPeriodAndSkipFactor(struct XsDeviceModeR* thisPtr, uint16_t period, uint16_t skip);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief A structure for storing device modes. */
struct XsDeviceModeR {
	XsOutputMode		m_outputMode;		//!< The output mode
	XsOutputSettings	m_outputSettings;	//!< The output settings
	uint16_t			m_updateRate;		//!< The desired update rate

#ifdef __cplusplus
	//! default constructor, initializes to the given (default) MT settings
	explicit XsDeviceModeR(	const XsOutputMode mode = XS_DEFAULT_OUTPUT_MODE,
					const XsOutputSettings settings = XS_DEFAULT_OUTPUT_SETTINGS,
					const uint16_t rate = XS_DEFAULT_UPDATE_RATE)
	: m_outputMode(mode), m_outputSettings(settings), m_updateRate(rate)
	{}

	/*! \copydoc XsDeviceModeR_updateRate */
	inline int updateRate(void) const
	{
		return XsDeviceModeR_updateRate(this);
	}

	/*! \copydoc XsDeviceModeR_getPeriodAndSkipFactor */
	inline void getPeriodAndSkipFactor(uint16_t& period, uint16_t& skip) const
	{
		XsDeviceModeR_getPeriodAndSkipFactor(this, &period, &skip);
	}

	/*! \copydoc XsDeviceModeR_setPeriodAndSkipFactor */
	inline void setPeriodAndSkipFactor(uint16_t period, uint16_t skip)
	{
		XsDeviceModeR_setPeriodAndSkipFactor(this, period, skip);
	}

	//! \brief Check if all fields of the two structures are equal
	inline bool operator == (const XsDeviceModeR& dev) const
	{
		return	m_outputMode == dev.m_outputMode &&
				m_outputSettings == dev.m_outputSettings &&
				m_updateRate == dev.m_updateRate;
	}
#endif
};
typedef struct XsDeviceModeR XsDeviceModeR;

#endif // file guard
