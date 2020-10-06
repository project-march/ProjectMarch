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

#ifndef XSDEVICEMODE_H
#define XSDEVICEMODE_H

#include <xsens/xstypesconfig.h>
#include <xsens/pstdint.h>
#include "xdaconfig.h"
#include "xsorientationmode.h"
#include "xsfloatformat.h"
#include "xscoordinatesystem.h"
#include "xscalibrateddatamode.h"
#include <xsens/xsoutputmode.h>
#include <xsens/xsoutputsettings.h>

struct XsDeviceMode;
struct XsDeviceModeR;
struct XsDeviceModePS;

#ifdef __cplusplus
#include "xsdevicemoder.h"
#include "xsdevicemodeps.h"
extern "C" {
#else
#define XSDEVICEMODE_INITIALIZER	{ XOM_None, XOS_Timestamp_PacketCounter, 0, 0 }
#endif

XDA_DLL_API void XsDeviceMode_setModeFlag(struct XsDeviceMode* thisPtr, XsOutputMode flag, int enabled);
XDA_DLL_API int XsDeviceMode_isModeFlagEnabled(const struct XsDeviceMode* thisPtr, XsOutputMode flag);
XDA_DLL_API void XsDeviceMode_setSettingsFlag(struct XsDeviceMode* thisPtr, XsOutputSettings flag, int enabled);
XDA_DLL_API int XsDeviceMode_isSettingsFlagEnabled(const struct XsDeviceMode* thisPtr, XsOutputSettings flag);
XDA_DLL_API double XsDeviceMode_sampleRate(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setUpdateRate(struct XsDeviceMode* thisPtr, int rate);
XDA_DLL_API uint16_t XsDeviceMode_updateRate(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setOrientationMode(struct XsDeviceMode* thisPtr, XsOrientationMode mode);
XDA_DLL_API XsOrientationMode XsDeviceMode_orientationMode(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setDataFormat(struct XsDeviceMode* thisPtr, XsFloatFormat ff);
XDA_DLL_API XsFloatFormat XsDeviceMode_dataFormat(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setCalibratedDataMode(struct XsDeviceMode* thisPtr, XsCalibratedDataMode mode);
XDA_DLL_API XsCalibratedDataMode XsDeviceMode_calibratedDataMode(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setAnalogInChannelEnabled(struct XsDeviceMode* thisPtr, int channelIndex, int enabled);
XDA_DLL_API int XsDeviceMode_isAnalogInChannelEnabled(const struct XsDeviceMode* thisPtr, int channelIndex);
XDA_DLL_API void XsDeviceMode_setCoordinateSystem(struct XsDeviceMode* thisPtr, XsCoordinateSystem coordinatesystem);
XDA_DLL_API XsCoordinateSystem XsDeviceMode_coordinateSystem(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setStatusOutputEnabled(struct XsDeviceMode* thisPtr, int enabled);
XDA_DLL_API int XsDeviceMode_isStatusOutputEnabled(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setTemperatureOutputEnabled(struct XsDeviceMode* thisPtr, int enabled);
XDA_DLL_API int XsDeviceMode_isTemperatureOutputEnabled(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setPositionDataEnabled(struct XsDeviceMode* thisPtr, int enabled);
XDA_DLL_API int XsDeviceMode_isPositionDataEnabled(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setVelocityDataEnabled(struct XsDeviceMode* thisPtr, int enabled);
XDA_DLL_API int XsDeviceMode_isVelocityDataEnabled(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setGpsPvtDataEnabled(struct XsDeviceMode* thisPtr, int enabled);
XDA_DLL_API int XsDeviceMode_isGpsPvtDataEnabled(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_setRawDataOutputEnabled(struct XsDeviceMode* thisPtr, int enabled);
XDA_DLL_API int XsDeviceMode_isRawDataOutputEnabled(const struct XsDeviceMode* thisPtr);
XDA_DLL_API void XsDeviceMode_toXsDeviceMode(const struct XsDeviceMode* thisPtr, struct XsDeviceModeR *modeR);
XDA_DLL_API void XsDeviceMode_toXsDeviceMode2(const struct XsDeviceMode* thisPtr, struct XsDeviceModePS *modePS);


#ifdef __cplusplus
} // extern "C"
#endif

struct XsDeviceMode {
#ifdef __cplusplus

	/*! \brief Construct a device mode with the given output \a rate in Hz. */
	inline explicit XsDeviceMode(int rate = 100)
		: m_outputMode(XOM_None)
		, m_outputSettings(XOS_Timestamp_PacketCounter)
		, m_period(0)
		, m_skip(0)
	{
		XsDeviceMode_setUpdateRate(this, rate);
	}

	/*! \brief Construct a device mode with the given \a sampleperiod and \a skipfactor. */
	inline explicit XsDeviceMode(uint16_t sampleperiod, uint16_t skipfactor)
		: m_outputMode(XOM_None)
		, m_outputSettings(XOS_Timestamp_PacketCounter)
		, m_period(sampleperiod)
		, m_skip(skipfactor)
	{
	}

	/*! \brief Construct a device mode object based on \a other. */
	inline XsDeviceMode(const XsDeviceMode& other)
		: m_outputMode(other.m_outputMode)
		, m_outputSettings(other.m_outputSettings)
		, m_period(other.m_period)
		, m_skip(other.m_skip)
	{
	}

	/*! \brief Construct a device mode from a XsDeviceModeR. */
	inline explicit XsDeviceMode(const XsDeviceModeR& other)
	{
		m_outputMode = other.m_outputMode;
		m_outputSettings = other.m_outputSettings;
		other.getPeriodAndSkipFactor(m_period, m_skip);
	}

	/*! \brief Construct a device mode from a XsDeviceModePS. */
	inline explicit XsDeviceMode(const XsDeviceModePS& other)
	{
		m_outputMode = other.m_outputMode;
		m_outputSettings = other.m_outputSettings;
		m_period = other.m_period;
		m_skip = other.m_skip;
	}

	/*! \brief Destroy this device mode. */
	inline ~XsDeviceMode()
	{
	}

	/*! \brief Assign values from \a other to this. */
	inline const XsDeviceMode& operator=(const XsDeviceMode& other)
	{
		if (this != &other)
		{
			m_outputMode = other.m_outputMode;
			m_outputSettings = other.m_outputSettings;
			m_period = other.m_period;
			m_skip = other.m_skip;
		}

		return *this;
	}

	/*! \copydoc XsDeviceMode_setModeFlag */
	inline void setModeFlag(XsOutputMode flag, bool enabled = true)
	{
		XsDeviceMode_setModeFlag(this, flag, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isModeFlagEnabled */
	inline bool isModeFlagEnabled(XsOutputMode flag) const
	{
		return (XsDeviceMode_isModeFlagEnabled(this, flag) != 0);
	}

	/*! \copydoc XsDeviceMode_setSettingsFlag */
	inline void setSettingsFlag(XsOutputSettings flag, bool enabled = true)
	{
		XsDeviceMode_setSettingsFlag(this, flag, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isSettingsFlagEnabled */
	inline bool isSettingsFlagEnabled(XsOutputSettings flag) const
	{
		return (XsDeviceMode_isSettingsFlagEnabled(this, flag) != 0);
	}

	/*! \copydoc XsDeviceMode_sampleRate */
	inline double sampleRate(void) const
	{
		return XsDeviceMode_sampleRate(this);
	}

	/*! \copydoc XsDeviceMode_updateRate */
	inline uint16_t updateRate(void) const
	{
		return XsDeviceMode_updateRate(this);
	}

	/*! \copydoc XsDeviceMode_setUpdateRate */
	inline void setUpdateRate(int rate)
	{
		XsDeviceMode_setUpdateRate(this, rate);
	}

	/*! \brief Set the period and skip factor directly
		\param sampleperiod		The sample period to use
		\param skipfactor		The skip factor to use
		\sa XsDeviceMode_setUpdateRate() */
	inline void setPeriodAndSkipFactor(int sampleperiod, int skipfactor)
	{
		assert(sampleperiod >= 0 && skipfactor >= 0);
		m_period = (uint16_t) sampleperiod;
		m_skip = (uint16_t) skipfactor;
	}

	/*! \brief Return the current period
		\returns The current period
		\sa XsDeviceMode_setUpdateRate()
	*/
	inline int period() const
	{
		return (int) m_period;
	}

	/*! \brief Return the current skip factor
		\returns The current skip factor
		\sa XsDeviceMode_setUpdateRate()
	*/
	inline int skipFactor() const
	{
		return m_skip;
	}

	/*! \brief Test if the \a other device mode is equal to this. */
	inline bool operator == (const XsDeviceMode& other) const
	{
		if (m_outputMode == other.m_outputMode && m_outputSettings == other.m_outputSettings && m_period == other.m_period && m_skip == other.m_skip)
			return true;

		return false;
	}

	/*! \brief Test if the \a other device mode is different from this. */
	inline bool operator != (const XsDeviceMode& other) const
	{
		return !((*this) == other);
	}

	/*! \copydoc XsDeviceMode_setOrientationMode */
	inline void setOrientationMode(XsOrientationMode mode)
	{
		XsDeviceMode_setOrientationMode(this, mode);
	}

	/*! \copydoc XsDeviceMode_orientationMode */
	inline XsOrientationMode orientationMode() const
	{
		return XsDeviceMode_orientationMode(this);
	}

	/*! \copydoc XsDeviceMode_setDataFormat */
	inline void setDataFormat(XsFloatFormat ff)
	{
		XsDeviceMode_setDataFormat(this, ff);
	}

	/*! \copydoc XsDeviceMode_dataFormat */
	inline XsFloatFormat dataFormat() const
	{
		return XsDeviceMode_dataFormat(this);
	}

	/*! \copydoc XsDeviceMode_setCalibratedDataMode */
	inline void setCalibratedDataMode(XsCalibratedDataMode mode)
	{
		XsDeviceMode_setCalibratedDataMode(this, mode);
	}

	/*! \copydoc XsDeviceMode_calibratedDataMode */
	inline XsCalibratedDataMode calibratedDataMode() const
	{
		return XsDeviceMode_calibratedDataMode(this);
	}

	/*! \copydoc XsDeviceMode_setAnalogInChannelEnabled */
	inline void setAnalogInChannelEnabled(int channelIndex, bool enabled)
	{
		XsDeviceMode_setAnalogInChannelEnabled(this, channelIndex, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isAnalogInChannelEnabled */
	inline bool isAnalogInChannelEnabled(int channelIndex) const
	{
		return (XsDeviceMode_isAnalogInChannelEnabled(this, channelIndex) != 0);
	}

	/*! \copydoc XsDeviceMode_setCoordinateSystem */
	inline void setCoordinateSystem(XsCoordinateSystem coordinatesystem)
	{
		XsDeviceMode_setCoordinateSystem(this, coordinatesystem);
	}

	/*! \brief \copybrief XsDeviceMode_coordinateSystem */
	inline XsCoordinateSystem coordinateSystem() const
	{
		return XsDeviceMode_coordinateSystem(this);
	}

	/*! \copydoc XsDeviceMode_setStatusOutputEnabled */
	inline void setStatusOutputEnabled(bool enabled)
	{
		XsDeviceMode_setStatusOutputEnabled(this, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isStatusOutputEnabled */
	inline bool isStatusOutputEnabled() const
	{
		return (XsDeviceMode_isStatusOutputEnabled(this) != 0);
	}

	/*! \copydoc XsDeviceMode_setTemperatureOutputEnabled  */
	inline void setTemperatureOutputEnabled(bool enabled)
	{
		XsDeviceMode_setTemperatureOutputEnabled(this, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isTemperatureOutputEnabled  */
	inline bool isTemperatureOutputEnabled() const
	{
		return (XsDeviceMode_isTemperatureOutputEnabled(this) != 0);
	}

	/*! \copydoc XsDeviceMode_setPositionDataEnabled  */
	inline void setPositionDataEnabled(bool enabled)
	{
		XsDeviceMode_setPositionDataEnabled(this, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isPositionDataEnabled */
	inline bool isPositionDataEnabled() const
	{
		return (XsDeviceMode_isPositionDataEnabled(this) != 0);
	}

	/*! \copydoc XsDeviceMode_setVelocityDataEnabled */
	inline void setVelocityDataEnabled(bool enabled)
	{
		XsDeviceMode_setVelocityDataEnabled(this, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isVelocityDataEnabled */
	inline bool isVelocityDataEnabled() const
	{
		return (XsDeviceMode_isVelocityDataEnabled(this) != 0);
	}

	/*! \copydoc XsDeviceMode_setGpsPvtDataEnabled */
	inline void setGpsPvtDataEnabled(bool enabled)
	{
		XsDeviceMode_setGpsPvtDataEnabled(this, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isGpsPvtDataEnabled */
	inline bool isGpsPvtDataEnabled() const
	{
		return (XsDeviceMode_isGpsPvtDataEnabled(this) != 0);
	}

	/*! \copydoc XsDeviceMode_setRawDataOutputEnabled */
	inline void setRawDataOutputEnabled(bool enabled)
	{
		XsDeviceMode_setRawDataOutputEnabled(this, (int)enabled);
	}

	/*! \copydoc XsDeviceMode_isRawDataOutputEnabled */
	inline bool isRawDataOutputEnabled() const
	{
		return (XsDeviceMode_isRawDataOutputEnabled(this) != 0);
	}

	/*! \brief \copybrief XsDeviceMode_toXsDeviceMode \sa XsDeviceMode_toXsDeviceMode
		\returns the XsDeviceModeR object
		\sa XsDeviceModeR
	*/
	inline XsDeviceModeR toXsDeviceMode() const
	{
		struct XsDeviceModeR xdmr;
		XsDeviceMode_toXsDeviceMode(this, &xdmr);
		return xdmr;
	}

	/*! \brief \copybrief XsDeviceMode_toXsDeviceMode2 \sa XsDeviceMode_toXsDeviceMode2
		\returns the XsDeviceModePS object
		\sa XsDeviceModeR
	*/
	inline XsDeviceModePS toXsDeviceMode2() const
	{
		struct XsDeviceModePS xdmps;
		XsDeviceMode_toXsDeviceMode2(this, &xdmps);
		return xdmps;
	}

	/*! \brief Returns the output mode
		\returns The output mode
		\see XsOutputMode
	*/
	inline XsOutputMode outputMode() const
	{
		return m_outputMode;
	}

	/*! \brief Set the output \a mode
		\param mode The output mode to set
		\see XsOutputMode
	*/
	inline void setOutputMode(XsOutputMode mode)
	{
		m_outputMode = mode;
	}

	/*! \brief Returns the output settings
		\returns The output settings
		\see XsOutputSettings
	*/
	inline XsOutputSettings outputSettings() const
	{
		return m_outputSettings;
	}

	/*! \brief Set the output \a settings
		\param settings The outputsettings to set
		\see XsOutputSettings
	*/
	inline void setOutputSettings(XsOutputSettings settings)
	{
		m_outputSettings = settings;
	}

private:
//! \protectedsection
#endif

	// Legacy output settings
	XsOutputMode m_outputMode;			//!< The output mode (bitfield that contains what should be output)
	XsOutputSettings m_outputSettings;	//!< The output settings (bitfield that conatins how the output should be formatted)
	uint16_t m_period;					//!< The period between updates in 115200Hz intervals
	uint16_t m_skip;					//!< The skip factor wrt \a m_period
};

typedef struct XsDeviceMode XsDeviceMode;

#endif // file guard
