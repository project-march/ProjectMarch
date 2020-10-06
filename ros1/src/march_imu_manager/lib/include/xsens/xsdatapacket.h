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

#ifndef XSDATAPACKET_H
#define XSDATAPACKET_H

#include "xstypedefs.h"
#include "pstdint.h"
#include "xsmessage.h"
#include "xstimestamp.h"
#include "xsdataidentifier.h"
#include "xsushortvector.h"
#include "xsscrdata.h"
#include "xscalibrateddata.h"
#include "xsgpspvtdata.h"
#include "xspressure.h"
#include "xssdidata.h"
#include "xsvector.h"
#include "xsquaternion.h"
#include "xsmatrix.h"
#include "xseuler.h"
#include "xsanalogindata.h"
#include "xsutctime.h"
#include "xsrawgpsdop.h"
#include "xsrawgpssol.h"
#include "xsrawgpssvinfo.h"
#include "xsrawgpstimeutc.h"
#include "xsrawgnsspvtdata.h"
#include "xsrawgnsssatinfo.h"
#include "xsdeviceid.h"
#include "xsrange.h"
#include "xstriggerindicationdata.h"
#include "xssnapshot.h"

#ifndef XSNOEXPORT
#define XSNOEXPORT
#endif

struct XsDataPacket;
struct XSNOEXPORT DataPacketPrivate;
#ifdef __cplusplus
extern "C"
{
#else
typedef struct XsDataPacket XsDataPacket;
//#define XSDATAPACKET_INITIALIZER	{ 0, 0, XSDEVICEID_INITIALIZER, -1 } //Use XsDataPacket_construct in all cases because of dynamic initialization
#endif

XSTYPES_DLL_API void XsDataPacket_construct(XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_destruct(XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_clear(XsDataPacket* thisPtr, XsDataIdentifier id);
XSTYPES_DLL_API void XsDataPacket_copy(XsDataPacket* copy, XsDataPacket const* src);
XSTYPES_DLL_API void XsDataPacket_swap(XsDataPacket* thisPtr, XsDataPacket* other);
XSTYPES_DLL_API int XsDataPacket_empty(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_itemCount(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setMessage(XsDataPacket* thisPtr, const XsMessage* msg);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_dataFormat(const XsDataPacket* thisPtr, XsDataIdentifier id);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawAcceleration(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawAccelerationConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawAcceleration(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawGyroscopeData(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawGyroscopeDataConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGyroscopeData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGyroscopeData(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawGyroscopeTemperatureData(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawGyroscopeTemperatureDataConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGyroscopeTemperatureData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGyroscopeTemperatureData(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawMagneticField(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawMagneticFieldConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawMagneticField(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawMagneticField(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API uint16_t XsDataPacket_rawTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsRawTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawTemperature(XsDataPacket* thisPtr, uint16_t temp);
XSTYPES_DLL_API XsScrData* XsDataPacket_rawData(const XsDataPacket* thisPtr, XsScrData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawData(XsDataPacket* thisPtr, const XsScrData* data);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedAcceleration(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedGyroscopeData(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedGyroscopeData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedGyroscopeData(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedMagneticField(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedMagneticField(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedMagneticField(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsCalibratedData* XsDataPacket_calibratedData(const XsDataPacket* thisPtr, XsCalibratedData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedData(XsDataPacket* thisPtr, const XsCalibratedData* data);
XSTYPES_DLL_API XsQuaternion* XsDataPacket_orientationQuaternion(const XsDataPacket* thisPtr, XsQuaternion* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API void XsDataPacket_setOrientationQuaternion(XsDataPacket* thisPtr, const XsQuaternion* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API XsEuler* XsDataPacket_orientationEuler(const XsDataPacket* thisPtr, XsEuler* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API void XsDataPacket_setOrientationEuler(XsDataPacket* thisPtr, const XsEuler* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API XsMatrix* XsDataPacket_orientationMatrix(const XsDataPacket* thisPtr, XsMatrix* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API void XsDataPacket_setOrientationMatrix(XsDataPacket* thisPtr, const XsMatrix* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API int XsDataPacket_containsOrientation(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_orientationIdentifier(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_coordinateSystemOrientation(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsSdiData* XsDataPacket_sdiData(const XsDataPacket* thisPtr, XsSdiData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsSdiData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSdiData(XsDataPacket* thisPtr, const XsSdiData* data);
XSTYPES_DLL_API XsDeviceId* XsDataPacket_storedDeviceId(const XsDataPacket* thisPtr, XsDeviceId* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsStoredDeviceId(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setStoredDeviceId(XsDataPacket* thisPtr, const XsDeviceId* data);
XSTYPES_DLL_API uint32_t XsDataPacket_status(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsStatus(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsDetailedStatus(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setStatus(XsDataPacket* thisPtr, uint32_t data);
XSTYPES_DLL_API void XsDataPacket_setStatusByte(XsDataPacket* thisPtr, uint8_t data);
XSTYPES_DLL_API uint8_t XsDataPacket_packetCounter8(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsPacketCounter8(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPacketCounter8(XsDataPacket* thisPtr, uint8_t counter);
XSTYPES_DLL_API uint16_t XsDataPacket_packetCounter(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsPacketCounter(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPacketCounter(XsDataPacket* thisPtr, uint16_t counter);
XSTYPES_DLL_API uint32_t XsDataPacket_sampleTimeFine(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsSampleTimeFine(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSampleTimeFine(XsDataPacket* thisPtr, uint32_t counter);
XSTYPES_DLL_API uint32_t XsDataPacket_sampleTimeCoarse(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsSampleTimeCoarse(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSampleTimeCoarse(XsDataPacket* thisPtr, uint32_t counter);
XSTYPES_DLL_API uint64_t XsDataPacket_sampleTime64(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsSampleTime64(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSampleTime64(XsDataPacket* thisPtr, uint64_t counter);
XSTYPES_DLL_API XsVector* XsDataPacket_freeAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsFreeAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setFreeAcceleration(XsDataPacket* thisPtr, const XsVector* g);
XSTYPES_DLL_API double XsDataPacket_temperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setTemperature(XsDataPacket* thisPtr, double temp);
XSTYPES_DLL_API XsGpsPvtData* XsDataPacket_gpsPvtData(const XsDataPacket* thisPtr, XsGpsPvtData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsGpsPvtData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setGpsPvtData(XsDataPacket* thisPtr, const XsGpsPvtData* data);
XSTYPES_DLL_API int XsDataPacket_containsPressure(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsPressure* XsDataPacket_pressure(const XsDataPacket* thisPtr, XsPressure* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsPressureAge(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPressure(XsDataPacket* thisPtr, const XsPressure* data);
XSTYPES_DLL_API XsAnalogInData* XsDataPacket_analogIn1Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsAnalogIn1Data(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAnalogIn1Data(XsDataPacket* thisPtr, const XsAnalogInData* data);
XSTYPES_DLL_API XsAnalogInData* XsDataPacket_analogIn2Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsAnalogIn2Data(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAnalogIn2Data(XsDataPacket* thisPtr, const XsAnalogInData* data);
XSTYPES_DLL_API XsVector* XsDataPacket_positionLLA(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsPositionLLA(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPositionLLA(XsDataPacket* thisPtr, const XsVector* data);
XSTYPES_DLL_API XsVector* XsDataPacket_latitudeLongitude(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsLatitudeLongitude(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setLatitudeLongitude(XsDataPacket* thisPtr, const XsVector* data);
XSTYPES_DLL_API double XsDataPacket_altitude(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsAltitude(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAltitude(XsDataPacket* thisPtr, double data);
XSTYPES_DLL_API double XsDataPacket_altitudeMsl(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsAltitudeMsl(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAltitudeMsl(XsDataPacket* thisPtr, double data);
XSTYPES_DLL_API XsVector* XsDataPacket_velocity(const XsDataPacket* thisPtr, XsVector* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API int XsDataPacket_containsVelocity(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setVelocity(XsDataPacket* thisPtr, const XsVector* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_velocityIdentifier(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_coordinateSystemVelocity(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsUtcTime* XsDataPacket_utcTime(const XsDataPacket* thisPtr, XsUtcTime* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsUtcTime(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setUtcTime(XsDataPacket* thisPtr, const XsUtcTime* data);
XSTYPES_DLL_API XsRange* XsDataPacket_frameRange(const XsDataPacket* thisPtr, XsRange* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsFrameRange(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setFrameRange(XsDataPacket* thisPtr, const XsRange* r);
XSTYPES_DLL_API int XsDataPacket_rssi(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsRssi(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRssi(XsDataPacket* thisPtr, int r);
XSTYPES_DLL_API XsRawGpsDop* XsDataPacket_rawGpsDop(const XsDataPacket* thisPtr, XsRawGpsDop* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsDop(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGpsDop(XsDataPacket* thisPtr, const XsRawGpsDop* data);
XSTYPES_DLL_API XsRawGpsSol* XsDataPacket_rawGpsSol(const XsDataPacket* thisPtr, XsRawGpsSol* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsSol(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGpsSol(XsDataPacket* thisPtr, const XsRawGpsSol* data);
XSTYPES_DLL_API XsRawGpsTimeUtc* XsDataPacket_rawGpsTimeUtc(const XsDataPacket* thisPtr, XsRawGpsTimeUtc* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsTimeUtc(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGpsTimeUtc(XsDataPacket* thisPtr, const XsRawGpsTimeUtc* data);
XSTYPES_DLL_API XsRawGpsSvInfo* XsDataPacket_rawGpsSvInfo(const XsDataPacket* thisPtr, XsRawGpsSvInfo* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsSvInfo(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGpsSvInfo(XsDataPacket* thisPtr, const XsRawGpsSvInfo* data);
XSTYPES_DLL_API XsRawGnssPvtData* XsDataPacket_rawGnssPvtData(const XsDataPacket* thisPtr, XsRawGnssPvtData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGnssPvtData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGnssPvtData(XsDataPacket* thisPtr, const XsRawGnssPvtData* r);
XSTYPES_DLL_API XsRawGnssSatInfo* XsDataPacket_rawGnssSatInfo(const XsDataPacket* thisPtr, XsRawGnssSatInfo* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGnssSatInfo(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGnssSatInfo(XsDataPacket* thisPtr, const XsRawGnssSatInfo* r);
XSTYPES_DLL_API XsDataPacket* XsDataPacket_merge(XsDataPacket* thisPtr, const XsDataPacket* other, int overwrite);
XSTYPES_DLL_API void XsDataPacket_setTriggerIndication(XsDataPacket* thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData const * triggerIndicationData);
XSTYPES_DLL_API XsTriggerIndicationData* XsDataPacket_triggerIndication(XsDataPacket const * thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsTriggerIndication(XsDataPacket const * thisPtr, XsDataIdentifier triggerId);
XSTYPES_DLL_API void XsDataPacket_toMessage(XsDataPacket const * thisPtr, XsMessage* msg);

XSTYPES_DLL_API void XsDataPacket_setAwindaSnapshot(XsDataPacket* thisPtr, XsSnapshot const * data, int retransmission);
XSTYPES_DLL_API XsSnapshot* XsDataPacket_awindaSnapshot(XsDataPacket const * thisPtr, XsSnapshot* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsAwindaSnapshot(XsDataPacket const * thisPtr);
XSTYPES_DLL_API int XsDataPacket_isAwindaSnapshotARetransmission(XsDataPacket const * thisPtr);

XSTYPES_DLL_API void XsDataPacket_setFullSnapshot(XsDataPacket* thisPtr, XsSnapshot const * data, int retransmission);
XSTYPES_DLL_API XsSnapshot* XsDataPacket_fullSnapshot(XsDataPacket const * thisPtr, XsSnapshot* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsFullSnapshot(XsDataPacket const * thisPtr);

XSTYPES_DLL_API void XsDataPacket_setRawBlob(XsDataPacket* thisPtr, XsByteArray const* data);
XSTYPES_DLL_API XsByteArray* XsDataPacket_rawBlob(XsDataPacket const * thisPtr, XsByteArray* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawBlob(XsDataPacket const * thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsDataPacket {
#ifdef __cplusplus
	/*! \brief Default constructor, initializes empty data packet or from the supplied \a msg
		\param msg Either 0 to create an empty object or a pointer to a valid %XsMessage containing
		MTData2 data.
	*/
	inline explicit XsDataPacket(const XsMessage* msg = 0)
	{
		XsDataPacket_construct(this);
		if (msg)
			XsDataPacket_setMessage(this, msg);
	}

	/*! \brief Copy constructor
		\param pack The packet to copy from
	*/
	inline XsDataPacket(const XsDataPacket& pack)
	{
		XsDataPacket_construct(this);
		*this = pack;
	}

	//! \copydoc XsDataPacket_destruct
	inline ~XsDataPacket()
	{
		XsDataPacket_destruct(this);
	}

	/*! \brief Assignment operator
		\param other The packet to copy from
		\returns A reference to this %XsDataPacket
		\sa XsDataPacket_copy
	*/
	inline const XsDataPacket& operator = (const XsDataPacket& other)
	{
		if (this != &other)
			XsDataPacket_copy(this, &other);
		return *this;
	}

	//! \copydoc XsDataPacket_swap
	inline void swap(XsDataPacket& other)
	{
		XsDataPacket_swap(this, &other);
	}

	//! \copydoc XsDataPacket_clear
	inline void clear(XsDataIdentifier id = XDI_None)
	{
		XsDataPacket_clear(this, id);
	}

	/*! \brief \copybrief XsDataPacket_empty */
	inline bool empty(void) const
	{
		return 0 != XsDataPacket_empty(this);
	}

	//! \brief Return the device ID associated with the data packet
	inline XsDeviceId deviceId() const
	{
		return m_deviceId;
	}

	//! \brief Return the number of data items in the packet
	inline uint16_t itemCount() const
	{
		return static_cast<uint16_t>(static_cast<unsigned int>(XsDataPacket_itemCount(this)));
	}

	//! \copydoc XsDataPacket_setMessage
	inline void setMessage(const XsMessage& msg)
	{
		XsDataPacket_setMessage(this, &msg);
	}

	/*! \brief Returns a const reference to the message that contains the data packet
	*/
	inline XsMessage toMessage() const
	{
		XsMessage msg;
		XsDataPacket_toMessage(this, &msg);
		return msg;
	}

	//! \brief Set the device ID associated with this data packet
	inline void setDeviceId(const XsDeviceId id)
	{
		m_deviceId = id;
	}

	/*! \copydoc XsDataPacket_dataFormat */
	inline XsDataIdentifier dataFormat(XsDataIdentifier id) const
	{
		return XsDataPacket_dataFormat(this, id);
	}

	/*! \brief \copybrief XsDataPacket_rawAcceleration */
	inline XsUShortVector rawAcceleration(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawAcceleration(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawAccelerationConverted */
	inline XsVector rawAccelerationConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawAccelerationConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawAcceleration */
	inline bool containsRawAcceleration(void) const
	{
		return 0 != XsDataPacket_containsRawAcceleration(this);
	}

	/*! \copydoc XsDataPacket_setRawAcceleration */
	inline void setRawAcceleration(const XsUShortVector& vec)
	{
		XsDataPacket_setRawAcceleration(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeData */
	inline XsUShortVector rawGyroscopeData(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawGyroscopeData(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeDataConverted */
	inline XsVector rawGyroscopeDataConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawGyroscopeDataConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawGyroscopeData */
	inline bool containsRawGyroscopeData(void) const
	{
		return 0 != XsDataPacket_containsRawGyroscopeData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGyroscopeData */
	inline void setRawGyroscopeData(const XsUShortVector& vec)
	{
		XsDataPacket_setRawGyroscopeData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeTemperatureData */
	inline XsUShortVector rawGyroscopeTemperatureData(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawGyroscopeTemperatureData(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeTemperatureDataConverted */
	inline XsVector rawGyroscopeTemperatureDataConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawGyroscopeTemperatureDataConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawGyroscopeTemperatureData */
	inline bool containsRawGyroscopeTemperatureData(void) const
	{
		return 0 != XsDataPacket_containsRawGyroscopeTemperatureData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGyroscopeTemperatureData */
	inline void setRawGyroscopeTemperatureData(const XsUShortVector& vec)
	{
		XsDataPacket_setRawGyroscopeTemperatureData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawMagneticField */
	inline XsUShortVector rawMagneticField(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawMagneticField(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawMagneticFieldConverted */
	inline XsVector rawMagneticFieldConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawMagneticFieldConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawMagneticField */
	inline bool containsRawMagneticField(void) const
	{
		return 0 != XsDataPacket_containsRawMagneticField(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawMagneticField */
	inline void setRawMagneticField(const XsUShortVector& vec)
	{
		XsDataPacket_setRawMagneticField(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawTemperature */
	inline uint16_t rawTemperature(void) const
	{
		return XsDataPacket_rawTemperature(this);
	}

	/*! \copydoc XsDataPacket_containsRawTemperature */
	inline bool containsRawTemperature(void) const
	{
		return 0 != XsDataPacket_containsRawTemperature(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawTemperature */
	inline void setRawTemperature(uint16_t temp)
	{
		XsDataPacket_setRawTemperature(this, temp);
	}

	/*! \brief \copybrief XsDataPacket_rawData */
	inline XsScrData rawData(void) const
	{
		XsScrData returnVal;
		return *XsDataPacket_rawData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawData */
	inline bool containsRawData(void) const
	{
		return 0 != XsDataPacket_containsRawData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawData */
	inline void setRawData(const XsScrData& data)
	{
		XsDataPacket_setRawData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_calibratedAcceleration */
	inline XsVector calibratedAcceleration(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedAcceleration(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedAcceleration */
	inline bool containsCalibratedAcceleration(void) const
	{
		return 0 != XsDataPacket_containsCalibratedAcceleration(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedAcceleration */
	inline void setCalibratedAcceleration(const XsVector& vec)
	{
		XsDataPacket_setCalibratedAcceleration(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedGyroscopeData */
	inline XsVector calibratedGyroscopeData(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedGyroscopeData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedGyroscopeData */
	inline bool containsCalibratedGyroscopeData(void) const
	{
		return 0 != XsDataPacket_containsCalibratedGyroscopeData(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedGyroscopeData */
	inline void setCalibratedGyroscopeData(const XsVector& vec)
	{
		XsDataPacket_setCalibratedGyroscopeData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedMagneticField */
	inline XsVector calibratedMagneticField(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedMagneticField(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedMagneticField */
	inline bool containsCalibratedMagneticField(void) const
	{
		return 0 != XsDataPacket_containsCalibratedMagneticField(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedMagneticField */
	inline void setCalibratedMagneticField(const XsVector& vec)
	{
		XsDataPacket_setCalibratedMagneticField(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedData */
	inline XsCalibratedData calibratedData(void) const
	{
		XsCalibratedData returnVal;
		return *XsDataPacket_calibratedData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedData */
	inline bool containsCalibratedData(void) const
	{
		return 0 != XsDataPacket_containsCalibratedData(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedData */
	inline void setCalibratedData(const XsCalibratedData& data)
	{
		XsDataPacket_setCalibratedData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_orientationQuaternion */
	inline XsQuaternion orientationQuaternion(XsDataIdentifier coordinateSystem) const
	{
		XsQuaternion returnVal;
		return *XsDataPacket_orientationQuaternion(this, &returnVal, coordinateSystem);
	}

	/*! \brief returns the orientation as a quaternion with the current coordinate system*/
	inline XsQuaternion orientationQuaternion() const
	{
		XsQuaternion returnVal;
		return *XsDataPacket_orientationQuaternion(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \brief \copybrief XsDataPacket_setOrientationQuaternion */
	inline void setOrientationQuaternion(const XsQuaternion& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationQuaternion(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_orientationEuler */
	inline XsEuler orientationEuler(XsDataIdentifier coordinateSystem) const
	{
		XsEuler returnVal;
		return *XsDataPacket_orientationEuler(this, &returnVal, coordinateSystem);
	}

	/*! \brief returns the orientation as an XsEuler with the current coordinate system*/
	inline XsEuler orientationEuler() const
	{
		XsEuler returnVal;
		return *XsDataPacket_orientationEuler(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \brief \copybrief XsDataPacket_setOrientationEuler */
	inline void setOrientationEuler(const XsEuler& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationEuler(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_orientationMatrix */
	inline XsMatrix orientationMatrix(XsDataIdentifier coordinateSystem) const
	{
		XsMatrix returnVal;
		return *XsDataPacket_orientationMatrix(this, &returnVal, coordinateSystem);
	}

	/*! \brief returns the orientation as an orientation matrix with the current coordinate system*/
	inline XsMatrix orientationMatrix() const
	{
		XsMatrix returnVal;
		return *XsDataPacket_orientationMatrix(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \brief \copybrief XsDataPacket_setOrientationMatrix */
	inline void setOrientationMatrix(const XsMatrix& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationMatrix(this, &data, coordinateSystem);
	}

	/*! \copydoc XsDataPacket_containsOrientation */
	inline bool containsOrientation(void) const
	{
		return 0 != XsDataPacket_containsOrientation(this);
	}

	/*! \copydoc XsDataPacket_orientationIdentifier */
	inline XsDataIdentifier orientationIdentifier() const
	{
		return XsDataPacket_orientationIdentifier(this);
	}

	/*! \copydoc XsDataPacket_coordinateSystemOrientation */
	inline XsDataIdentifier coordinateSystemOrientation() const
	{
		return XsDataPacket_coordinateSystemOrientation(this);
	}

	/*! \brief \copybrief XsDataPacket_sdiData */
	inline XsSdiData sdiData(void) const
	{
		XsSdiData returnVal;
		return *XsDataPacket_sdiData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsSdiData */
	inline bool containsSdiData(void) const
	{
		return 0 != XsDataPacket_containsSdiData(this);
	}

	/*! \copydoc XsDataPacket_setSdiData */
	inline void setSdiData(const XsSdiData& data)
	{
		XsDataPacket_setSdiData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_storedDeviceId
	  \returns the device ID stored in this packet
	*/
	inline XsDeviceId storedDeviceId(void) const
	{
		XsDeviceId returnVal;
		return *XsDataPacket_storedDeviceId(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsStoredDeviceId */
	inline bool containsStoredDeviceId(void) const
	{
		return 0 != XsDataPacket_containsStoredDeviceId(this);
	}

	/*! \copydoc XsDataPacket_setStoredDeviceId */
	inline void setStoredDeviceId(const XsDeviceId& data)
	{
		XsDataPacket_setStoredDeviceId(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_status */
	inline uint32_t status(void) const
	{
		return XsDataPacket_status(this);
	}

	/*! \copydoc XsDataPacket_containsStatus */
	inline bool containsStatus(void) const
	{
		return 0 != XsDataPacket_containsStatus(this);
	}

	/*! \copydoc XsDataPacket_containsDetailedStatus */
	inline bool containsDetailedStatus(void) const
	{
		return 0 != XsDataPacket_containsDetailedStatus(this);
	}

	/*! \brief \copybrief XsDataPacket_setStatus */
	inline void setStatus(const uint32_t data)
	{
		XsDataPacket_setStatus(this, data);
	}

	/*! \brief \copybrief XsDataPacket_setStatusByte */
	inline void setStatusByte(const uint8_t data)
	{
		XsDataPacket_setStatusByte(this, data);
	}

	/*! \brief \copybrief XsDataPacket_packetCounter8 */
	inline uint8_t packetCounter8(void) const
	{
		return XsDataPacket_packetCounter8(this);
	}

	/*! \copydoc XsDataPacket_containsPacketCounter8 */
	inline bool containsPacketCounter8(void) const
	{
		return 0 != XsDataPacket_containsPacketCounter8(this);
	}

	/*! \brief \copybrief XsDataPacket_setPacketCounter8 */
	inline void setPacketCounter8(uint8_t counter)
	{
		XsDataPacket_setPacketCounter8(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_packetCounter */
	inline uint16_t packetCounter(void) const
	{
		return XsDataPacket_packetCounter(this);
	}

	/*! \copydoc XsDataPacket_containsPacketCounter */
	inline bool containsPacketCounter(void) const
	{
		return 0 != XsDataPacket_containsPacketCounter(this);
	}

	/*! \brief \copybrief XsDataPacket_setPacketCounter */
	inline void setPacketCounter(uint16_t counter)
	{
		XsDataPacket_setPacketCounter(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTimeFine */
	inline uint32_t sampleTimeFine(void) const
	{
		return XsDataPacket_sampleTimeFine(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTimeFine */
	inline bool containsSampleTimeFine(void) const
	{
		return 0 != XsDataPacket_containsSampleTimeFine(this);
	}

	/*! \brief \copybrief XsDataPacket_setSampleTimeFine */
	inline void setSampleTimeFine(uint32_t counter)
	{
		XsDataPacket_setSampleTimeFine(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTimeCoarse */
	inline uint32_t sampleTimeCoarse(void) const
	{
		return XsDataPacket_sampleTimeCoarse(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTimeCoarse */
	inline bool containsSampleTimeCoarse(void) const
	{
		return 0 != XsDataPacket_containsSampleTimeCoarse(this);
	}

	/*! \brief \copybrief XsDataPacket_setSampleTimeCoarse */
	inline void setSampleTimeCoarse(uint32_t counter)
	{
		XsDataPacket_setSampleTimeCoarse(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTime64 */
	inline uint64_t sampleTime64(void) const
	{
		return XsDataPacket_sampleTime64(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTime64 */
	inline bool containsSampleTime64(void) const
	{
		return 0 != XsDataPacket_containsSampleTime64(this);
	}

	/*! \brief \copybrief XsDataPacket_setSampleTime64 */
	inline void setSampleTime64(uint64_t counter)
	{
		XsDataPacket_setSampleTime64(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_freeAcceleration */
	inline XsVector freeAcceleration(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_freeAcceleration(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsFreeAcceleration */
	inline bool containsFreeAcceleration(void) const
	{
		return 0 != XsDataPacket_containsFreeAcceleration(this);
	}

	/*! \brief \copybrief XsDataPacket_setFreeAcceleration */
	inline void setFreeAcceleration(const XsVector& g)
	{
		XsDataPacket_setFreeAcceleration(this, &g);
	}

	/*! \brief \copybrief XsDataPacket_temperature */
	inline double temperature(void) const
	{
		return XsDataPacket_temperature(this);
	}

	/*! \copydoc XsDataPacket_containsTemperature */
	inline bool containsTemperature(void) const
	{
		return 0 != XsDataPacket_containsTemperature(this);
	}

	/*! \brief \copybrief XsDataPacket_setTemperature */
	inline void setTemperature(double temp)
	{
		XsDataPacket_setTemperature(this, temp);
	}

	/*! \brief \copybrief XsDataPacket_gpsPvtData */
	inline XsGpsPvtData gpsPvtData(void) const
	{
		XsGpsPvtData returnVal;
		return *XsDataPacket_gpsPvtData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsGpsPvtData */
	inline bool containsGpsPvtData(void) const
	{
		return 0 != XsDataPacket_containsGpsPvtData(this);
	}

	/*! \copydoc XsDataPacket_setGpsPvtData */
	inline void setGpsPvtData(const XsGpsPvtData& data)
	{
		XsDataPacket_setGpsPvtData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_pressure */
	inline XsPressure pressure(void) const
	{
		XsPressure returnVal;
		return *XsDataPacket_pressure(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsPressure */
	inline bool containsPressure(void) const
	{
		return 0 != XsDataPacket_containsPressure(this);
	}

	/*! \copydoc XsDataPacket_containsPressureAge */
	inline bool containsPressureAge(void) const
	{
		return 0 != XsDataPacket_containsPressureAge(this);
	}

	/*! \brief \copybrief XsDataPacket_setPressure */
	inline void setPressure(const XsPressure& data)
	{
		XsDataPacket_setPressure(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_analogIn1Data */
	inline XsAnalogInData analogIn1Data(void) const
	{
		XsAnalogInData returnVal;
		return *XsDataPacket_analogIn1Data(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsAnalogIn1Data */
	inline bool containsAnalogIn1Data(void) const
	{
		return 0 != XsDataPacket_containsAnalogIn1Data(this);
	}

	/*! \brief \copybrief XsDataPacket_setAnalogIn1Data */
	inline void setAnalogIn1Data(const XsAnalogInData& data)
	{
		XsDataPacket_setAnalogIn1Data(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_analogIn2Data */
	inline XsAnalogInData analogIn2Data(void) const
	{
		XsAnalogInData returnVal;
		return *XsDataPacket_analogIn2Data(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsAnalogIn2Data */
	inline bool containsAnalogIn2Data(void) const
	{
		return 0 != XsDataPacket_containsAnalogIn2Data(this);
	}

	/*! \brief \copybrief XsDataPacket_setAnalogIn2Data */
	inline void setAnalogIn2Data(const XsAnalogInData& data)
	{
		XsDataPacket_setAnalogIn2Data(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_positionLLA */
	inline XsVector positionLLA(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_positionLLA(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsPositionLLA */
	inline bool containsPositionLLA(void) const
	{
		return 0 != XsDataPacket_containsPositionLLA(this);
	}

	/*! \copydoc XsDataPacket_setPositionLLA */
	inline void setPositionLLA(const XsVector& data)
	{
		XsDataPacket_setPositionLLA(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_latitudeLongitude */
	inline XsVector latitudeLongitude(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_latitudeLongitude(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsLatitudeLongitude */
	inline bool containsLatitudeLongitude(void) const
	{
		return 0 != XsDataPacket_containsLatitudeLongitude(this);
	}

	/*! \copydoc XsDataPacket_setLatitudeLongitude */
	inline void setLatitudeLongitude(const XsVector& data)
	{
		XsDataPacket_setLatitudeLongitude(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_altitude */
	inline double altitude(void) const
	{
		return XsDataPacket_altitude(this);
	}

	/*! \copydoc XsDataPacket_containsAltitude */
	inline bool containsAltitude(void) const
	{
		return 0 != XsDataPacket_containsAltitude(this);
	}

	/*! \copydoc XsDataPacket_setAltitude */
	inline void setAltitude(double data)
	{
		XsDataPacket_setAltitude(this, data);
	}

	/*! \brief \copybrief XsDataPacket_altitudeMsl */
	inline double altitudeMsl(void) const
	{
		return XsDataPacket_altitudeMsl(this);
	}

	/*! \copydoc XsDataPacket_containsAltitudeMsl */
	inline bool containsAltitudeMsl(void) const
	{
		return 0 != XsDataPacket_containsAltitudeMsl(this);
	}

	/*! \copydoc XsDataPacket_setAltitudeMsl */
	inline void setAltitudeMsl(double data)
	{
		XsDataPacket_setAltitudeMsl(this, data);
	}

	/*! \brief \copybrief XsDataPacket_velocity */
	inline XsVector velocity(XsDataIdentifier coordinateSystem) const
	{
		XsVector returnVal;
		return *XsDataPacket_velocity(this, &returnVal, coordinateSystem);
	}

	/*! \brief returns the velocity with the current coordinate system*/
	inline XsVector velocity(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_velocity(this, &returnVal, coordinateSystemVelocity());
	}

	/*! \copydoc XsDataPacket_containsVelocity */
	inline bool containsVelocity(void) const
	{
		return 0 != XsDataPacket_containsVelocity(this);
	}

	/*! \brief \copybrief XsDataPacket_setVelocity */
	inline void setVelocity(const XsVector& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setVelocity(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_velocityIdentifier */
	inline XsDataIdentifier velocityIdentifier() const
	{
		return XsDataPacket_velocityIdentifier(this);
	}

	/*! \copydoc XsDataPacket_coordinateSystemVelocity */
	inline XsDataIdentifier coordinateSystemVelocity() const
	{
		return XsDataPacket_coordinateSystemVelocity(this);
	}

	/*! \brief \copybrief XsDataPacket_utcTime */
	inline XsUtcTime utcTime(void) const
	{
		XsUtcTime returnVal;
		return *XsDataPacket_utcTime(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsUtcTime */
	inline bool containsUtcTime(void) const
	{
		return 0 != XsDataPacket_containsUtcTime(this);
	}

	/*! \brief \copybrief XsDataPacket_setUtcTime */
	inline void setUtcTime(const XsUtcTime& data)
	{
		XsDataPacket_setUtcTime(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_frameRange */
	inline XsRange frameRange() const
	{
		XsRange returnVal;
		return *XsDataPacket_frameRange(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsFrameRange */
	inline bool containsFrameRange() const
	{
		return 0 != XsDataPacket_containsFrameRange(this);
	}

	/*! \copydoc XsDataPacket_setFrameRange */
	inline void setFrameRange(const XsRange& r)
	{
		XsDataPacket_setFrameRange(this, &r);
	}

	/*! \brief \copybrief XsDataPacket_rssi */
	inline int rssi() const
	{
		return XsDataPacket_rssi(this);
	}

	/*! \copydoc XsDataPacket_containsRssi */
	inline bool containsRssi() const
	{
		return 0 != XsDataPacket_containsRssi(this);
	}

	/*! \copydoc XsDataPacket_setRssi */
	inline void setRssi(int r)
	{
		XsDataPacket_setRssi(this, r);
	}


	/*! \brief \copybrief XsDataPacket_rawGpsDop */
	inline XsRawGpsDop rawGpsDop(void) const
	{
		XsRawGpsDop returnVal;
		return *XsDataPacket_rawGpsDop(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsDop */
	inline bool containsRawGpsDop(void) const
	{
		return 0 != XsDataPacket_containsRawGpsDop(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGpsDop */
	XSNOEXPORT inline void setRawGpsDop(XsRawGpsDop const& raw)
	{
		XsDataPacket_setRawGpsDop(this, &raw);
	}

	/*! \brief \copybrief XsDataPacket_rawGpsSol */
	inline XsRawGpsSol rawGpsSol(void) const
	{
		XsRawGpsSol returnVal;
		return *XsDataPacket_rawGpsSol(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsSol */
	inline bool containsRawGpsSol(void) const
	{
		return 0 != XsDataPacket_containsRawGpsSol(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGpsSol */
	XSNOEXPORT inline void setRawGpsSol(XsRawGpsSol const& raw)
	{
		XsDataPacket_setRawGpsSol(this, &raw);
	}

	/*! \brief \copybrief XsDataPacket_rawGpsTimeUtc */
	inline XsRawGpsTimeUtc rawGpsTimeUtc(void) const
	{
		XsRawGpsTimeUtc returnVal;
		return *XsDataPacket_rawGpsTimeUtc(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsTimeUtc */
	inline bool containsRawGpsTimeUtc(void) const
	{
		return 0 != XsDataPacket_containsRawGpsTimeUtc(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGpsTimeUtc */
	XSNOEXPORT inline void setRawGpsTimeUtc(XsRawGpsTimeUtc const& raw)
	{
		XsDataPacket_setRawGpsTimeUtc(this, &raw);
	}

	/*! \brief \copybrief XsDataPacket_rawGpsSvInfo */
	inline XsRawGpsSvInfo rawGpsSvInfo(void) const
	{
		XsRawGpsSvInfo returnVal;
		return *XsDataPacket_rawGpsSvInfo(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsSvInfo */
	inline bool containsRawGpsSvInfo(void) const
	{
		return 0 != XsDataPacket_containsRawGpsSvInfo(this);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsSvInfo */
	XSNOEXPORT inline void setRawGpsSvInfo(XsRawGpsSvInfo const& raw)
	{
		XsDataPacket_setRawGpsSvInfo(this, &raw);
	}

	/*! \brief \copybrief XsDataPacket_rawGnssPvtData */
	inline XsRawGnssPvtData rawGnssPvtData(void) const
	{
		XsRawGnssPvtData returnVal;
		return *XsDataPacket_rawGnssPvtData(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGnnsPvtData */
	inline bool containsRawGnssPvtData(void) const
	{
		return 0 != XsDataPacket_containsRawGnssPvtData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGnnsPvtData */
	XSNOEXPORT inline void setRawGnssPvtData(XsRawGnssPvtData const& raw)
	{
		XsDataPacket_setRawGnssPvtData(this, &raw);
	}

	/*! \brief \copybrief XsDataPacket_rawGnssSatInfo */
	inline XsRawGnssSatInfo rawGnssSatInfo(void) const
	{
		XsRawGnssSatInfo returnVal;
		return *XsDataPacket_rawGnssSatInfo(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGnssSatInfo */
	inline bool containsRawGnssSatInfo(void) const
	{
		return 0 != XsDataPacket_containsRawGnssSatInfo(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGnnsSatInfo */
	XSNOEXPORT inline void setRawGnssSatInfo(XsRawGnssSatInfo const& raw)
	{
		XsDataPacket_setRawGnssSatInfo(this, &raw);
	}
	/*! \brief \copybrief XsDataPacket_fullSnapshot */
	inline XsSnapshot fullSnapshot(void) const
	{
		XsSnapshot returnVal;
		return *XsDataPacket_fullSnapshot(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsFullSnapshot */
	inline bool containsFullSnapshot(void) const
	{
		return 0 != XsDataPacket_containsFullSnapshot(this);
	}

	/*! \brief \copybrief XsDataPacket_setFullSnapshot */
	XSNOEXPORT inline void setFullSnapshot(XsSnapshot const& raw, bool retransmission)
	{
		XsDataPacket_setFullSnapshot(this, &raw, retransmission?1:0);
	}
	/*! \brief \copybrief XsDataPacket_awindaSnapshot */
	XSNOEXPORT inline XsSnapshot awindaSnapshot(void) const
	{
		XsSnapshot returnVal;
		return *XsDataPacket_awindaSnapshot(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsAwindaSnapshot */
	XSNOEXPORT inline bool containsAwindaSnapshot(void) const
	{
		return 0 != XsDataPacket_containsAwindaSnapshot(this);
	}

	/*! \brief \copybrief XsDataPacket_setAwindaSnapshot */
	XSNOEXPORT inline void setAwindaSnapshot(XsSnapshot const& raw, bool retransmission)
	{
		XsDataPacket_setAwindaSnapshot(this, &raw, retransmission?1:0);
	}

	/*! \brief \copybrief XsDataPacket_containsAwindaSnapshot */
	inline bool isAwindaSnapshotARetransmission(void) const
	{
		return 0 != XsDataPacket_isAwindaSnapshotARetransmission(this);
	}

	/*! \copydoc XsDataPacket_merge */
	inline XsDataPacket& merge(const XsDataPacket& other, bool overwrite = true)
	{
		return *XsDataPacket_merge(this, &other, overwrite?1:0);
	}

	/*! \private \brief Set the time of arrival of the data packet */
	inline void setTimeOfArrival(XsTimeStamp t)
	{
		m_toa = t;
	}

	/*! \brief Return the time of arrival of the data packet. Only valid for live streams. The behaviour for file streams is undefined and may change in the future. */
	inline XsTimeStamp timeOfArrival() const
	{
		return m_toa;
	}

	/*! \private \brief Set the packet ID of the data packet*/
	inline void setPacketId(int64_t t)
	{
		m_packetId = t;
	}

	/*! \brief Return the ID of the packet.
		\details This ID is based on, depending on availability: (1) packet counter (2) sample time (3) arrival order
		\returns The ID of the packet.
	*/
	inline int64_t packetId() const
	{
		return m_packetId;
	}

	/*! \copydoc XsDataPacket_setTriggerIndication */
	void setTriggerIndication(XsDataIdentifier triggerId, XsTriggerIndicationData const & triggerIndicationData)
	{
		XsDataPacket_setTriggerIndication(this, triggerId, &triggerIndicationData);
	}

	/*! \copydoc XsDataPacket_containsTriggerIndication */
	inline bool containsTriggerIndication(XsDataIdentifier triggerId) const
	{
		return 0 != XsDataPacket_containsTriggerIndication(this, triggerId);
	}

	/*! \brief Returns the trigger indication data of a packet
		\details
		If the packet does not contain the requested data, the return val struct will be set to all zeroes
		\param[in] triggerId The trigger data identifier to add data for (e.g. XDI_TriggerIn1 or XDI_TriggerIn2)
		\returns the trigger indication data of a packet
	*/
	XsTriggerIndicationData triggerIndication(XsDataIdentifier triggerId)
	{
		XsTriggerIndicationData returnVal;
		return *XsDataPacket_triggerIndication(this, triggerId, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawBlob */
	inline XsByteArray rawBlob(void) const
	{
		XsByteArray returnVal;
		return *XsDataPacket_rawBlob(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawBlob */
	inline bool containsRawBlob(void) const
	{
		return 0 != XsDataPacket_containsRawBlob(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawBlob */
	inline void setRawBlob(XsByteArray const& data)
	{
		XsDataPacket_setRawBlob(this, &data);
	}

//protected:
	/*! \privatesection */
#endif // __cplusplus
	struct XSNOEXPORT DataPacketPrivate* d;

	XsDeviceId			m_deviceId;					//!< The device Id to which the message belongs
	XsTimeStamp			m_toa;						//!< Time of arrival (live packets only)
	int64_t				m_packetId;					//!< 64 bit packet id, based on, depending on availability: (1) packet counter (2) sample time (3) arrival order
};

#endif // file guard
