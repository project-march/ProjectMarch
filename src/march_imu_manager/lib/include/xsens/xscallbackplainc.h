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

#ifndef XSCALLBACKPLAINC_H
#define XSCALLBACKPLAINC_H

#include <xsens/pstdint.h>
#include <xsens/xsresultvalue.h>
#include <xsens/xsinforequest.h>
#include "xsdevicestate.h"
#include "xsconnectivitystate.h"

#ifndef __cplusplus
#define XSCALLBACK_INITIALIZER		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#endif

struct XsDevice;
struct XsDevicePtrArray;
struct XsDataPacket;
struct XsString;
struct XsMessage;

/*! \brief Structure that contains callback functions for the Xsens Device API
	\details When using C++, please use the overloaded class XsCallback instead.

	This structure contains pointers to functions that will be called by XDA when certain
	events occur. To use it in C, set any callback you do not wish to use to 0 and put a valid
	function pointer in the others. Then pass the object to an XsControl or XsDevice object's
	addCallbackHandler function.

	\note XDA does not copy the structure contents and does not take ownership of it. So make sure it
	is allocated on the heap or at least removed from wherever it was added by calling
	removeCallbackHandler before it is destroyed.
*/
typedef struct XsCallbackPlainC
{
/*! \defgroup Callbacks Callback functions.
	\addtogroup Callbacks
	@{
*/
	/*! \brief Called when a device's state has changed (ie config mode, measurement mode, recording mode)
		\param dev The device that initiated the callback. This may be 0 in some cases.
		\param newState The new device state
		\param oldState The old device state
	*/
	void (*m_onDeviceStateChanged)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsDeviceState newState, XsDeviceState oldState);

	/*! \brief Called when new data has been received from a device or read from a file. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\param dev The device that initiated the callback.
		\param packet The data packet that has been received (and processed). This may be 0 when the callback originates from a non-device, such as the XsDataBundler.
	*/
	void (*m_onLiveDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsDataPacket* packet);

	/*! \brief Called when XDA detects that packets have been missed.
		\param dev The device that initiated the callback.
		\param count The number of samples that were missed
		\param first The sample counter / packet identifier of the first missed sample
		\param last The sample counter / packet identifier of the last missed sample
	*/
	void (*m_onMissedPackets)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, int count, int first, int last);

	/*! \brief Called when a wakeup message has been received from a device. This indicates that the device has just been reset or plugged in.
		\param dev The device that initiated the callback.
	*/
	void (*m_onWakeupReceived)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev);

	/*! \brief Called when a long-duration operation has made some progress or has completed. Examples include loadLogFile and flushing of retransmissions (Awinda). When \a current == \a total the operation has completed.
		\param dev The device that initiated the callback.
		\param current The current progress.
		\param total The total work to be done. When \a current equals \a total, the task is completed.
		\param identifier An identifier for the task. This may for example be a filename for file read operations.
	*/
	void (*m_onProgressUpdated)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, int current, int total, const struct XsString* identifier);

	/*! \brief Called when XDA has a message that could be written to a log file. \returns 0 to prevent the message from being written, non-0 to allow the write. This includes data packets. \param message The message that is ready to be written to file \sa m_onWriteDataToLogFile
		\param dev The device that initiated the callback.
		\param message The message that will be written.
		\returns true if the write to file should be allowed. Note that if ANY callback decides that the write is not allowed, it will be disallowed.
	*/
	int  (*m_onWriteMessageToLogFile)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsMessage* message);

	/*! \brief Called when XDA has a data packet that could be written to a log file.
		\param dev The device that initiated the callback.
		\param data The data message that is ready to be written to file \sa onWriteMessageToLogFile
	*/
	void (*m_onRecordingDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsDataPacket* data);

	/*! \brief Called when XDA has detected a change in the connectivity state of a device
		\param dev The device that initiated the callback.
		\param newState The new connectivity state
	*/
	void (*m_onConnectivityChanged)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsConnectivityState newState);

	/*! \brief Called when an information request has resulted in a response
		\details When the information request has completed, the data can be retrieved through the usual
		functions. Ie. when a requestBatteryLevel() resulted in an onInfoResponse(.., XIR_BatteryLevel),
		the XsDevice::batteryLevel function will return the received battery level.
		\param dev The device that initiated the callback.
		\param request The type of request that was completed
	*/
	void (*m_onInfoResponse)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsInfoRequest request);


	/*! \brief Called when an error has occurred while handling incoming data
		\param dev The device that generated the error message
		\param error The error code that specifies exactly what problem occurred
	*/
	void (*m_onError)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsResultValue error);

	/*! \brief Called when a non data, non reply message has been received
		\param dev The device that generated the error message
		\param message The message that has been received
	*/
	void (*m_onNonDataMessage)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, struct XsMessage const * message);

	/*! \brief Called just after a valid message (after parsing) is received from the device.
		\param dev The device that sent the message
		\param message The message that has been received
	*/
	void (*m_onMessageReceivedFromDevice)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, struct XsMessage const * message);

	/*! \brief Called just after a message is sent to the device.
		\param dev The device that will receive the message
		\param message The message that will be sent
	*/
	void (*m_onMessageSentToDevice)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, struct XsMessage const * message);

	/*! \brief Called when new data has been received for devices connected to the same main device. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\param devs A managed array of devices for which data is available. The array will be cleaned up by XDA after the callback returns.
		\param packets A plain array of pointers to data packets matching the order of \a devs. The array will be cleaned up by XDA after the callback returns.
	*/
	void (*m_onAllLiveDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevicePtrArray* devs, const struct XsDataPacket** packets);

	/*! \brief Called when new data has been received for devices connected to the same main device. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\param devs A managed array of devices for which data is available. The array will be cleaned up by XDA after the callback returns.
		\param packets A plain array of pointers to data packets matching the order of \a devs. The array will be cleaned up by XDA after the callback returns.
	*/
	void (*m_onAllRecordingDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevicePtrArray* devs, const struct XsDataPacket** packets);

	/*! \brief Called when XDA detects that data is forever unavailable
		\details This differs from onMissedPackets, since missed packets may be retransmitted, while unavailable
		data can no longer be retransmitted.
		\param dev The device that initiated the callback.
		\param packetId The sample counter / packet identifier of the unavailable sample
	*/
	void (*m_onDataUnavailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, int64_t packetId);

//! @}
#ifdef __cplusplus
	// Make sure that this struct is not used in C++ (except as base class for XsCallback)
	friend class XsCallback;
	XsCallbackPlainC() {}
	~XsCallbackPlainC() throw() {}
private:
	XsCallbackPlainC(XsCallbackPlainC const &);
	XsCallbackPlainC& operator = (XsCallbackPlainC const &);
#endif

} XsCallbackPlainC;

#endif
