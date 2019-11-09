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

#ifndef XSSTATUSFLAG_H
#define XSSTATUSFLAG_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Status flags
	\details These flags define the function of specific bits in the status returned by
	XsDataPacket::status()
	\sa XsDataPacket::status()
*/

enum XsStatusFlag {
	 XSF_SelfTestOk			= 0x01		//!< Is set when the self test result was ok
	,XSF_OrientationValid	= 0x02		//!< Is set when the computed orientation is valid. The orientation may be invalid during startup or when the sensor data is clipping during violent (for the device) motion
	,XSF_GpsValid			= 0x04		//!< Is set when the device has a GPS receiver and the receiver says that there is a GPS position fix.

	,XSF_NoRotationMask				= 0x18		//!< If all of these flags are set, the No Rotation algorithm is running
	,XSF_NoRotationAborted			= 0x10		//!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm was aborted
	,XSF_NoRotationSamplesRejected	= 0x08		//!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running but has rejected samples
	,XSF_NoRotationRunningNormally	= 0x18		//!< If all these flags are set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running normally

	,XSF_ClipAccX	= 0x00000100
	,XSF_ClipAccY	= 0x00000200
	,XSF_ClipAccZ	= 0x00000400
	,XSF_ClipGyrX	= 0x00000800
	,XSF_ClipGyrY	= 0x00001000
	,XSF_ClipGyrZ	= 0x00002000
	,XSF_ClipMagX	= 0x00004000
	,XSF_ClipMagY	= 0x00008000
	,XSF_ClipMagZ	= 0x00010000

	,XSF_Retransmitted	= 0x00080000	//!< When set Indicates the sample was received as a retransmission
	,XSF_Interpolated	= 0x00100000	//!< When set Indicates the sample is an interpolation between other samples
	,XSF_SyncIn			= 0x00200000	//!< When set indicates a sync-in event has been triggered
	,XSF_SyncOut		= 0x00400000	//!< When set Indicates a sync-out event has been generated


	,XSF_FilterMode	= 0x03800000		//!< Mask for the 3 bit filter mode field
};

/*! \brief Status flag bit offsets
	\details Sometimes (rarely) it is necessary to know the bit offset instead of the bit mask (ie when
	shifting to only keep a subset of flags) for the status flags. This enumeration provides these
	offsets.
	\sa XsStatusFlag
*/
enum XsStatusFlagOffset {
	 XSFO_OffsetSelfTestOk			= 0
	,XSFO_OffsetOrientationValid	= 1
	,XSFO_OffsetGpsValid			= 2
	,XSFO_OffsetNoRotation			= 3

	,XSFO_OffsetClipAccX			= 8
	,XSFO_OffsetClipAccY			= 9
	,XSFO_OffsetClipAccZ			= 10
	,XSFO_OffsetClipGyrX			= 11
	,XSFO_OffsetClipGyrY			= 12
	,XSFO_OffsetClipGyrZ			= 13
	,XSFO_OffsetClipMagX			= 14
	,XSFO_OffsetClipMagY			= 15
	,XSFO_OffsetClipMagZ			= 16

	,XSFO_Retransmitted				= 19
	,XSFO_Interpolated				= 20
	,XSFO_SyncIn					= 21
	,XSFO_SyncOut					= 22

	,XSFO_FilterMode				= 23	// bits 23 -> 23 + XSFO_FilterModeNrOfBits - 1
	,XSFO_FilterModeNrOfBits		= 3		// note: bit 26 is reserved for future use
};

/*! @} */
typedef enum XsStatusFlag XsStatusFlag;
typedef enum XsStatusFlagOffset XsStatusFlagOffset;

#endif // file guard
