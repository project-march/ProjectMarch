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

#ifndef XSSNAPSHOT_H
#define XSSNAPSHOT_H

#include "xstypesconfig.h"
#include "xsdeviceid.h"

struct XsSnapshot;

#ifdef __cplusplus
extern "C" {
#else
#define XSSNAPSHOT_INITIALIZER { XSDEVICEID_INITIALIZER, 0, 0, 0,0,0,0, 0,0,0, 0,0,0, 0, 0, 0, 0 }
#endif

XSTYPES_DLL_API void XsSnapshot_construct(struct XsSnapshot* thisPtr);
XSTYPES_DLL_API void XsSnapshot_destruct(struct XsSnapshot* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

enum SnapshotType
{
	ST_Awinda = 0, //
	ST_Full
};
typedef enum SnapshotType SnapshotType;
/*! \brief A container for Snapshot data
*/
struct XsSnapshot
{
	XsDeviceId m_deviceId;	/*!< \brief The ID of the device that created the data */
	uint32_t m_frameNumber;		/*!< \brief The frame */
	uint64_t m_timestamp;	/*!< \brief The timestamp */
	int32_t m_iQ[4];		/*!< \brief The integrated orientation */
	int64_t m_iV[3];		/*!< \brief The integrated velocity */
	int32_t m_mag[3];		/*!< \brief The magnetic field */
	int32_t m_baro;			/*!< \brief The barometric pressure */
	uint16_t m_status;		/*!< \brief The clipping flags of the latest interval  */
	uint8_t m_accClippingCounter;	/*!< \brief The clipping event counter for the Acc */
	uint8_t m_gyrClippingCounter;	/*!< \brief The clipping event counter for the Gyr */
	SnapshotType m_type; /*!< \brief The type of the snapshot (Awinda, Full) */
#ifdef __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsSnapshot& other) const
	{
		if (m_frameNumber != other.m_frameNumber ||
			m_baro != other.m_baro ||
			m_status != other.m_status ||
			m_accClippingCounter != other.m_accClippingCounter ||
			m_gyrClippingCounter != other.m_gyrClippingCounter||
			m_type != other.m_type)
			return false;

		for (int i = 0; i < 3; ++i)
		{
			if (m_iQ[i] != other.m_iQ[i] ||
				m_iV[i] != other.m_iV[i] ||
				m_mag[i] != other.m_mag[i])
				return false;
		}
		if (m_type == ST_Full)
		{
			if (m_iQ[3] != other.m_iQ[3] || m_timestamp != other.m_timestamp  )
				return false;
		}
		return true;
	}
#endif
};
typedef struct XsSnapshot XsSnapshot;

/*! \brief Status flag definitions for XsSnapshot status field */
enum SnapshotStatusFlag
{
	FSFL_ClipAccX		= 0x0001,
	FSFL_ClipAccY		= 0x0002,
	FSFL_ClipAccZ		= 0x0004,
	FSFL_ClipAccMask	= 0x0007,
	FSFL_ClipGyrX		= 0x0008,
	FSFL_ClipGyrY		= 0x0010,
	FSFL_ClipGyrZ		= 0x0020,
	FSFL_ClipGyrMask	= 0x0038,
	FSFL_ClipMagX		= 0x0040,
	FSFL_ClipMagY		= 0x0080,
	FSFL_ClipMagZ		= 0x0100,
	FSFL_ClipMagMask	= 0x01C0,
	FSFL_MagIsNew		= 0x0200,
	FSFL_BaroIsNew		= 0x0400,
	FSFL_RotationMask	= 0x1800
};
typedef enum SnapshotStatusFlag SnapshotStatusFlag;



#define FSFL_ClipAccShift			0
#define FSFL_ClipGyrShift			04
#define FSFL_ClipMagshift			6
#define FSFL_RotationShift		11

#endif
