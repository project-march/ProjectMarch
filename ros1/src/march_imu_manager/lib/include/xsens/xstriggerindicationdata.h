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

#ifndef XSTRIGGERINDICATIONDATA_H
#define XSTRIGGERINDICATIONDATA_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C" {
#else
#define XSTRIGGERINDICATIONDATA_INITIALIZER	{ 0, 0, 0, 0 }
#endif

struct XsTriggerIndicationData;

XSTYPES_DLL_API void XsTriggerIndicationData_destruct(struct XsTriggerIndicationData* thisPtr);
XSTYPES_DLL_API int XsTriggerIndicationData_valid(const struct XsTriggerIndicationData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief Data for a trigger indication message */
struct XsTriggerIndicationData {
	uint8_t m_line;			//!< The line number
	uint8_t m_polarity;		//!< The polarity
	uint32_t m_timestamp;	//!< The timestamp
	uint16_t m_frameNumber;	//!< The frame number

#ifdef __cplusplus
	/*! Constructor
		\param[in] line Line
		\param[in] polarity Polarity
		\param[in] timestamp Timestamp
		\param[in] frameNumber Frame number
	*/
	explicit XsTriggerIndicationData(uint8_t line = 0, uint8_t polarity = 0, uint32_t timestamp = 0, uint16_t frameNumber = 0)
	  : m_line(line), m_polarity(polarity), m_timestamp(timestamp), m_frameNumber(frameNumber)
	{}

	/*! \brief \copybrief XsTriggerIndicationData_destruct */
	inline void clear()
	{
		XsTriggerIndicationData_destruct(this);
	}

	/*! \brief \copybrief XsTriggerIndicationData_valid */
	inline bool valid() const
	{
		return 0 != XsTriggerIndicationData_valid(this);
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (XsTriggerIndicationData const& other) const
	{
		return m_line == other.m_line &&
				m_polarity == other.m_polarity &&
				m_timestamp == other.m_timestamp &&
				m_frameNumber == other.m_frameNumber;
	}
#endif
};

typedef struct XsTriggerIndicationData XsTriggerIndicationData;

#endif // file guard
