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

#ifndef XSOUTPUTCONFIGURATION_H
#define XSOUTPUTCONFIGURATION_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsdataidentifier.h"

#define XS_MAX_OUTPUTCONFIGURATIONS			(32)

#ifdef __cplusplus
extern "C" {
#else
#define XSOUTPUTCONFIGURATION_INITIALIZER		{ XDI_None, 0 }
#endif

struct XsOutputConfiguration;

XSTYPES_DLL_API void XsOutputConfiguration_swap(struct XsOutputConfiguration* a, struct XsOutputConfiguration* b);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief Single data type output configuration
	\details This structure contains a single data type and the frequency at which it should be produced.
	If m_frequency is 0xFFFF and the %XsOutputConfiguration is used for input, the device will configure
	itself to its maximum frequency for the data type. If it is 0xFFFF and reported by the device,
	the data has no maximum frequency, but is sent along with appropriate packets (ie. packet counter)
*/
struct XsOutputConfiguration {
	XsDataIdentifier m_dataIdentifier;	//!< The data identifier
	uint16_t m_frequency;				//!< The frequency

#ifdef __cplusplus
	//! Constructor, initializes to an empty object
	XsOutputConfiguration()
		: m_dataIdentifier(XDI_None), m_frequency(0) {}

	//! Constructor, initializes to specified values
	XsOutputConfiguration(XsDataIdentifier di, uint16_t freq)
		: m_dataIdentifier(di), m_frequency(freq)
	{}

	//! Comparison operator
	bool operator == (const XsOutputConfiguration& other) const
	{
		return m_dataIdentifier == other.m_dataIdentifier && m_frequency == other.m_frequency;
	}
#endif
};
typedef struct XsOutputConfiguration XsOutputConfiguration;

#endif // file guard
