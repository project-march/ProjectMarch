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

#ifndef XSDATAFORMAT_H
#define XSDATAFORMAT_H

#include "xstypesconfig.h"
#include "xsoutputmode.h"
#include "xsoutputsettings.h"

/*! \brief A structure for storing data formats. */
struct XsDataFormat {
	XsOutputMode		m_outputMode;		//!< The stored output mode
	XsOutputSettings	m_outputSettings;	//!< The stored output settings

#ifdef __cplusplus
	/*! \brief Construct an XsDataFormat specifier

	  \param mode the outputmode
	  \param settings the output settings
	*/
	inline explicit XsDataFormat(const XsOutputMode mode = XS_DEFAULT_OUTPUT_MODE, const XsOutputSettings settings = XS_DEFAULT_OUTPUT_SETTINGS)
				: m_outputMode(mode), m_outputSettings(settings) {}

	//! Copy constructor
	inline XsDataFormat(const XsDataFormat& other) : m_outputMode(other.m_outputMode), m_outputSettings(other.m_outputSettings) {}

	//! Assignment operator
	inline const XsDataFormat& operator = (const XsDataFormat& other)
	{
		//lint --e{1529} assignment to self ok
		m_outputMode = other.m_outputMode;
		m_outputSettings = other.m_outputSettings;
		return *this;
	}

	//! Equality operator
	inline bool operator == (const XsDataFormat& other) const
	{
		return m_outputMode == other.m_outputMode && m_outputSettings == other.m_outputSettings;
	}
#endif
};

#endif	// file guard

