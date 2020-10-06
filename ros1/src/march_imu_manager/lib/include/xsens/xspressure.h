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

#ifndef XSPRESSURE_H
#define XSPRESSURE_H

#include "pstdint.h"

/*! \brief Pressure data.
	\details Contains the pressure data and the pressure age
*/
struct XsPressure {
#ifdef __cplusplus
	/*! \brief Create an XsPressure
		\param pressure the pressure
		\param age the pressure age
	*/
	explicit XsPressure(double pressure = 0, uint8_t age = 0) :
		m_pressure(pressure),
		m_pressureAge(age)
	{
	}

	/*! \brief Create a new XsPressure as copy from \a other
		\param other the pressure carrier to copy from
	*/
	inline XsPressure(XsPressure const& other) :
		m_pressure(other.m_pressure),
		m_pressureAge(other.m_pressureAge)
	{
	}

	/*! \brief Copy the data from \a other
		\param other the pressure carrier to copy from
		\return this
	*/
	inline XsPressure const & operator=(XsPressure const& other)
	{
		//lint --e{1529} trivial assignment
		m_pressure = other.m_pressure;
		m_pressureAge = other.m_pressureAge;
		return *this;
	}

	/*! \brief Return true if this is equal to \a other
		\param other the pressure carrier to compare against
		\return true if both XsPressures are equal
	*/
	inline bool operator==(XsPressure const& other) const
	{
		return other.m_pressure == m_pressure && other.m_pressureAge == m_pressureAge;
	}
#endif
	double		m_pressure;		//!< Pressure in Pascal
	uint8_t		m_pressureAge;	//!< Age of pressure data in samples
};
typedef struct XsPressure XsPressure;

#endif // file guard
