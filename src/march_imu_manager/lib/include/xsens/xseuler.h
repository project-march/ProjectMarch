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

#ifndef XSEULER_H
#define XSEULER_H

#include "xsmath.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#else
#define XSEULER_INITIALIZER { { { XsMath_zero,XsMath_zero,XsMath_zero } } }
#endif

struct XsEuler;
struct XsQuaternion;
struct XsMatrix;

XSTYPES_DLL_API void XsEuler_destruct(struct XsEuler* thisPtr);
XSTYPES_DLL_API int XsEuler_empty(const struct XsEuler* thisPtr);
XSTYPES_DLL_API void XsEuler_fromQuaternion(struct XsEuler* thisPtr, const struct XsQuaternion* quat);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsEuler {
#ifdef __cplusplus
	//! \brief Constructor that creates an Euler object with all angles 0
	inline XsEuler() : m_x(XsMath_zero), m_y(XsMath_zero), m_z(XsMath_zero) {}
	//! \brief Constructor that creates an Euler object the specified angles
	inline XsEuler(XsReal x_, XsReal y_, XsReal z_) : m_x(x_), m_y(y_), m_z(z_) {}
	//! \brief Constructor that creates an Euler object from \a other
	inline XsEuler(const XsEuler& other) : m_x(other.m_x), m_y(other.m_y), m_z(other.m_z) {}

	//! \brief \copybrief XsEuler_fromQuaternion
	inline explicit XsEuler(const XsQuaternion& q)
	{
		XsEuler_fromQuaternion(this, &q);
	}

	//! \brief Assigns the \a other XsEuler object to this one
	inline XsEuler& operator=(const XsEuler& other)
	{
		//lint --e{1529} trivial assignment
		m_x = other.m_x;
		m_y = other.m_y;
		m_z = other.m_z;
		return *this;
	}

	//! \brief Returns the \a index'th euler angle in the object
	inline XsReal operator[](XsSize index) const
	{
		assert (index <= 2);
		return m_data[index];
	}

	//! \brief Returns a reference to the \a index'th euler angle in the object
	inline XsReal &operator[](XsSize index)
	{
		assert (index <= 2);
		return m_data[index];
	}

	//! \brief Returns true if all angles in this object are zero
	inline bool empty() const
	{
		return m_x == XsMath_zero && m_y == XsMath_zero && m_z == XsMath_zero;
	}

	//! \brief Return a const pointer to the internal data
	inline const XsReal* data() const
	{
		return m_data;
	}

	//! \brief \copybrief XsEuler_fromQuaternion
	inline XsEuler& fromQuaternion(const XsQuaternion& quat)
	{
		XsEuler_fromQuaternion(this, &quat);
		return *this;
	}

	/*! \brief Returns true if the values in \a other are exactly equal to this
	*/
	inline bool operator == (const XsEuler& other) const
	{
		return m_roll == other.m_roll && m_pitch == other.m_pitch && m_yaw == other.m_yaw;
	}

	/*! \brief Returns true if the values in \a other are different from this
	*/
	inline bool operator != (const XsEuler& other) const
	{
		return m_roll != other.m_roll || m_pitch != other.m_pitch || m_yaw != other.m_yaw;
	}

	//! \brief Returns the roll or x value
	inline XsReal roll() const { return m_roll; }
	//! \brief Returns the pitch or y value
	inline XsReal pitch() const { return m_pitch; }
	//! \brief Returns the yaw or z value
	inline XsReal yaw() const { return m_yaw; }

	//! \brief Returns the x or roll value
	inline XsReal x() const { return m_x; }
	//! \brief Returns the y or pitch value
	inline XsReal y() const { return m_y; }
	//! \brief Returns the z or yaw value
	inline XsReal z() const { return m_z; }

	//! \brief Returns true if the values of this and \a other are within \a tolerance of each other
	inline bool compare(const XsEuler& other, XsReal tolerance) const
	{
		return	fabs(m_x - other.m_x) <= tolerance &&
				fabs(m_y - other.m_y) <= tolerance &&
				fabs(m_z - other.m_z) <= tolerance;
	}

private:
#endif

	union {
		struct {
			XsReal m_x;		//!< Stores the x component of the euler triplet
			XsReal m_y;		//!< Stores the y component of the euler triplet
			XsReal m_z;		//!< Stores the z component of the euler triplet
		};
		struct {
			XsReal m_roll;		//!< Stores the roll component of the euler triplet
			XsReal m_pitch;		//!< Stores the pitch component of the euler triplet
			XsReal m_yaw;		//!< Stores the yaw component of the euler triplet
		};
		XsReal m_data[3];	//!< Stores the euler triplet in an array of three elements
	};
};

typedef struct XsEuler XsEuler;

#endif // file guard
