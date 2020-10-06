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

#ifndef XSATOMICINT_H
#define XSATOMICINT_H

#include "xstypesconfig.h"

#ifdef XSENS_64BIT
typedef long long XsAtomicIntValue;
#else
typedef long XsAtomicIntValue;
#endif
struct XsAtomicInt;
#ifndef __cplusplus
#define XSATOMICINT_INITIALIZER	{ 0 }
#endif

#ifdef _MSC_VER
#include <windows.h>
	#ifdef XSENS_64BIT
		#define atomicIncrement(a)	InterlockedIncrement64(&a)
		#define atomicDecrement(a)	InterlockedDecrement64(&a)
	#else
		#define atomicIncrement(a)	InterlockedIncrement(&a)
		#define atomicDecrement(a)	InterlockedDecrement(&a)
	#endif
#else
	#ifdef XSENS_64BIT
		#define atomicIncrement(a)	__sync_add_and_fetch(&a, 1)
		#define atomicDecrement(a)	__sync_sub_and_fetch(&a, 1)
	#else
		#define atomicIncrement(a)	(++a)
		#define atomicDecrement(a)	(--a)
	#endif
#endif

/*! \relates XsAtomicInt \brief Increase the value by one unit (prefix notation). */
#define XsAtomicInt_preIncrement(a) ((XsAtomicIntValue) atomicIncrement((a)->m_value))

/*! \relates XsAtomicInt \brief Decrease the value by one unit (prefix notation). */
#define XsAtomicInt_preDecrement(a) ((XsAtomicIntValue) atomicDecrement((a)->m_value))

/*! \relates XsAtomicInt \brief Increase the value by one unit (postfix notation). */
#define XsAtomicInt_postIncrement(a) ((XsAtomicIntValue) atomicIncrement((a)->m_value) - 1)

/*! \relates XsAtomicInt \brief Decrease the value by one unit (postfix notation). */
#define XsAtomicInt_postDecrement(a) ((XsAtomicIntValue) atomicDecrement((a)->m_value) + 1)

/*!	\class XsAtomicInt
	\brief Wrapper class for easy use of XsAtomicIntValue values
*/
struct XsAtomicInt {
	volatile XsAtomicIntValue m_value;				//!< The actual value

#ifdef __cplusplus
	/*! \brief Initialize the value. */
	XsAtomicInt(XsAtomicIntValue val = 0)
		: m_value(val)
	{}

	/*! \brief Set the value to the given \a val. */
	inline void setValue(XsAtomicIntValue val)
	{
		m_value = val;
	}

	/*! \brief Get the current value. */
	inline XsAtomicIntValue value() const
	{
		return (XsAtomicIntValue) m_value;
	}

	/*! \brief Increase the value by one unit (prefix). */
	inline XsAtomicIntValue operator++()
	{
		return XsAtomicInt_preIncrement(this);
	}

	/*! \brief Decrease the value by one unit (prefix). */
	inline XsAtomicIntValue operator--()
	{
		return XsAtomicInt_preDecrement(this);
	}

	/*! \brief Increase the value by one unit (postfix). */
	inline XsAtomicIntValue operator++(int)
	{
		return XsAtomicInt_postIncrement(this);
	}

	/*! \brief Decrease the value by one unit (postfix). */
	inline XsAtomicIntValue operator--(int)
	{
		return XsAtomicInt_postDecrement(this);
	}

#endif	// __cplusplus
};

typedef struct XsAtomicInt XsAtomicInt;

#endif	// file guard
