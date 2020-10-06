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

#ifndef XSRANGE_H
#define XSRANGE_H

#include "xstypesconfig.h"

#ifdef __cplusplus
extern "C" {
#else
#define XSRANGE_INITIALIZER	{ 0, -1 }
#endif

struct XsRange;

XSTYPES_DLL_API int XsRange_count(const struct XsRange* thisPtr);
XSTYPES_DLL_API int XsRange_contains(const struct XsRange* thisPtr, int i);
XSTYPES_DLL_API int XsRange_interval(const struct XsRange* thisPtr);
XSTYPES_DLL_API void XsRange_setRange(struct XsRange* thisPtr, int f, int l);
XSTYPES_DLL_API int XsRange_empty(const struct XsRange* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsRange {
#ifdef __cplusplus
	//! \brief Constructs a range starting at \a f and ending at \a l. Default values are 0 and -1 respectively.
	inline explicit XsRange(int f = 0, int l = -1)
		: m_first(f)
		, m_last(l)
	{}

	//! \brief Constructs a range based upon \a other
	inline XsRange(const XsRange& other)
		: m_first(other.m_first)
		, m_last(other.m_last)
	{}

	//! \brief Assigns the range \a other to this
	inline XsRange& operator = (const XsRange& other)
	{
		//lint --e{1529} trivial assignment
		m_first = other.m_first;
		m_last = other.m_last;
		return *this;
	}

	//! \brief \copybrief XsRange_count
	inline int count() const
	{
		return XsRange_count(this);
	}

	//! \brief \copybrief XsRange_interval
	inline int interval() const
	{
		return XsRange_interval(this);
	}

	//! \brief \copybrief XsRange_contains
	inline bool contains(int i) const
	{
		return 0 != XsRange_contains(this, i);
	}

	//! \brief Set the range to \a f - \a l
	inline void setRange(int f, int l)
	{
		m_first = f;
		m_last = l;
	}

	//! \brief \copybrief XsRange_empty
	inline bool empty() const
	{
		return 0 != XsRange_empty(this);
	}

	//! \brief Return the \e first value of the range
	inline int first() const
	{
		return m_first;
	}

	//! \brief Return the \e last value of the range
	inline int last() const
	{
		return m_last;
	}

	//! \brief Return true if this is equal to other
	bool operator == (const XsRange& other) const
	{
		return m_first == other.m_first && m_last == other.m_last;
	}

	//! \brief Return true if start of this is less than start of other
	bool operator < (const XsRange& other) const
	{
		return m_first < other.m_first;
	}

private:
#endif

	int m_first;	//!< Storage for the lower end of the range
	int m_last;		//!< Storage for the upper end of the range
};

typedef struct XsRange XsRange;

#if defined(__cplusplus) && !defined(XSENS_NO_STL)
#include <ostream>

template<typename _CharT, typename _Traits>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& o, XsRange const& xs)
{
	return (o << "(" << xs.first() << ", " << xs.last() << "]");
}
#endif

#endif // file guard
