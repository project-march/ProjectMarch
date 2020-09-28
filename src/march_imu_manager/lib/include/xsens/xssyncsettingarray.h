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

#ifndef XSSYNCSETTINGARRAY_H
#define XSSYNCSETTINGARRAY_H

#include "xsarray.h"

#ifdef __cplusplus
#include "xssyncsetting.h"
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsSyncSettingArrayDescriptor;

#ifndef __cplusplus
#define XSSYNCSETTINGSARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsSyncSettingArrayDescriptor)

struct XsSyncSetting;
XSARRAY_STRUCT(XsSyncSettingArray, struct XsSyncSetting);
typedef struct XsSyncSettingArray XsSyncSettingArray;

XSTYPES_DLL_API void XsSyncSettingArray_construct(XsSyncSettingArray* thisPtr, XsSize count, struct XsSyncSetting const* src);
#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsSyncSettingArray : public XsArrayImpl<XsSyncSetting, g_xsSyncSettingArrayDescriptor, XsSyncSettingArray> {
	//! \brief Constructs an XsSyncSettingArray
	inline explicit XsSyncSettingArray(XsSize sz = 0, XsSyncSetting const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsSyncSettingArray as a copy of \a other
	inline XsSyncSettingArray(XsSyncSettingArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsSyncSettingArray that references the data supplied in \a ref
	inline explicit XsSyncSettingArray(XsSyncSetting* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}

#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsSyncSettingArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsSyncSettingArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
};
#endif
#endif // file guard
