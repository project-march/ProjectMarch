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

#ifndef XSBYTEARRAY_H
#define XSBYTEARRAY_H

#include "xsarray.h"
#include "pstdint.h"

#ifdef __cplusplus
#include "xsstring.h"
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsByteArrayDescriptor;

#ifndef __cplusplus
#define XSBYTEARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsByteArrayDescriptor)
XSARRAY_STRUCT(XsByteArray, uint8_t);
typedef struct XsByteArray XsByteArray;
XSTYPES_DLL_API void XsByteArray_construct(XsByteArray* thisPtr, XsSize count, uint8_t const* src);

// obsolete:
#define XsByteArray_ref(thisPtr, sz, src, flags)	XsArray_ref(thisPtr, sz, src, flags)
#define XsByteArray_assign(thisPtr, sz, src)		XsArray_assign(thisPtr, sz, src)
#define XsByteArray_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsByteArray_copy(thisPtr, copy)				XsArray_copy(copy, thisPtr)
#define XsByteArray_append(thisPtr, other)			XsArray_append(thisPtr, other)
#define XsByteArray_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsByteArray_popBack(thisPtr, count)			XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsByteArray_fromString(str, copy)			XsArray_assign(copy, str->m_size?str->m_size:1, str->m_size?str->m_data:"\0")
#define XsByteArray_swap(a, b)						XsArray_swap(a, b)
#define XsByteArray_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)

#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsByteArray : public XsArrayImpl<uint8_t, g_xsByteArrayDescriptor, XsByteArray> {
	//! \brief Constructs an XsByteArray
	inline explicit XsByteArray(XsSize sz = 0, uint8_t const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsByteArray as a copy of \a other
	inline XsByteArray(XsByteArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsByteArray that references the data supplied in \a ref
	inline explicit XsByteArray(uint8_t* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}
#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsByteArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsByteArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
	//! \brief Constructs an XsByteArray as a copy of the supplied XsString, including the terminating 0
	inline XsByteArray(XsString const& src)
		: ArrayImpl()
	{
		assign(src.size()+1, reinterpret_cast<uint8_t const*>(src.c_str()));
	}

	//! \brief Return a pointer to the internal data buffer
	inline uint8_t* data() { return begin().operator ->(); }

	//! \brief Return a pointer to the internal data buffer
	inline uint8_t const* data() const { return begin().operator ->(); }

	/*! \brief Return the data at position \a offset converted into a T
		\details This function will translate a part of the contained data into a type T. T needs to be a real
		POD type or structure with a trivial default constructor, since its contents will be completely
		overwritten by a memcpy
		\param offset The offset of the first byte to convert (in byte units, not in T units)
		\returns The converted value
	*/
	template <typename T>
	inline T getValue(XsSize offset) const
	{
		assert(offset+sizeof(T) <= size());
		T tmp;
		memcpy(&tmp, data()+offset, sizeof(T));
		return tmp;
	}

	/*! \brief Append \a value T to the byte array
		\param value The value to append to the array. The type T needs to be a POD structure or a primitive type
		since it gets copied into the array using memcpy.
	*/
	template <typename T>
	inline void appendValue(T const& value)
	{
		XsSize offset = size();
		resize(offset + sizeof(T));
		memcpy(data()+offset, &value, sizeof(T));
	}

	/*! \brief Set \a value T in the byte array at byte position \a offset
		\param value The value to append to the array. The type T needs to be a POD structure or a primitive type
		since it gets copied into the array using memcpy.
		\param offset The position of the first byte to write. The array will be enlarged if necessary
	*/
	template <typename T>
	inline void setValue(T const& value, XsSize offset)
	{
		if (size() < offset + sizeof(T))
			resize(offset + sizeof(T));
		memcpy(data()+offset, &value, sizeof(T));
	}
};
#endif
#endif // file guard
