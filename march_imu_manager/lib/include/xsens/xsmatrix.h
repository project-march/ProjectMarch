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

#ifndef XSMATRIX_H
#define XSMATRIX_H

#include "xsmath.h"
#include <math.h>
#include <stddef.h>

struct XsMatrix;
struct XsEuler;
struct XsQuaternion;

#ifdef __cplusplus
extern "C" {
#else
#define XSMATRIX_INITIALIZER	{ NULL, 0, 0, 0, XSDF_Managed }
typedef struct XsMatrix XsMatrix;
#endif

XSTYPES_DLL_API void XsMatrix_ref(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, XsReal* buffer, XsDataFlags flags);
XSTYPES_DLL_API void XsMatrix_construct(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, const XsReal* src, XsSize srcStride);
XSTYPES_DLL_API void XsMatrix_assign(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, const XsReal* src, XsSize srcStride);
XSTYPES_DLL_API void XsMatrix_destruct(XsMatrix* thisPtr);
XSTYPES_DLL_API void XsMatrix_copy(XsMatrix* copy, XsMatrix const* src);
XSTYPES_DLL_API void XsMatrix_setZero(XsMatrix* thisPtr);
XSTYPES_DLL_API int XsMatrix_empty(const XsMatrix* thisPtr);
XSTYPES_DLL_API void XsMatrix_multiplyScalar(const XsMatrix* thisPtr, XsReal scalar, XsMatrix* dest);
XSTYPES_DLL_API XsSize XsMatrix_offset(const XsMatrix* thisPtr, XsSize row, XsSize column);
XSTYPES_DLL_API XsReal XsMatrix_value(const XsMatrix* thisPtr, XsSize row, XsSize column);
XSTYPES_DLL_API void XsMatrix_setValue(XsMatrix* thisPtr, XsSize row, XsSize column, XsReal value);
XSTYPES_DLL_API int XsMatrix_dimensionsMatch(const XsMatrix* thisPtr, XsSize rows, XsSize columns);
XSTYPES_DLL_API void XsMatrix_fromQuaternion(XsMatrix* thisPtr, const struct XsQuaternion* quat);
XSTYPES_DLL_API void XsMatrix_swap(XsMatrix* a, XsMatrix* b);

#define XsMatrix_offsetM(thisPtr, row, column)			(thisPtr->m_stride*row + column)

#ifdef __cplusplus
} // extern "C"
#endif
#ifndef XSENS_NO_PACK
#pragma pack(push, 1)
#endif
struct XsMatrix {
	//lint --e{613}
XSCPPPROTECTED
	XsReal* const m_data;		//!< Contained data
	const XsSize m_rows;		//!< Number of rows in the matrix
	const XsSize m_cols;		//!< Number of columns in the matrix
	const XsSize m_stride;		//!< Number of items per row in memory (usually equal to cols but not always)
	const int m_flags;			//!< Flags for data management

#ifdef __cplusplus
	//! \brief Return the data management flags of the matrix.
	inline int flags() { return m_flags; }
public:
	/*! \brief Initialize an XsMatrix object with the specified number of \a rows and \a cols */
	inline explicit XsMatrix(XsSize rows = 0, XsSize cols = 0, XsSize strde = 0, const XsReal* dat = 0)
		: m_data(0)
		, m_rows(0)
		, m_cols(0)
		, m_stride(0)
		, m_flags(0)
	{
		if (rows && cols)
			XsMatrix_construct(this, rows, cols, strde?strde:cols, dat, 0);
	}

	/*! \brief Initialize an XsMatrix object from the \a other XsMatrix */
	inline XsMatrix(const XsMatrix& other)
		: m_data(0)
		, m_rows(0)
		, m_cols(0)
		, m_stride(0)
		, m_flags(0)
	{
		XsMatrix_copy(this, &other);
	}

	/*! \brief Initialize an XsMatrix object that references the data passed in \a ref. \a rows, \a cols and \a stride can be used to specify the layout of the data */
	inline explicit XsMatrix(XsReal* ref, XsSize rows, XsSize cols, XsSize stride, XsDataFlags flags = XSDF_None)
		: m_data(ref)
		, m_rows(rows)
		, m_cols(cols)
		, m_stride(stride)
		, m_flags(flags)
	{
	}

	/*! \brief Initialize a copy of \a other in an XsMatrix object that references the data passed in \a ref. \a rows, \a cols and \a stride can be used to specify the layout of the data */
	inline explicit XsMatrix(const XsMatrix& other, XsReal* ref, XsSize rows, XsSize cols, XsSize stride, XsDataFlags flags = XSDF_None)
		: m_data(ref)
		, m_rows(rows)
		, m_cols(cols)
		, m_stride(stride)
		, m_flags(flags)
	{
		XsMatrix_copy(this, &other);
	}

	//! \brief \copybrief XsMatrix_fromQuaternion
	inline explicit XsMatrix(const XsQuaternion& quat)
		: m_data(0)
		, m_rows(0)
		, m_cols(0)
		, m_stride(0)
		, m_flags(0)
	{
		XsMatrix_fromQuaternion(this, &quat);
	}

	//! \copydoc XsMatrix_destruct
	inline ~XsMatrix()
	{
		XsMatrix_destruct(this);
	}

	/*! \brief Resize the matrix to the specified number of \a rows and \a cols, destroying its current contents */
	inline void setSize(XsSize rows, XsSize cols, XsSize stride = 0)
	{
		XsMatrix_assign(this, rows, cols, stride, 0, 0);
	}

	/*! \brief \copybrief XsMatrix_copy */
	inline XsMatrix& operator=(const XsMatrix& other)
	{
		//lint --e{1529} self-assignment checked by copy function
		XsMatrix_copy(this, &other);
		return *this;
	}

	//! \brief \copybrief XsMatrix_empty
	inline bool empty() const
	{
		return 0 != XsMatrix_empty(this);
	}

	//! \brief \copybrief XsMatrix_setZero
	inline void setZero()
	{
		XsMatrix_setZero(this);
	}

	//! \copydoc XsMatrix_offset */
	inline XsSize offset(XsSize row, XsSize column) const
	{
		return XsMatrix_offset(this, row, column);
	}

	/*! \brief Returns the value at \a row and \a column in the matrix */
	inline XsReal value(XsSize row, XsSize column) const
	{
		return m_data[XsMatrix_offset(this, row, column)];
	}

	/*! \brief Sets the \a value at \a row and \a column in the matrix */
	inline void setValue(XsSize row, XsSize column, XsReal value)
	{
		m_data[XsMatrix_offsetM(this, row, column)] = value;
	}

	/*! \brief Returns a pointer to the data in \a row */
	inline const XsReal* operator[](XsSize row) const
	{
		return &m_data[XsMatrix_offsetM(this, row, 0)];
	}

	/*! \brief Returns a reference to the data in \a row */
	inline XsReal* operator[](XsSize row)
	{
		return &m_data[XsMatrix_offsetM(this, row, 0)];
	}

	/*! \brief \copybrief XsMatrix_multiplyScalar */
	inline XsMatrix operator*(XsReal scalar) const
	{
		XsMatrix tmp(m_rows, m_cols);
		XsMatrix_multiplyScalar(this, scalar, &tmp);
		return tmp;
	}

	/*! \brief \copybrief XsMatrix_fromQuaternion */
	inline XsMatrix& fromQuaternion(const XsQuaternion& quat)
	{
		XsMatrix_fromQuaternion(this, &quat);
		return *this;
	}

	/*! \brief Fill the matrix with zeroes */
	inline void zero()
	{
		for (XsSize r = 0; r < m_rows; ++r)
			for (XsSize c = 0; c < m_cols; ++c)
				m_data[XsMatrix_offsetM(this, r, c)] = XsMath_zero;
	}

	/*! \brief Return the number of rows in the matrix */
	inline XsSize rows() const
	{
		return m_rows;
	}

	/*! \brief Return the number of columns in the matrix */
	inline XsSize cols() const
	{
		return m_cols;
	}

	/*! \brief Return the stride of the matrix.
		\details The stride of a matrix is for internal administration. It defines the number of items
		in a row in the data buffer. This is always greater than or equal to the number of columns.
		Especially for matrices that reference a part of another matrix this may differ from the
		cols() value.
		\returns The stride of the matrix.
	*/
	inline XsSize stride() const
	{
		return m_stride;
	}

	//! \brief Return a const pointer to the internal data
	inline const XsReal* data() const
	{
		return m_data;
	}

	//! \brief Returns true if \a other is numerically identical to this
	inline bool operator ==(const XsMatrix& other) const
	{
		if (this == &other)
			return true;
		if (m_rows != other.m_rows || m_cols != other.m_cols)
			return false;
		for (XsSize r = 0; r < m_rows; ++r)
			for (XsSize c = 0; c < m_cols; ++c)
				if (m_data[XsMatrix_offsetM(this, r, c)] != other.m_data[XsMatrix_offsetM((&other), r, c)])
					return false;
		return true;
	}

	//! \brief Returns true if the values of this and \a other are within \a tolerance of each other
	inline bool compare(const XsMatrix& other, XsReal tolerance) const
	{
		if (this == &other)
			return true;
		if (m_rows != other.m_rows || m_cols != other.m_cols)
			return false;
		for (XsSize r = 0; r < m_rows; ++r)
			for (XsSize c = 0; c < m_cols; ++c)
				if (fabs(m_data[XsMatrix_offsetM(this, r, c)] - other.m_data[XsMatrix_offsetM((&other), r, c)]) > tolerance)
					return false;
		return true;
	}

	/*! \brief Swap the contents of \a b with this
		\details This function swaps the internal buffers so no actual data is moved around. For unmanaged
		data an elementwise swap is done, but only if the matrices are the same size.
		\param b Object whose contents will be swapped with this
	*/
	inline void swap(XsMatrix& b)
	{
		XsMatrix_swap(this, &b);
	}

#endif
};
#ifndef XSENS_NO_PACK
#pragma pack(pop)
#endif

#ifdef __cplusplus
//! \brief Multiplies all values in the matrix \a m by \a scalar
inline XsMatrix operator *(XsReal scalar, const XsMatrix &m)
{
	return (m*scalar);
}
#endif

#endif // file guard
