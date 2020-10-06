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

#ifndef XSINTLIST_H
#define XSINTLIST_H

#include "xsintarray.h"

#define XsIntList	XsIntArray

#ifndef __cplusplus
// obsolete:
#define XSINTLIST_INITIALIZER		XsIntArray_INITIALIZER
#define XsIntList_construct(thisPtr, sz, src)	XsIntArray_construct(thisPtr, sz, src)
#define XsIntList_assign(thisPtr, sz, src)		XsArray_assign(thisPtr, sz, src)
#define XsIntList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsIntList_copy(thisPtr, copy)			XsArray_copy(copy, thisPtr)
#define XsIntList_append(thisPtr, other)		XsArray_append(thisPtr, other)
#define XsIntList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsIntList_popBack(thisPtr, count)		XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsIntList_swap(a, b)					XsArray_swap(a, b)
#define XsIntList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)
#define XsIntList_find(thisPtr, needle)			XsArray_find(thisPtr, needle)

#endif
#endif // file guard
