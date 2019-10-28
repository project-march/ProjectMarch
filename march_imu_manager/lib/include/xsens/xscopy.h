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

#ifndef XSCOPY_H
#define XSCOPY_H

#define XSLISTCOPY(C)	\
	if (copy == thisPtr)\
	{\
		return;\
	}\
	C##_assign(copy, thisPtr->m_size, thisPtr->m_data);

#define XSLISTSWAP3(C, B, S)	\
	if ((!a->m_data || (a->m_flags & XSDF_Managed)) && (!b->m_data || (b->m_flags & XSDF_Managed))) {\
		B tmp;\
		*((C**) &tmp.m_data) = a->m_data;\
		*((XsSize*) &tmp.m_size) = a->m_size;\
		*((int*) &tmp.m_flags) = a->m_flags;\
		*((C**) &a->m_data) = b->m_data;\
		*((XsSize*) &a->m_size) = b->m_size;\
		*((int*) &a->m_flags) = b->m_flags;\
		*((C**) &b->m_data) = tmp.m_data;\
		*((XsSize*) &b->m_size) = tmp.m_size;\
		*((int*) &b->m_flags) = tmp.m_flags;\
	} else {	/* elementwise swap */ \
		XsSize i;\
		assert(a->m_size == b->m_size);\
		for (i = 0; i < a->m_size; ++i) S(&a->m_data[i], &b->m_data[i]);\
	}

#define XSLISTSWAP2(C, B)	XSLISTSWAP3(C, B, C##_swap)

#define XSLISTSWAP(C)	XSLISTSWAP2(C, C##List)

#endif // file guard
