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

#ifndef __XSENS_LINUX_H
#define __XSENS_LINUX_H

/* #undef HAVE_STRTOK_S */
/* #undef HAVE_STRTOK_R */
/* #undef HAVE__WCSICMP */
/* #undef HAVE_WCSICMP */
/* #undef HAVE__WCSNICMP */
/* #undef HAVE_WCSNICMP */
#define HAVE_WCSNCASECMP
#define HAVE_WCSCASECMP
/* #undef HAVE_SPRINTF_S */
/* #undef HAVE_STRCPY_S */
/* #undef HAVE_STRCAT_S */
/* #undef HAVE_STRNCPY_S */
/* #undef HAVE_FOPEN_S */
/* #undef HAVE_STRICMP */
/* #undef HAVE__STRICMP */
#define HAVE_STRCASECMP
/* #undef HAVE_STRNICMP */
/* #undef HAVE__STRNICMP */
#define HAVE_STRNCASECMP
/* #undef HAVE_WFOPEN */
/* #undef HAVE__WFOPEN */
/* #undef HAVE__STRDUP */
#define HAVE_STRDUP
/* #undef HAVE__WCSDUP */
#define HAVE_WCSDUP
/* #undef HAVE__FULLPATH */
/* #undef HAVE_FULLPATH */
#define HAVE_REALPATH
/* #undef HAVE_WFULLPATH */
/* #undef HAVE_WFREOPEN */
/* #undef HAVE__UNLINK */
#define HAVE_UNLINK
/* #undef HAVE__ISNAN */
#define HAVE_ISNAN
/* #undef HAVE__FILENO */
#define HAVE_FILENO
/* #undef HAVE_VSNPRINTF_S */
/* #undef HAVE_VSPRINTF_S */
/* #undef HAVE___DEBUGBREAK */

#ifdef __GNUC__

#include "pstdint.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <wchar.h>
#include <assert.h>
#include <pthread.h>
#include <unistd.h>

#include "xstypedefs.h"

#define LPSTR char *
#define _T

#define _ASSERT assert
#define _ASSERTE assert
#define HMODULE void *

#define LINUX_MBS_LENGTH 256
#define LINUX_MBS_MODELENGTH 5

#ifdef __cplusplus
#define XSEXTC extern "C"
#else
#define XSEXTC
#endif

#define __compat_always_deprecated(msg) __attribute__((deprecated(msg)))
#if 0
#define __compat_deprecated(msg) __compat_always_deprecated(msg)
#else
#define __compat_deprecated(msg)
#endif
#define __compat_defdeprecated __compat_deprecated("compat should be nicely merged into xstypes")

#ifndef HAVE_SPRINTF_S
XSEXTC int32_t sprintf_s(char *dest, uint32_t num, const char *fmt, ...) __compat_defdeprecated;
#endif

#ifndef HAVE_STRCPY_S
XSEXTC int32_t strcpy_s(char *dest, uint32_t size, const char *src) __compat_defdeprecated;

#ifdef __cplusplus
template <uint32_t size>
int32_t strcpy_s(char (&dest)[size], const char *src) __compat_defdeprecated;
template <uint32_t size>
int32_t strcpy_s(char (&dest)[size], const char *src)
{
	if (!src) return EINVAL;
	if (size == 0 || size < strlen(src) + 1) return ERANGE;

	strcpy(dest, src);

	return 0;
}
#endif
#endif

#ifndef HAVE_STRCAT_S
XSEXTC int32_t strcat_s(char *dest, uint32_t size, const char *src) __compat_defdeprecated;

#ifdef __cplusplus
template <uint32_t size>
int32_t strncpy_s(char (&dest)[size], const char *src, uint32_t count) __compat_defdeprecated;
template <uint32_t size>
int32_t strncpy_s(char (&dest)[size], const char *src, uint32_t count)
{
	if (!src) return EINVAL;
	uint32_t itemsToAdd = (count < strlen(src) ? count : strlen(src));
	if (size == 0 || size < itemsToAdd + 1) return ERANGE;

	strncpy(dest, src, count);

	return 0;
}
#endif
#endif

#ifndef HAVE_STRNCPY_S
XSEXTC int32_t strncpy_s(char *dest, uint32_t size, const char *src, uint32_t count) __compat_defdeprecated;

#ifdef __cplusplus
template <uint32_t size>
int32_t strcat_s(char (&dest)[size], const char *src) __compat_defdeprecated;
template <uint32_t size>
int32_t strcat_s(char (&dest)[size], const char *src)
{
	if (!src) return EINVAL;
	if (size == 0 || size < strlen(src) + strlen(dest) + 1) return ERANGE;

	strcat (dest, src);

	return 0;
}
#endif
#endif

#ifndef HAVE_FOPEN_S
XSEXTC int fopen_s(FILE **file, const char *filename, const char *mode) __compat_deprecated("Use XsFile instead");
#endif

#ifndef HAVE__STRICMP
#define _stricmp strcasecmp
#endif
#ifndef HAVE_STRICMP
#define stricmp strcasecmp
#endif
#ifndef HAVE__STRNICMP
#define _strnicmp strncasecmp
#endif
#ifndef HAVE_STRICMP
#define strnicmp strncasecmp
#endif

/* not available on cygwin */
#ifdef HAVE_WCSCASECMP
#define _wcsicmp wcscasecmp
#else
#define _wcsicmp(s1, s2)		compat_wcsnicmp(s1, s2, 0x7fffffff)
XSEXTC int compat_wcsnicmp(const wchar_t* s1, const wchar_t* s2, size_t length) __compat_defdeprecated;
#endif

/* not available on cygwin */
#ifdef HAVE_WCSNCASECMP
#define _wcsnicmp wcsncasecmp
#else
#define _wcsnicmp(s1, s2, n)	compat_wcsnicmp(s1, s2, n)
XSEXTC int compat_wcsnicmp(const wchar_t* s1, const wchar_t* s2, size_t length) __compat_defdeprecated;
#endif

#define _wfopen wfopen
#ifndef HAVE_WFOPEN
#define wfopen compat_wfopen
XSEXTC FILE *compat_wfopen(const wchar_t *filename, const wchar_t *mode) __compat_deprecated("Use XsFile instead");
#endif

#define _wfreopen wfreopen
#ifndef HAVE_WFREOPEN
#define wfreopen compat_wfreopen
XSEXTC FILE *compat_wfreopen(const wchar_t *path, const wchar_t *mode, FILE *) __compat_deprecated("Use XsFile instead");
#endif

#define _set_printf_count_output(x)

#ifndef HAVE__STRDUP
#define _strdup strdup
#endif

#ifndef HAVE__WCSDUP
#ifdef HAVE_WCSDUP
#define _wcsdup wcsdup
#else
#define _wcsdup compat_wcsdup
#endif
#endif
XSEXTC wchar_t *compat_wcsdup(const wchar_t *data) __compat_defdeprecated;

#ifndef HAVE_FULLPATH
#define fullpath(src,dest,len) realpath(src,dest)
#endif
#ifndef HAVE__FULLPATH
#define _fullpath fullpath
#endif

#ifndef HAVE__WFULLPATH
#define _wfullpath compat_wfullpath
XSEXTC int32_t compat_wfullpath(wchar_t *resolvedPath, const wchar_t *path, size_t length) __compat_deprecated("Use Xsfile_fullPath instead");
#endif

#ifndef HAVE_STRTOK_S
XSEXTC char *strtok_s(char *token, const char *delim, char **context) __compat_defdeprecated;
#endif

#ifndef _finite
#define _finite compat_finite
XSEXTC int compat_finite(XsReal x) __compat_deprecated("Use XsMath_isFinite instead");
#endif

#define LOBYTE(w)	((uint8_t)((w) & 0xff))
#define HIBYTE(w)	((uint8_t)(((w) >> 8) & 0xff))

#define __assume(val)

#if !defined(HAVE_CDECL)
#	if !defined(__cdecl)
#		if defined(__x86_64__)
#			define __cdecl
#		else
#			define __cdecl __attribute__((cdecl))
#		endif
#	endif
#	if !defined(__stdcall)
#		if defined(__x86_64__)
#			define __stdcall
#		else
#			define __stdcall __attribute__((stdcall))
#		endif
#	endif
#endif

#ifndef HAVE__UNLINK
#define _unlink unlink
#endif
#ifndef HAVE___DEBUGBREAK
#define __debugbreak()
#endif
#ifndef HAVE__FILENO
#define _fileno fileno
#endif
#ifndef HAVE__STRDUP
#define _strdup strdup
#endif
#ifndef HAVE__ISNAN
#define _isnan isnan
#endif

#ifndef GetRValue
/* The others will most likely not be defined either */
#define GetRValue(rgb) ((uint8_t)(rgb))
#define GetGValue(rgb) ((uint8_t)(((uint32_t)(rgb)) >> 8))
#define GetBValue(rgb) ((uint8_t)(((uint32_t)(rgb)) >> 16))
#endif

typedef pthread_mutex_t CRITICAL_SECTION __compat_deprecated("Use XsMutex instead");

static inline void InitializeCriticalSection(CRITICAL_SECTION *cs) __compat_deprecated("Use XsMutex instead");
static inline void DeleteCriticalSection(CRITICAL_SECTION *cs) __compat_deprecated("Use XsMutex instead");
static inline void EnterCriticalSection(CRITICAL_SECTION *cs) __compat_deprecated("Use XsMutex instead");
static inline int TryEnterCriticalSection(CRITICAL_SECTION *cs) __compat_deprecated("Use XsMutex instead");
static inline void LeaveCriticalSection(CRITICAL_SECTION *cs) __compat_deprecated("Use XsMutex instead");

static inline void InitializeCriticalSection(CRITICAL_SECTION *cs)	{ pthread_mutex_init(cs, 0); }
static inline void DeleteCriticalSection(CRITICAL_SECTION *cs)		{ pthread_mutex_destroy(cs); }
static inline void EnterCriticalSection(CRITICAL_SECTION *cs)		{ pthread_mutex_lock(cs); }
static inline int TryEnterCriticalSection(CRITICAL_SECTION *cs)		{ return (pthread_mutex_trylock(cs) == 0); }
static inline void LeaveCriticalSection(CRITICAL_SECTION *cs)		{ pthread_mutex_unlock(cs); }

#ifndef DWORD
#	define DWORD uint32_t
#endif

XSEXTC int IsBadWritePtr(void *p, size_t size);

#ifndef HAVE_VSNPRINTF_S
#define _TRUNCATE 1
#define vsnprintf_s(str, size, thingy, fmt, vp) vsnprintf(str, size, fmt, vp)
#endif
#ifndef HAVE_VSPRINTF_S
#define vsprintf_s(str, size, format, args) vsprintf(str, format, args)
#endif

// MRO (10/24/2013): Linux compiler does not support override keyword yet
// \todo Remove override define when linux compiler supports it
#ifdef __GNUC__
#if (__GNUC__ >= 4) && (__GNUC_MINOR__ >= 7)
	// override keyword supported
#else
	#define override
#endif
#endif

#endif // __GNUC__
#endif // __XSENS_LINUX_H
