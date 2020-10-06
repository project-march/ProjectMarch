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

#ifndef XSLIBRARYLOADER_H
#define XSLIBRARYLOADER_H

#include "xstypesconfig.h"
#include "xsstring.h"

struct XsLibraryLoader;

#ifdef __cplusplus
extern "C" {
#else
#define XSLIBRARYLOADER_INITIALIZER { NULL }
typedef struct XsLibraryLoader XsLibraryLoader;
#endif

XSTYPES_DLL_API int XsLibraryLoader_load(XsLibraryLoader* thisp, const XsString* libraryName);
XSTYPES_DLL_API void* XsLibraryLoader_resolve(const XsLibraryLoader* thisp, const char *functionName);
XSTYPES_DLL_API int XsLibraryLoader_unload(XsLibraryLoader* thisp);
XSTYPES_DLL_API int XsLibraryLoader_isLoaded(const XsLibraryLoader* thisp);
XSTYPES_DLL_API void XsLibraryLoader_getErrorString(XsString* error);

#ifdef __cplusplus
}
#endif

/*! \brief The Xsens dynamic library loader base class
*/
struct XsLibraryLoader {
#ifdef __cplusplus
public:
	/*! \brief Create a library loader */
	inline XsLibraryLoader() :
		m_handle(NULL)
	{
		// avoid compiler warnings about
		// an unused handle. It is used in the c implementations
		(void)m_handle;
	}

	/*! \brief Destroy a library loader */
	inline ~XsLibraryLoader()
	{
		unload();
	}

	/*! \brief Load the library
	  \param[in] libraryName the name of the library to load
	  \return true if the library could be loaded, false otherwise
	*/
	inline bool load(const XsString& libraryName)
	{
		return XsLibraryLoader_load(this, &libraryName) != 0;
	}

	/*! \brief Return true if a library has been loaded

	  \return true if a library has been loaded, false otherwise
	*/
	inline bool isLoaded() const
	{
		return XsLibraryLoader_isLoaded(this) != 0;
	}

	/*! \brief Resolve a function from the library

	  \param[in] functionName the name of the function to resolve
	  \return a pointer to the resolved function, NULL if nothing could be resolved
	*/
	inline void* resolve(const char *functionName) const
	{
		return XsLibraryLoader_resolve(this, functionName);
	}

	/*! \brief Unload the loaded library
	*/
	inline void unload() throw()
	{
		XsLibraryLoader_unload(this);
	}

	/*! \brief Return a string describing the error that occurred

	  Use this function after a function returned with an error to
	  receive some extra information about what went wrong.

	  \returns a string describing the error that occurred
	*/
	inline static XsString errorString()
	{
		XsString rv;
		XsLibraryLoader_getErrorString(&rv);
		return rv;
	}
private:
#endif
	void* m_handle;
};

#endif //XSLIBRARYLOADER_H
