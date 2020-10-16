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

#include "xsens/xdaconfig.h"
#include "xsens/xstypes_include.h"
#include "xsens/xsaccesscontrolmode.h"
#include "xsens/xdainfo.h"
#include "xsens/xsalignmentframe.h"
#include "xsens/xscalibrateddatamode.h"
#include "xsens/xscallback.h"
#include "xsens/xscallbackplainc.h"
#include "xsens/xsconnectivitystate.h"
#include "xsens/xscoordinatesystem.h"
#include "xsens/xsdef.h"
#include "xsens/xsdeviceconfiguration.h"
#include "xsens/xsdevicemode.h"
#include "xsens/xsdevicemodeps.h"
#include "xsens/xsdevicemoder.h"
#include "xsens/xsdeviceoptionflag.h"
#include "xsens/xsdeviceptr.h"
#include "xsens/xsdeviceptrarray.h"
#include "xsens/xsdeviceptrlist.h"
#include "xsens/xsdevicestate.h"
#include "xsens/xserrormode.h"
#include "xsens/xsfilterprofile.h"
#include "xsens/xsfilterprofilearray.h"
#include "xsens/xsfilterprofilelist.h"
#include "xsens/xsfloatformat.h"
#include "xsens/xsoption.h"
#include "xsens/xscomcallbackoptions.h"
#include "xsens/xsorientationmode.h"
#include "xsens/xsprocessingflag.h"
#include "xsens/xsprotocoltype.h"
#include "xsens/xsnmeastringtype.h"
#include "xsens/xsrejectreason.h"
#include "xsens/xsresetmethod.h"
#include "xsens/xsscanner.h"
#include "xsens/xsselftestresult.h"
#include "xsens/xsstatusflag.h"
#include "xsens/xsusbhubinfo.h"
#include "xsens/xsdeviceref.h"
#include "xsens/xsnetworkscanner.h"
#include "xsens/xscontrol.h"
#include "xsens/xsdevice.h"
