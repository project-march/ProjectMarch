#pragma once

#include <xsensdeviceapi.h>
#include <xstypes.h>
#include <set>

typedef std::set<XsDevice*> XsDeviceSet;

class WirelessMasterCallback : public XsCallback
{
    public:
        XsDeviceSet getWirelessMTWs() const;

    protected:
        virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState);

    private:
        mutable XsMutex m_mutex;
        XsDeviceSet m_connectedMTWs;
};
