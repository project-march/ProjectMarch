#pragma once

#include <xsensdeviceapi.h>
#include <xstypes.h>
#include <set>

typedef std::set<XsDevice*> XsDeviceSet;

class WirelessMaster : public XsCallback
{
    public:
        WirelessMaster();
        ~WirelessMaster();
        XsDeviceSet getWirelessMTWs() const;

        int init();
        int configure(const int updateRate, const int channel);

    protected:
        virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState);

    private:
        static int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

        XsControl* m_control = nullptr;
        XsDevicePtr m_master = nullptr;
        mutable XsMutex m_mutex;
        XsDeviceSet m_connectedMTWs;
};
