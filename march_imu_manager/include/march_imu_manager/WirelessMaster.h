#pragma once

#include <unordered_map>
#include <memory>

#include <ros/ros.h>

#include <xsensdeviceapi.h>
#include <xstypes.h>

#include "march_imu_manager/Mtw.h"

class WirelessMaster : public XsCallback
{
    public:
        WirelessMaster(ros::NodeHandle* node);
        ~WirelessMaster();

        int init();
        int configure(const int updateRate, const int channel);

        int startMeasurement();

        void update();

    protected:
        virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState);

    private:
        static int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

        ros::NodeHandle* m_node;

        XsControl* m_control = nullptr;
        XsDevicePtr m_master = nullptr;
        mutable XsMutex m_mutex;
        std::unordered_map<uint32_t, std::unique_ptr<Mtw>> m_connectedMtws;
        std::unordered_map<uint32_t, ros::Publisher> m_publishers;
};
