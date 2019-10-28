#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <unordered_map>

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

        void waitForConnections(const size_t connections = 1);

        bool startMeasurement();

        bool isMeasuring();

        void update();

        static int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

    protected:
        virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState);

    private:
        ros::NodeHandle* m_node;

        std::mutex m_mutex;
        std::condition_variable m_cv;

        XsControl* m_control = nullptr;
        XsDevicePtr m_master = nullptr;

        std::unordered_map<uint32_t, std::unique_ptr<Mtw>> m_connectedMtws;
        std::unordered_map<uint32_t, ros::Publisher> m_publishers;
};
