#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <ros/ros.h>

#include <xsensdeviceapi.h>
#include <xstypes.h>

#include "march_imu_manager/Mtw.h"

/**
 * The wireless master class that connects to MTws and publishes the data on
 * '/march/imu/' ROS topics.
 */
class WirelessMaster : public XsCallback
{
    public:
        WirelessMaster(ros::NodeHandle* node);
        ~WirelessMaster();

        /**
         * Finds and constructs a wireless master.
         * This method must be called first before everything else.
         */
        int init();

        /**
         * Configures the wireless master with given settings.
         * Can only be configured once init() succeeded.
         *
         * @param updateRate the desired update rate of the master in Hz.
         * @param channel the desired radio channel, defaults to 25.
         * @returns error code, 0 if successfull, -1 otherwise.
         */
        int configure(const int updateRate, const int channel = 25);

        /**
         * Waits for the given amount of MTws to connect.
         * Blocks the thread until the amount has connected.
         *
         * @param connections the amount of connections to wait for, defaults to 1.
         */
        void waitForConnections(const size_t connections = 1);

        /**
         * Starts the measurement of the MTw. This is required in order to
         * publish anything in the update loop. Once the measurement is started
         * no MTws will be able to connect.
         *
         * @returns true if successfull, false otherwise.
         */
        bool startMeasurement();

        /**
         * Returns whether the MTws are measuring.
         */
        bool isMeasuring() const;

        /**
         * Publishes all data from the MTws. This is supposed to be called in
         * an update loop.
         */
        void update();

        /**
         * Finds the closest supported update rate to the given desired rate.
         *
         * @param supportedUpdateRates rates that are supported by the master.
         * @param desiredUpdateRate rate that is desired.
         */
        static int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

    protected:
        /**
         * Callback for when new MTws connect or disconnect.
         * Runs in a seperate thread.
         */
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
