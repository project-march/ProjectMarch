#pragma once

#include <deque>

#include <ros/ros.h>

#include <xsens/xscallback.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xsdevice.h>
#include <xsens/xsmutex.h>

class Mtw : public XsCallback
{
    public:
        Mtw(XsDevice* device, size_t maxBufferSize = 100);

        /**
         * Returns whether any new packets are available.
         */
        bool dataAvailable() const;

        /**
         * Returns the oldest packet received from the MTw.
         * Does not delete it.
         */
        const XsDataPacket* getOldestPacket() const;

        /**
         * Deletes the oldest packet received from the MTw.
         */
        void deleteOldestPacket();

        /**
         * Returns the device id of the MTw. This id can also be found on the
         * back of the device.
         */
        const XsDeviceId getId() const;

    protected:
        /**
         * Callback when new packets are available. This runs in a seperate
         * thread and only stores the packets in the buffer.
         */
        virtual void onLiveDataAvailable(XsDevice* device, const XsDataPacket* packet);

    private:
        /**
         * Configures the MTw to output orientation, velocity and acceleration.
         */
        void configure();

        mutable XsMutex m_mutex;
        std::deque<XsDataPacket> m_packetBuffer;
        XsDevice* m_device;

        size_t m_maxBufferSize;
};
