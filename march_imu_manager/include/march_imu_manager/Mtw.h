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

        bool dataAvailable() const;

        const XsDataPacket* getOldestPacket() const;
        void deleteOldestPacket();

        const XsDeviceId getId() const;

    protected:
        virtual void onLiveDataAvailable(XsDevice* device, const XsDataPacket* packet);

    private:
        void configure();

        mutable XsMutex m_mutex;
        std::deque<XsDataPacket> m_packetBuffer;
        XsDevice* m_device;

        size_t m_maxBufferSize;
};
