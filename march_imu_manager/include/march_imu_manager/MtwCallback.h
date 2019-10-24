#pragma once

#include <ros/ros.h>
#include <xsens/xscallback.h>
#include <xsens/xsmutex.h>
#include <mutex>
#include <condition_variable>
#include <list>

struct XsDataPacket;
struct XsDevice;

typedef std::pair<ros::Time, XsDataPacket> RosXsDataPacket;

class MtwCallback : public XsCallback
{
    public:
        MtwCallback(int mtwIndex, XsDevice* device, size_t maxBufferSize = 5);

        bool dataAvailable() const;

        XsDataPacket const * getOldestPacket() const;

        RosXsDataPacket next(const std::chrono::milliseconds &timeout);

        void deleteOldestPacket();

        int getMtwIndex() const;

        const XsDevice& device() const;

    protected:
        virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet);

    private:
        mutable XsMutex m_mutex;
        std::list<XsDataPacket> m_packetBuffer;
        int m_mtwIndex;
        XsDevice* m_device;

        std::mutex m_stdmutex;
        std::condition_variable m_condition;
        std::list<RosXsDataPacket> m_rosBuffer;
        size_t m_maxBufferSize;
};
