// Copyright 2019 Project March
#include <xsens/xsdevice.h>
#include <xsens/xsdatapacket.h>

#include "march_imu_manager/MtwCallback.h"

MtwCallback::MtwCallback(int mtwIndex, XsDevice* device, size_t maxBufferSize) :
    m_mtwIndex(mtwIndex),
    m_device(device),
    m_maxBufferSize(maxBufferSize)
{
}

bool MtwCallback::dataAvailable() const
{
    XsMutexLocker lock(this->m_mutex);
    return !this->m_packetBuffer.empty();

    // std::unique_lock<std::mutex> lock(m_stdmutex);
    // return !m_rosBuffer.empty();
}

XsDataPacket const * MtwCallback::getOldestPacket() const
{
    XsMutexLocker lock(this->m_mutex);
    XsDataPacket const * packet = &this->m_packetBuffer.front();
    return packet;
}

// Returns empty packet on timeout
RosXsDataPacket MtwCallback::next(const std::chrono::milliseconds& timeout)
{
    RosXsDataPacket packet;

    std::unique_lock<std::mutex> lock(this->m_stdmutex);
    // XsMutexLocker lock(this->m_mutex);

    auto isRosBufferEmpty = [this] { return !this->m_rosBuffer.empty(); };
    if (this->m_condition.wait_for(lock, timeout, isRosBufferEmpty))
    {
        assert(!this->m_rosBuffer.empty());

        packet = this->m_rosBuffer.front();
        this->m_rosBuffer.pop_front();
    }

    return packet;
}

void MtwCallback::deleteOldestPacket()
{
    XsMutexLocker lock(this->m_mutex);
    this->m_packetBuffer.pop_front();
}

int MtwCallback::getMtwIndex() const
{
    return this->m_mtwIndex;
}

const XsDevice& MtwCallback::device() const
{
    assert(this->m_device != 0);
    return *this->m_device;
}

void MtwCallback::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
    // std::unique_lock<std::mutex> lock(this->m_stdmutex);
    // ros::Time now = ros::Time::now();

    XsMutexLocker lock(this->m_mutex);
    // NOTE: Processing of packets should not be done in this thread.

    this->m_packetBuffer.push_back(*packet);
    if (this->m_packetBuffer.size() > 300)
    {
        std::cout << std::endl;
        deleteOldestPacket();
    }
    /*
       assert(packet != 0);

    // Discard oldest packet if buffer full
    if (this->m_rosBuffer.size() == this->m_maxBufferSize)
    {
    this->m_rosBuffer.pop_front();
    }

    // Push new packet
    this->m_rosBuffer.push_back(RosXsDataPacket(now, *packet));

    // Manual unlocking is done before notifying, to avoid waking up
    // the waiting thread only to block again
    lock.unlock();
    this->m_condition.notify_one();
    */
}
