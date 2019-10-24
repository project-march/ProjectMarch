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
    XsMutexLocker lock(m_mutex);
    return !m_packetBuffer.empty();

    //std::unique_lock<std::mutex> lock(m_stdmutex);
    //return !m_rosBuffer.empty();
}

XsDataPacket const * MtwCallback::getOldestPacket() const 
{
    XsMutexLocker lock(m_mutex);
    XsDataPacket const * packet = &m_packetBuffer.front();
    return packet;
}

// Returns empty packet on timeout
RosXsDataPacket MtwCallback::next(const std::chrono::milliseconds& timeout)
{
    RosXsDataPacket packet;

    std::unique_lock<std::mutex> lock(m_stdmutex);
    //XsMutexLocker lock(m_mutex);

    if (m_condition.wait_for(lock, timeout, [&] { return !m_rosBuffer.empty(); }))
    {
        assert(!m_rosBuffer.empty());

        packet = m_rosBuffer.front();
        m_rosBuffer.pop_front();
    }

    return packet;
}

void MtwCallback::deleteOldestPacket()
{
    XsMutexLocker lock(m_mutex);
    m_packetBuffer.pop_front();
}

int MtwCallback::getMtwIndex() const
{
    return m_mtwIndex;
}

const XsDevice& MtwCallback::device() const
{
    assert(m_device != 0);
    return *m_device;
}

void MtwCallback::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
    //std::unique_lock<std::mutex> lock(m_stdmutex);
    //ros::Time now = ros::Time::now();

    XsMutexLocker lock(m_mutex);
    // NOTE: Processing of packets should not be done in this thread.

    m_packetBuffer.push_back(*packet);
    if (m_packetBuffer.size() > 300)
    {
        std::cout << std::endl;
        deleteOldestPacket();
    }
    /*
       assert(packet != 0);

    // Discard oldest packet if buffer full
    if (m_rosBuffer.size() == m_maxBufferSize)
    {
    m_rosBuffer.pop_front();
    }

    // Push new packet
    m_rosBuffer.push_back(RosXsDataPacket(now, *packet));

    // Manual unlocking is done before notifying, to avoid waking up
    // the waiting thread only to block again
    lock.unlock();
    m_condition.notify_one();
    */
}
