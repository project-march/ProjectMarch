// Copyright 2019 Project March
#include "march_imu_manager/Mtw.h"

Mtw::Mtw(XsDevice* device, size_t maxBufferSize) :
    m_device(device),
    m_maxBufferSize(maxBufferSize)
{
    this->m_device->addCallbackHandler(this);
    configure();
}

void Mtw::configure()
{
    XsOutputMode outputMode = XOM_Calibrated & XOM_Orientation;
    XsOutputSettings outputSettings =
        XOS_Timestamp_PacketCounter &
        XOS_OrientationMode_Matrix &
        XOS_CalibratedMode_AccGyrOnly;

    this->m_device->setOutputMode(outputMode);
    this->m_device->setOutputSettings(outputSettings);
}

bool Mtw::dataAvailable()
{
    std::unique_lock<std::mutex> lck(this->m_mutex);
    return !this->m_packetBuffer.empty();
}

const XsDataPacket* Mtw::getOldestPacket()
{
    std::unique_lock<std::mutex> lck(this->m_mutex);
    XsDataPacket const * packet = &this->m_packetBuffer.front();
    return packet;
}

void Mtw::deleteOldestPacket()
{
    std::unique_lock<std::mutex> lck(this->m_mutex);
    this->m_packetBuffer.pop_front();
}

const XsDeviceId Mtw::getId() const
{
    assert(this->m_device != 0);
    return this->m_device->deviceId();
}

void Mtw::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
    std::unique_lock<std::mutex> lck(this->m_mutex);

    // NOTE: Processing of packets should not be done in this thread.
    this->m_packetBuffer.push_back(*packet);
    if (this->m_packetBuffer.size() > this->m_maxBufferSize)
    {
        ROS_WARN("Packet buffer is full, deleting oldest packet");
        this->m_packetBuffer.pop_front();
    }
}
