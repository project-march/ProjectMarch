// Copyright 2019 Project March
#include "march_imu_manager/mtw.h"

Mtw::Mtw(XsDevice* device, size_t max_buffer_size) : device_(device), max_buffer_size_(max_buffer_size)
{
  this->device_->addCallbackHandler(this);
  configure();
}

void Mtw::configure()
{
  XsOutputMode outputMode = XOM_Calibrated & XOM_Orientation;
  XsOutputSettings outputSettings =
      XOS_Timestamp_PacketCounter & XOS_OrientationMode_Matrix & XOS_CalibratedMode_AccGyrOnly;

  this->device_->setOutputMode(outputMode);
  this->device_->setOutputSettings(outputSettings);
}

bool Mtw::dataAvailable()
{
  std::unique_lock<std::mutex> lck(this->mutex_);
  return !this->packet_buffer_.empty();
}

const XsDataPacket* Mtw::getOldestPacket()
{
  std::unique_lock<std::mutex> lck(this->mutex_);
  XsDataPacket const* packet = &this->packet_buffer_.front();
  return packet;
}

void Mtw::deleteOldestPacket()
{
  std::unique_lock<std::mutex> lck(this->mutex_);
  this->packet_buffer_.pop_front();
}

const XsDeviceId Mtw::getId() const
{
  assert(this->device_ != 0);
  return this->device_->deviceId();
}

void Mtw::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
  std::unique_lock<std::mutex> lck(this->mutex_);

  // NOTE: Processing of packets should not be done in this thread.
  this->packet_buffer_.push_back(*packet);
  if (this->packet_buffer_.size() > this->max_buffer_size_)
  {
    ROS_WARN("Packet buffer is full, deleting oldest packet");
    this->packet_buffer_.pop_front();
  }
}
