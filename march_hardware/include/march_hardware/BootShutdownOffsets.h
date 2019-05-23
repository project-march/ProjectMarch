// Copyright 2019 Project March.
#ifndef MARCH_IV_BOOTSHUTDOWNOFFSETS_H
#define MARCH_IV_BOOTSHUTDOWNOFFSETS_H

#include <ros/ros.h>

class BootShutdownOffsets
{
  int masterOk;
  int shutdown;
  int shutdownAllowed;

public:
  BootShutdownOffsets(int masterOkByteOffset, int shutdownByteOffset, int shutdownAllowedByteOffset)
    : masterOk(masterOkByteOffset), shutdown(shutdownByteOffset), shutdownAllowed(shutdownAllowedByteOffset)
  {
    if (masterOkByteOffset < 0 || shutdownByteOffset < 0 || shutdownAllowedByteOffset < 0)
    {
      ROS_ERROR("Negative byte offset not possible");
      throw std::runtime_error("Negative byte offset not possible");
    }
  }

  BootShutdownOffsets()
  {
    masterOk = -1;
    shutdown = -1;
    shutdownAllowed = -1;
  }

  int getMasterOkByteOffset() const
  {
    if (masterOk == -1)
    {
      ROS_FATAL("masterOkOffset is -1");
      throw std::runtime_error("masterOkOffset is -1");
    }
    return masterOk;
  }

  int getShutdownByteOffset() const
  {
    if (shutdown == -1)
    {
      ROS_FATAL("shutdownOffset is -1");
      throw std::runtime_error("shutdownOffset is -1");
    }
    return shutdown;
  }

  int getShutdownAllowedByteOffset() const
  {
    if (shutdownAllowed == -1)
    {
      ROS_FATAL("shutdownAllowedOffset is -1");
      throw std::runtime_error("shutdownAllowedOffset is -1");
    }
    return shutdownAllowed;
  }

  /** @brief Override comparison operator */
  friend bool operator==(const BootShutdownOffsets& lhs, const BootShutdownOffsets& rhs)
  {
    return lhs.masterOk == rhs.masterOk && lhs.shutdown == rhs.shutdown && lhs.shutdownAllowed == rhs.shutdownAllowed;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const BootShutdownOffsets& bootShutdownOffsets)
  {
    return os << "BootShutdownOffset(masterOk: " << bootShutdownOffsets.masterOk << ", "
              << "shutdown: " << bootShutdownOffsets.shutdown << ", "
              << "shutdownAllowed: " << bootShutdownOffsets.shutdownAllowed << ")";
  }
};

#endif  // MARCH_IV_BOOTSHUTDOWNOFFSETS_H