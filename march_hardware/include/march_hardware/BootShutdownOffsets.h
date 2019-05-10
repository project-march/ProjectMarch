// Copyright 2019 Project March.
#ifndef MARCH_IV_BOOTSHUTDOWNOFFSETS_H
#define MARCH_IV_BOOTSHUTDOWNOFFSETS_H

class BootShutdownOffsets
{
    //TODO(TIM) remove Offset part!
  int masterOk;
  int shutdown;
  int shutdownAllowed;

public:
  BootShutdownOffsets(int masterOkByteOffset, int shutdownByteOffset, int shutdownAllowedByteOffset)
    : masterOk(masterOkByteOffset)
    , shutdown(shutdownByteOffset)
    , shutdownAllowed(shutdownAllowedByteOffset)
  {
  }

  BootShutdownOffsets()
  {
    masterOk = -1;
    shutdown = -1;
    shutdownAllowed = -1;
  }

  int getMasterOkByteOffset() const
  {
    return masterOk;
  }

  int getShutdownByteOffset() const
  {
    return shutdown;
  }

  int getShutdownAllowedByteOffset() const
  {
    return shutdownAllowed;
  }

  /** @brief Override comparison operator */
  friend bool operator==(const BootShutdownOffsets& lhs, const BootShutdownOffsets& rhs)
  {
    return lhs.masterOk == rhs.masterOk && lhs.shutdown == rhs.shutdown &&
        lhs.shutdownAllowed == rhs.shutdownAllowed;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const BootShutdownOffsets& bootShutdownOffsets)
  {
    return os << "BootShutdownOffset(masterOk: " << bootShutdownOffsets.masterOk << ", "
              << "shutdownAllowed: " << bootShutdownOffsets.shutdownAllowed << ", "
              << "shutdown: " << bootShutdownOffsets.shutdown << ")";
  }

};

#endif  //MARCH_IV_BOOTSHUTDOWNOFFSETS_H