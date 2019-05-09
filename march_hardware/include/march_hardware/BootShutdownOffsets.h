// Copyright 2019 Project March.
#ifndef MARCH_IV_BOOTSHUTDOWNOFFSETS_H
#define MARCH_IV_BOOTSHUTDOWNOFFSETS_H

class BootShutdownOffsets
{
    //TODO(TIM) remove Offset part!
  int masterOkByteOffset;
  int shutdownByteOffset;
  int shutdownAllowedByteOffset;

public:
  BootShutdownOffsets(int masterOkByteOffset, int shutdownByteOffset, int shutdownAllowedByteOffset)
    : masterOkByteOffset(masterOkByteOffset)
    , shutdownByteOffset(shutdownByteOffset)
    , shutdownAllowedByteOffset(shutdownAllowedByteOffset)
  {
  }

  BootShutdownOffsets()
  {
    masterOkByteOffset = -1;
    shutdownByteOffset = -1;
    shutdownAllowedByteOffset = -1;
  }

  int getMasterOkByteOffset() const
  {
    return masterOkByteOffset;
  }

  int getShutdownByteOffset() const
  {
    return shutdownByteOffset;
  }

  int getShutdownAllowedByteOffset() const
  {
    return shutdownAllowedByteOffset;
  }
};

#endif  //MARCH_IV_BOOTSHUTDOWNOFFSETS_H