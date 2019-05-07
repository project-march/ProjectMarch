// Copyright 2019 Project March.
#ifndef MARCH_IV_STATEOFFSETS_H
#define MARCH_IV_STATEOFFSETS_H

class StateOffsets
{
  int masterOkByteOffset;
  int shutdownByteOffset;
  int shutdownAllowedByteOffset;

public:
  StateOffsets(int masterOkByteOffset, int shutdownByteOffset, int shutdownAllowedByteOffset)
    : masterOkByteOffset(masterOkByteOffset)
    , shutdownByteOffset(shutdownByteOffset)
    , shutdownAllowedByteOffset(shutdownAllowedByteOffset)
  {
  }

  StateOffsets()
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

#endif  // MARCH_IV_STATEOFFSETS_H