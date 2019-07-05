// Copyright 2019 Project March.

#ifndef MARCH_WS_IMOTIONCUBESTATE_H
#define MARCH_WS_IMOTIONCUBESTATE_H

namespace march4cpp
{
struct IMotionCubeState
{
public:
  IMotionCubeState() = default;

  std::string statusWord;
  std::string detailedError;
  std::string motionError;
  std::string state;
  std::string detailedErrorDescription;
  std::string motionErrorDescription;
};

#endif  // MARCH_WS_IMOTIONCUBESTATE_H
