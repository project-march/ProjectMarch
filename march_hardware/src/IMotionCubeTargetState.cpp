// Copyright 2019 Project March.

#include <march_hardware/IMotionCubeTargetState.h>

namespace march4cpp
{
const IMotionCubeTargetState IMotionCubeTargetState::SWITCH_ON_DISABLED =
    IMotionCubeTargetState("Switch on Disabled", 128, 0b0000000001001111, 64);
const IMotionCubeTargetState IMotionCubeTargetState::READY_TO_SWITCH_ON =
    IMotionCubeTargetState("Ready to Switch On", 6, 0b0000000001101111, 33);
const IMotionCubeTargetState IMotionCubeTargetState::SWITCHED_ON =
    IMotionCubeTargetState("Switched On", 7, 0b0000000001101111, 35);
const IMotionCubeTargetState IMotionCubeTargetState::OPERATION_ENABLED =
    IMotionCubeTargetState("Operation Enabled", 15, 0b0000000001101111, 39);
}  // namespace march4cpp
