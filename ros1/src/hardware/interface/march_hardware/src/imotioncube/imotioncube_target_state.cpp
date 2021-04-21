// Copyright 2019 Project March.

#include <march_hardware/imotioncube/imotioncube_target_state.h>

namespace march {
const IMotionCubeTargetState IMotionCubeTargetState::SWITCH_ON_DISABLED
    = IMotionCubeTargetState("Switch on Disabled", /*controlWord=*/128,
        /*stateMask=*/0b0000000001001111, /*state=*/64);
const IMotionCubeTargetState IMotionCubeTargetState::READY_TO_SWITCH_ON
    = IMotionCubeTargetState("Ready to Switch On", /*controlWord=*/6,
        /*stateMask=*/0b0000000001101111, /*state=*/33);
const IMotionCubeTargetState IMotionCubeTargetState::SWITCHED_ON
    = IMotionCubeTargetState("Switched On", /*controlWord=*/7,
        /*stateMask=*/0b0000000001101111, /*state=*/35);
const IMotionCubeTargetState IMotionCubeTargetState::OPERATION_ENABLED
    = IMotionCubeTargetState("Operation Enabled", /*controlWord=*/15,
        /*stateMask=*/0b0000000001101111, /*state=*/39);
} // namespace march
