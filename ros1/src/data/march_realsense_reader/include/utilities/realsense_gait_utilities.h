#ifndef MARCH_REALSENSE_GAIT_UTILITIES_H
#define MARCH_REALSENSE_GAIT_UTILITIES_H

/** This enum is used for specifying the obstacle that should be dynamically
 * made with the RealSense camera, made to match the
 *  GetGaitParametersRealSense.srv
 */
enum SelectedGait {
    stairs_up = 0,
    stairs_down = 1,
    ramp_up = 2,
    ramp_down = 3
};

#endif // MARCH_REALSENSE_GAIT_UTILITIES_H
