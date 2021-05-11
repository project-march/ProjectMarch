#ifndef MARCH_REALSENSE_CATEGORY_UTILITIES_H
#define MARCH_REALSENSE_CATEGORY_UTILITIES_H

/** This enum is used for specifying the category of gait that should be
 *  dynamically made with the RealSense camera, made to match the
 *  GetGaitParametersRealSense.srv
 */
enum RealSenseCategory {
    stairs_up = 0,
    stairs_down = 1,
    ramp_up = 2,
    ramp_down = 3,
    sit = 4
};

#endif // MARCH_REALSENSE_CATEGORY_UTILITIES_H
