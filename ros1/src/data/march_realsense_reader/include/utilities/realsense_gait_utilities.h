#ifndef MARCH_REALSENSE_GAIT_UTILITIES_H
#define MARCH_REALSENSE_GAIT_UTILITIES_H
/** Utility classes for the parameter determiner, these are made to match the
 *  GetGaitParametersRealSense.srv
 */


/** A struct that can be used for any parametric gait
 * see https://confluence.projectmarch.nl:8443/display/62tech/Obstacles+analysis#
 * for more explanation about the range of heights and step sizes
 */
//struct GaitParameters
//{
//    /** Parameter that determines the step size,
//     * between 0 and 1, where 0 is the small step for this obstacle and
//     * 1 is the large step gait for this obstacle.
//     *
//     * Stairs: Min depth is 20 cm, max is ? cm
//     * Ramp: Will not be used for ramp up/down
//     */
//    float step_size_parameter;
//
//    /** Parameter that determines the amount of HAA that should be used for
//     * the chosen step, between 0 and 1, where 0 is regular angle and 1 is
//     * maximal sideways step for this obstacle. Currently, we have not implemented
//     * this in the gaits yet.
//     *
//     * Stairs: Not used yet, should always be 0
//     * Ramp: Not used yet, should always be 0
//     */
//    float side_step_parameter;
//
//    /** Parameter that determines the height of the step to make,
//     * also between 0 and 1, where 0 is the low step for this obstacle and
//     * 1 is the high step gait for this obstacle.
//     *
//     * Stairs: Min height is 12 cm, max height is 22 cm
//     * Ramp: Min gradient is 5 degrees, max is  20 degrees
//     */
//    float step_height_parameter;
//};

/** This enum is used for specifying the obstacle that should be dynamically
 * made with the RealSense camera.
 */

enum SelectedGait{stairs_up, stairs_down, ramp_up, ramp_down};

#endif //MARCH_REALSENSE_GAIT_UTILITIES_H
