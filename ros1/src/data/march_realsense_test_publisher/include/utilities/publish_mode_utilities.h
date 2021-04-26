#ifndef MARCH_PUBLISH_MODE_UTILITIES_H
#define MARCH_PUBLISH_MODE_UTILITIES_H

/** This enum is used for specifying the obstacle that should be dynamically
 * made with the RealSense camera, made to match the
 *  PublishTestDataset.srv
 */
enum SelectedMode { start = 0, next = 1, end = 2, custom = 3, slide_show = 4 };

#endif // MARCH_PUBLISH_MODE_UTILITIES_H
