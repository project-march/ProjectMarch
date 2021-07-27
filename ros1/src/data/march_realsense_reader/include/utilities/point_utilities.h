#include <pcl/point_types.h>

#ifndef MARCH_POINT_UTILITIES_H
#define MARCH_POINT_UTILITIES_H

namespace point_utilities {
/*
 * Returns a pcl::PointXYZ object for x, y , z
 */
pcl::PointXYZ makePointXYZ(const float x, const float y, const float z)
{
    pcl::PointXYZ point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}
} // namespace point_utilities

#endif // MARCH_POINT_UTILITIES_H
