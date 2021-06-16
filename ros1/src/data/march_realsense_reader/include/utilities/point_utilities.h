//
// Created by ttveen on 14-06-21.
//
#include <pcl/features/normal_3d.h>

#ifndef MARCH_POINT_UTILITIES_H
#define MARCH_POINT_UTILITIES_H

namespace point_utilities {
pcl::PointXYZ makePoint(const float x, const float y, const float z)
{
    pcl::PointXYZ point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}
} // namespace point_utilities

#endif // MARCH_POINT_UTILITIES_H
