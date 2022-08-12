#include <utilities/linear_algebra_utilities.h>

namespace linear_algebra_utilities {
#define EPSILON 0.0001

bool normalizeNormal(const pcl::Normal& input_normal, pcl::Normal& normalized_normal)
{
    double input_normal_norm = normNormal(input_normal);
    if (input_normal_norm < EPSILON) {
        ROS_WARN_STREAM("Norm of normal to normalize is smaller then " << EPSILON << " result can be inaccurate");
    }
    normalized_normal.normal_x = input_normal.normal_x / float(input_normal_norm);
    normalized_normal.normal_y = input_normal.normal_y / float(input_normal_norm);
    normalized_normal.normal_z = input_normal.normal_z / float(input_normal_norm);
    double normalized_normal_norm = normNormal(normalized_normal);
    if (fabs(normalized_normal_norm - 1) > EPSILON) {
        ROS_WARN_STREAM("Norm of normalized normal is too far from 1. The norm is "
            << normalized_normal_norm << " which is more then " << EPSILON
            << "Away from 1. Normalizing not successful.");
        return false;
    }
    return true;
}

bool normalizeNormal(pcl::Normal& input_normal)
{
    return normalizeNormal(input_normal, input_normal);
}

// Return true if the z coordinate of point1 is lower then that of point2
bool pointIsLower(pcl::PointNormal point1, pcl::PointNormal point2)
{
    return point1.z < point2.z;
}

} // namespace linear_algebra_utilities
