#ifndef MARCH_LINEAR_ALGEBRA_UTILITIES_H
#define MARCH_LINEAR_ALGEBRA_UTILITIES_H

#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <vector>

namespace linear_algebra_utilities {
// Calculate a dot product of two vectors
#define EPSILON 0.0001
template <typename T>
T dotProductVector(std::vector<T> vector1, std::vector<T> vector2)
{
    return vector1[0] * vector2[0] + vector1[1] * vector2[1]
        + vector1[2] * vector2[2];
};

// Calculate a dot product of two objects with x, y and z attributes
template <typename T> double dotProductPoint(T point1, T point2)
{
    return point1.x * point2.x + point1.y * point2.y + point1.z * point2.z;
};

template <typename T> double dotProductNormal(T point1, T point2)
{
    return point1.normal_x * point2.normal_x + point1.normal_y * point2.normal_y
        + point1.normal_z * point2.normal_z;
}

// Calculate the distance between two points
template <typename T, typename Q>
double distanceBetweenPoints(T point1, Q point2)
{
    return sqrt((point1.x - point2.x) * (point1.x - point2.x)
        + (point1.y - point2.y) * (point1.y - point2.y)
        + (point1.z - point2.z) * (point1.z - point2.z));
}

// Project a point to a line
template <typename T>
pcl::PointXYZ projectPointToLine(
    T point, const pcl::ModelCoefficients::Ptr& line_coefficients)
{
    // The calculations only with pointXYZ types as pcl does the calculations
    pcl::PointXYZ point_to_project;
    point_to_project.x = point.x;
    point_to_project.y = point.y;
    point_to_project.z = point.z;

    // Interpreted as (x(t), y(t), z(t))^T = ([0], [1], [2])^T * t  + ([3], [4],
    // [5])^T
    pcl::PointXYZ direction;
    direction.x = line_coefficients->values[0];
    direction.y = line_coefficients->values[1];
    direction.z = line_coefficients->values[2];

    pcl::PointXYZ position;
    position.x = line_coefficients->values[3];
    position.y = line_coefficients->values[4];
    position.z = line_coefficients->values[5];

    // Compute the projected point using dot products
    pcl::PointXYZ projected_point;
    projected_point.getArray3fMap()
        = (((point_to_project.getArray3fMap() - position.getArray3fMap())
               * direction.getArray3fMap())
                  .sum()
              / (direction.getArray3fMap() * direction.getArray3fMap()).sum())
            * direction.getArray3fMap()
        + position.getArray3fMap();

    return projected_point;
}

template <typename T>
bool normalizeVector(
    std::Vector<T> input_vector, std::Vector<T> normalized_vector)
{
    pcl::Normal input_normal {};
    pcl::Normal output_normal {};
    input_normal.normal_x = input_vector[0];
    input_normal.normal_y = input_vector[1];
    input_normal.normal_z = input_vector[2];
    return normalizeNormal(input_normal, normalized_normal);
}

bool normalizeNormal(pcl::Normal input_normal, pcl::Normal normalized_normal)
{
    double input_normal_norm = dotProductNormal(input_normal, input_normal);
    if (input_normal_norm < EPSILON) {
        ROS_WARN_STREAM("Norm of normal to normalize is smaller then "
            << EPSILON << " result can be inaccurate");
    }
    normalized_normal.normal_x = input_normal_norm.normal_x / input_normal_norm;
    normalized_normal.normal_y = input_normal_norm.normal_y / input_normal_norm;
    normalized_normal.normal_z = input_normal_norm.normal_z / input_normal_norm;
    double normalized_normal_norm
        = dotProductNormal(normalized_normal, normalized_normal);
    if (abs(normalized_normal_norm - 1) > EPSILON) {
        ROS_WARN_STREAM(
            "Norm of normalized normal is too far from 1. The norm is "
            << normalized_normal_norm << " which is more then " << EPSILON
            << "Away from 1. Normalizing not successful");
        return false
    }
    return true;
}

// Calculate the distance between a point and a line
template <typename T>
double distancePointToLine(
    T point, pcl::ModelCoefficients::Ptr line_coefficients)
{
    pcl::PointXYZ projected_point
        = projectPointToLine<T>(point, line_coefficients);
    return distanceBetweenPoints<T, pcl::PointXYZ>(point, projected_point);
}

// Return true if the z coordinate of point1 is lower then that of point2
inline bool pointIsLower(pcl::PointNormal point1, pcl::PointNormal point2)
{
    return point1.z < point2.z;
}

} // namespace linear_algebra_utilities

#endif // MARCH_LINEAR_ALGEBRA_UTILITIES_H
