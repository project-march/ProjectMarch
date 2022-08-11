#ifndef MARCH_LINEAR_ALGEBRA_UTILITIES_H
#define MARCH_LINEAR_ALGEBRA_UTILITIES_H

#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <vector>

namespace linear_algebra_utilities {
// Calculate a dot product of two vectors
template <typename T> T dotProductVector(std::vector<T> vector1, std::vector<T> vector2);

// Calculate a dot product of two objects with x, y and z attributes
template <typename T> double dotProductPoint(T point1, T point2);

template <typename T> double dotProductNormal(T point1, T point2);

template <typename T> double normNormal(T point);

// Calculate the distance between two points
template <typename T, typename Q> double distanceBetweenPoints(T point1, Q point2);

// Project a point to a line
template <typename T> pcl::PointXYZ projectPointToLine(T point, const pcl::ModelCoefficients::Ptr& line_coefficients);

bool normalizeNormal(const pcl::Normal& input_normal, pcl::Normal& normalized_normal);

bool normalizeNormal(pcl::Normal& input_normal);

template <typename T> bool normalize3DVector(const std::vector<T>& input_vector, std::vector<T>& normalized_vector);

template <typename T> bool normalize3DVector(std::vector<T>& input_vector);

// Calculate the distance between a point and a line
template <typename T> double distancePointToLine(T point, pcl::ModelCoefficients::Ptr line_coefficients);

// Return true if the z coordinate of point1 is lower then that of point2
bool pointIsLower(pcl::PointNormal point1, pcl::PointNormal point2);

// Implement the templated functions in the header

// Calculate a dot product of two vectors
template <typename T> T dotProductVector(std::vector<T> vector1, std::vector<T> vector2)
{
    return vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
};

// Calculate a dot product of two objects with x, y and z attributes
template <typename T> double dotProductPoint(T point1, T point2)
{
    return point1.x * point2.x + point1.y * point2.y + point1.z * point2.z;
};

template <typename T> double dotProductNormal(T point1, T point2)
{
    return point1.normal_x * point2.normal_x + point1.normal_y * point2.normal_y + point1.normal_z * point2.normal_z;
}

template <typename T> double normNormal(T point)
{
    return sqrt(dotProductNormal(point, point));
}

// Calculate the distance between two points
template <typename T, typename Q> double distanceBetweenPoints(T point1, Q point2)
{
    return sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y)
        + (point1.z - point2.z) * (point1.z - point2.z));
}

// Project a point to a line
template <typename T> pcl::PointXYZ projectPointToLine(T point, const pcl::ModelCoefficients::Ptr& line_coefficients)
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
        = (((point_to_project.getArray3fMap() - position.getArray3fMap()) * direction.getArray3fMap()).sum()
              / (direction.getArray3fMap() * direction.getArray3fMap()).sum())
            * direction.getArray3fMap()
        + position.getArray3fMap();

    return projected_point;
}

template <typename T> bool normalize3DVector(const std::vector<T>& input_vector, std::vector<T>& normalized_vector)
{
    if (input_vector.size() != 3) {
        ROS_WARN_STREAM("The length of the input vector is not 3. Unable to "
                        "normalize vector.");
        return false;
    }
    pcl::Normal input_normal {};
    input_normal.normal_x = input_vector[0];
    input_normal.normal_y = input_vector[1];
    input_normal.normal_z = input_vector[2];

    pcl::Normal normalized_normal {};
    if (!normalizeNormal(input_normal, normalized_normal)) {
        return false;
    }

    normalized_vector[0] = normalized_normal.normal_x;
    normalized_vector[1] = normalized_normal.normal_y;
    normalized_vector[2] = normalized_normal.normal_z;
    return true;
}

template <typename T> bool normalize3DVector(std::vector<T>& input_vector)
{
    return normalize3DVector<T>(input_vector, input_vector);
}

// Calculate the distance between a point and a line
template <typename T> double distancePointToLine(T point, pcl::ModelCoefficients::Ptr line_coefficients)
{
    pcl::PointXYZ projected_point = projectPointToLine<T>(point, line_coefficients);
    return distanceBetweenPoints<T, pcl::PointXYZ>(point, projected_point);
}

} // namespace linear_algebra_utilities

#endif // MARCH_LINEAR_ALGEBRA_UTILITIES_H
