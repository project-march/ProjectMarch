#ifndef MARCH_LINEAR_ALGEBRA_UTILITIES_H
#define MARCH_LINEAR_ALGEBRA_UTILITIES_H

namespace linear_algebra_utilities
{
  // Calculate a dot product of two vectors
  template<typename T>
  T dotProductVector(std::vector<T> vector1, std::vector<T> vector2)
  {
    return vector1[0] * vector2[0] +
           vector1[1] * vector2[1] +
           vector1[2] * vector2[2];
  }

  // Calculate a dot product of two objects with x, y and z attributes
  template<typename T>
  double dotProductPoint(T point1, T point2)
  {
    return point1.x * point2.x +
           point1.y * point2.y +
           point1.z * point2.z;
  }

  // Calculate the distance between two points
  template<typename T, typename Q>
  double distanceBetweenPoints(T point1, Q point2)
  {
    return (point1.x - point2.x) *
           (point1.x - point2.x) +
           (point1.y - point2.y) *
           (point1.y - point2.y) +
           (point1.z - point2.z) *
           (point1.z - point2.z);
  }

  // Project a point to a line
  template<typename T>
  T projectPointToLine(T point, pcl::ModelCoefficients::Ptr line_coefficients)
  {
    // Line coefficients are stored as y = [0] * x + [1]
    pcl::PointXYZ direction = line_coefficients->values[0];
    pcl::PointXYZ position = line_coefficients->values[1];
    // Compute the projected point using
    T projected_point;
    projected_point.getArray3fMap() =
        (point.getArray3fMap() - position.getArray3fMap()) * direction.getArray3fMap()
        / (point.getArray3fMap() - position.getArray3fMap()) * (point.getArray3fMap() - position.getArray3fMap())
        * direction.getArray3fMap() + position.getArray3fMap();

    return projected_point;
  }

  // Calculate the distance between a point and a line
  template<typename T>
  double distancePointToLine(T point, pcl::ModelCoefficients::Ptr line_coefficients)
  {
    T projected_point = projectPointToLine<T>(point, line_coefficients);
    return distanceBetweenPoints<T, T>(point, projected_point);
  }
}

#endif //MARCH_LINEAR_ALGEBRA_UTILITIES_H
