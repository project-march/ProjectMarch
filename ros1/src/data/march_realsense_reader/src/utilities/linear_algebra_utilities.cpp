#include "utilities/linear_algebra_utilities.h"


// Calculate a dot product of two vectors
template<typename T>
T linear_algebra_utilities::dotProductVector(std::vector<T> vector1, std::vector<T> vector2)
{
  return vector1[0] * vector2[0] +
         vector1[1] * vector2[1] +
         vector1[2] * vector2[2];
}

// Calculate a dot product of two objects with x, y and z attributes
template<typename T>
double linear_algebra_utilities::dotProductPoint(T point1, T point2)
{
  return point1.z * point2.x +
         point1.y * point2.y +
         point1.z * point2.z;
}

