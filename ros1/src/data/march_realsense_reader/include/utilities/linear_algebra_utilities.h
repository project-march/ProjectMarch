#ifndef MARCH_LINEAR_ALGEBRA_UTILITIES_H
#define MARCH_LINEAR_ALGEBRA_UTILITIES_H

#include <vector>

namespace linear_algebra_utilities
{
  template<typename T>
  T dotProductVector(std::vector<T> vector1, std::vector<T> vector2);

  template<typename T>
  double dotProductPoint(T point1, T point2);
}

#endif //MARCH_LINEAR_ALGEBRA_UTILITIES_H
