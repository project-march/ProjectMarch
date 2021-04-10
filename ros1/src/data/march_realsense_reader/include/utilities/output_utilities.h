#ifndef MARCH_OUTPUT_UTILITIES_H
#define MARCH_OUTPUT_UTILITIES_H


namespace output_utilities
{
  // Turn a vector in to printable string
  template<typename T>
  std::string vectorToString(std::vector<T> vector)
  {
    std::string string = "";
    for(int i = 0; i < vector.size(); i++) {
      string += std::to_string(vector[i]) + ",   ";
    }
    return string;
  }

  // Turn point (with x y and z) into to printable string
  template<typename T>
  std::string pointToString(T point)
  {
    return std::to_string(point.x) + ",   " +
           std::to_string(point.y) + ",   " +
           std::to_string(point.z);
  }
}

#endif  // MARCH_OUTPUT_UTILITIES_H
