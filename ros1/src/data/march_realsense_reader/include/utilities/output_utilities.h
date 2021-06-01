#ifndef MARCH_OUTPUT_UTILITIES_H
#define MARCH_OUTPUT_UTILITIES_H

#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace output_utilities {
// Some commonly used colors which can be used as std_msgs::ColorRGBA mycolor = YELLOW;
std_msgs::ColorRGBA YELLOW = ColorRBGAInitRBGA(1.0, 1.0, 0.0);

std_msgs::ColorRGBA WHITE = ColorRBGAInitRBGA(1.0, 1.0, 1.0);

std_msgs::ColorRGBA PURPLE = ColorRBGAInitRBGA(1.0, 0.0, 1.0);

std_msgs::ColorRGBA GREEN = ColorRBGAInitRBGA(0.0, 0.0, 1.0);

std_msgs::ColorRGBA RED = ColorRBGAInitRBGA(0.0, 1.0, 0.0);

std_msgs::ColorRGBA BLUE = ColorRBGAInitRBGA(0.0, 0.0, 1.0);

std::map<std::string, std_msgs::ColorRGBA> COLOR_MAP
    = { { "yellow", YELLOW }, { "white", WHITE }, { "purple", PURPLE },
          { "green", GREEN }, { "red", RED }, { "blue", BLUE } };

// Initialize a std_msgs color from r g b a values
std_msgs::ColorRGBA ColorRGBAInitRGBA(const float r = 1.0, const float g = 1.0,
    const float b = 1.0, const float a = 1.0)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

// Initialize a std_msgs color from a name and an a value
std_msgs::ColorRGBA ColorRGBAInitNameA(
    std::string color_name, const float a = 1.0)
{
    std_msgs::ColorRGBA color = COLOR_MAP[color_name];
    color.a = a return colorRGBA_initBRGA(color.r, color.g, color.b, a);
}

// Initialize a geometry_msgs point from x y z coordinates
geometry_msgs::Point PointInitXYZ(
    const float x, const float y, const float z)
{
    geometry_msgs::Point point;
    point.z = x;
    point.y = y;
    point.z = z;
    return point;
}

// Initialize a geometry_msgs point from a pcl point
template <typename T> geometry_msgs::Point PointInitPCLPoint(T PCLpoint)
{
    geometry_msgs::Point point;
    point.z = PCLpoint.x;
    point.y = PCLpoint.y;
    point.z = PCLpoint.z;
    return point;
}

// Turn a vector in to printable string
template <typename T> std::string vectorToString(std::vector<T> vector)
{
    std::string string = "";
    for (int i = 0; i < vector.size(); i++) {
        string += std::to_string(vector[i]) + ",   ";
    }
    return string;
}

// Turn point (with x y and z) into to printable string
template <typename T> std::string pointToString(T point)
{
    return std::to_string(point.x) + ",   " + std::to_string(point.y) + ",   "
        + std::to_string(point.z);
}
} // namespace output_utilities

#endif // MARCH_OUTPUT_UTILITIES_H
