#ifndef MARCH_REALSENSE_READER_DEBUG_UTILITIES_H
#define MARCH_REALSENSE_READER_DEBUG_UTILITIES_H

#include <map>
#include <visualization_msgs/MarkerArray.h>

namespace debug_utilities {
// Initialize a std_msgs color from r g b a values
std_msgs::ColorRGBA colorRGBAInitRGBA(const float r = 1.0, const float g = 1.0,
    const float b = 1.0, const float a = 1.0)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

// Some commonly used colors which can be used as
// std_msgs::ColorRGBA mycolor = YELLOW;
std_msgs::ColorRGBA YELLOW = colorRGBAInitRGBA(1.0, 1.0, 0.0);

std_msgs::ColorRGBA WHITE = colorRGBAInitRGBA(1.0, 1.0, 1.0);

std_msgs::ColorRGBA PURPLE = colorRGBAInitRGBA(1.0, 0.0, 1.0);

std_msgs::ColorRGBA GREEN = colorRGBAInitRGBA(0.0, 0.0, 1.0);

std_msgs::ColorRGBA RED = colorRGBAInitRGBA(0.0, 1.0, 0.0);

std_msgs::ColorRGBA BLUE = colorRGBAInitRGBA(0.0, 0.0, 1.0);

std::map<std::string, std_msgs::ColorRGBA> COLOR_MAP
    = { { "yellow", YELLOW }, { "white", WHITE }, { "purple", PURPLE },
          { "green", GREEN }, { "red", RED }, { "blue", BLUE } };

// Initialize a std_msgs color from a name and an a value
std_msgs::ColorRGBA colorRGBAInitNameA(
    std::string color_name, const float a = 1.0)
{
    std_msgs::ColorRGBA color = COLOR_MAP[color_name];
    return colorRGBAInitRGBA(color.r, color.g, color.b, a);
}

// Initialize a geometry_msgs point from x y z coordinates
geometry_msgs::Point pointInitXYZ(const float x, const float y, const float z)
{
    geometry_msgs::Point point;
    point.z = x;
    point.y = y;
    point.z = z;
    return point;
}

// Initialize a geometry_msgs point from a pcl point
template <typename T> geometry_msgs::Point pointInitPCLPoint(T PCLpoint)
{
    geometry_msgs::Point point;
    point.z = PCLpoint.x;
    point.y = PCLpoint.y;
    point.z = PCLpoint.z;
    return point;
}
} // namespace debug_utilities

#endif // MARCH_REALSENSE_READER_DEBUG_UTILITIES_H
