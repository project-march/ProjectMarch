#ifndef MARCH_REALSENSE_READER_COLOR_UTILITIES_H
#define MARCH_REALSENSE_READER_COLOR_UTILITIES_H

#include <map>
#include <visualization_msgs/MarkerArray.h>

namespace color_utilities {
// Initialize a std_msgs color from r g b a values
std_msgs::ColorRGBA colorRGBAInitRGBA(
    const double r, const double g, const double b, const double a = 1.0)
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
std_msgs::ColorRGBA YELLOW = colorRGBAInitRGBA(/*r=*/1.0, /*g=*/1.0, /*b=*/0.0);

std_msgs::ColorRGBA WHITE = colorRGBAInitRGBA(/*r=*/1.0, /*g=*/1.0, /*b=*/1.0);

std_msgs::ColorRGBA PURPLE = colorRGBAInitRGBA(/*r=*/1.0, /*g=*/0.0, /*b=*/1.0);

std_msgs::ColorRGBA GREEN = colorRGBAInitRGBA(/*r=*/0.0, /*g=*/1.0, /*b=*/0.0);

std_msgs::ColorRGBA RED = colorRGBAInitRGBA(/*r=*/1.0, /*g=*/0.0, /*b=*/0.0);

std_msgs::ColorRGBA BLUE = colorRGBAInitRGBA(/*r=*/0.0, /*g=*/0.0, /*b=*/1.0);

std::map<std::string, std_msgs::ColorRGBA> COLOR_MAP
    = { { /*x=*/"yellow", YELLOW }, { /*x=*/"white", WHITE },
          { /*x=*/"purple", PURPLE }, { /*x=*/"green", GREEN },
          { /*x=*/"red", RED }, { /*x=*/"blue", BLUE } };

// Initialize a std_msgs color from a name and an a value
std_msgs::ColorRGBA colorRGBAInitNameA(
    const std::string& color_name, const double a = 1.0)
{
    std_msgs::ColorRGBA color = COLOR_MAP[color_name];
    return colorRGBAInitRGBA(color.r, color.g, color.b, a);
}
} // namespace color_utilities

#endif // MARCH_REALSENSE_READER_COLOR_UTILITIES_H
