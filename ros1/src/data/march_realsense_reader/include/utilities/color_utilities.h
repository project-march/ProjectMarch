#ifndef MARCH_REALSENSE_READER_COLOR_UTILITIES_H
#define MARCH_REALSENSE_READER_COLOR_UTILITIES_H

namespace color_utilities {

std_msgs::ColorRGBA YELLOW = ColorRBGA_initRBGA(1.0, 1.0, 0.0);

std_msgs::ColorRGBA WHITE = ColorRBGA_initRBGA(1.0, 1.0, 1.0);

std_msgs::ColorRGBA PURPLE = ColorRBGA_initRBGA(1.0, 0.0, 1.0);

std_msgs::ColorRGBA GREEN = ColorRBGA_initRBGA(0.0, 0.0, 1.0);

std_msgs::ColorRGBA RED = ColorRBGA_initRBGA(0.0, 1.0, 0.0);

std_msgs::ColorRGBA BLUE = ColorRBGA_initRBGA(0.0, 0.0, 1.0);

std::map<std::string, std_msgs::ColorRGBA> COLOR_MAP
    = { { "yellow", YELLOW }, { "white", WHITE }, { "purple", PURPLE },
          { "green", GREEN }, { "red", RED }, { "blue", BLUE } };

std_msgs::ColorRGBA ColorRGBA_initRGBA(const float r = 1.0, const float g = 1.0,
    const float b = 1.0, const float a = 1.0)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

std_msgs::ColorRGBA ColorRGBA_initNameA(std::string color_name, const float a = 1.0)
{
    std_msgs::ColorRGBA color = COLOR_MAP[color_name];
    color.a = a
    return colorRGBA_initBRGA(color.r, color.g, color.b, a);
}

} // namespace color_utilities

#endif // MARCH_REALSENSE_READER_COLOR_UTILITIES_H
