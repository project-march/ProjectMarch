#ifndef PROJECT_ANGLECONVERSIONS_H
#define PROJECT_ANGLECONVERSIONS_H

#include <cmath>

float DegtoRad(float deg)
{
    return static_cast<float>(deg * M_PI / 180);
}

float RadtoDeg(float rad)
{
    return static_cast<float>(rad * 180 / M_PI);
}

#endif //PROJECT_ANGLECONVERSIONS_H
