#ifndef MARCH4CPP__JOINT_H
#define MARCH4CPP__JOINT_H

#include <sstream>
#include "march_hardware/TemperatureSensor.h"
#include "march_hardware/IMotionCube.h"

class Joint {
private:
    std::string name;
    TemperatureSensor *temperatureSensor;
    IMotionCube *iMotionCube;

public:

    Joint(std::string name, TemperatureSensor *temperatureSensor = NULL, IMotionCube *iMotionCube = NULL);

    ~Joint() = default;

    void actuate(double effort);

    std::string getName();
    float getAngle();

    float getTemperature();
    void initialize();
};

#endif
