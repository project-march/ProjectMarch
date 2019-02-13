#ifndef MARCH4CPP__TEMPERATURESENSOR_H
#define MARCH4CPP__TEMPERATURESENSOR_H

#include <stdint.h>

class TemperatureSensor {

private:
    int slaveIndex;
    uint8_t byteOffset;

public:
    TemperatureSensor(int slaveIndex, uint8_t byteOffset);

    float getTemperature();

    void initialize();
};

#endif
