
#include <march_hardware/TemperatureSensor.h>
#include <stdexcept>

#include "march_hardware/TemperatureSensor.h"


TemperatureSensor::TemperatureSensor(int slaveIndex, uint8_t byteOffset) {
    this->slaveIndex = slaveIndex;
    this->byteOffset = byteOffset;
}

float TemperatureSensor::getTemperature() {
//    TODO(Martijn) read actual data from ethercat.
    return 0;
}

void TemperatureSensor::initialize() {
//    TODO(Martijn) initialize PDO/SDO settings here.

}
