// Copyright 2019 Project March.
#ifndef MARCH4CPP__PDOMAP_H
#define MARCH4CPP__PDOMAP_H

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatSDO.h>


namespace march4cpp
{

struct IMCObject{
    int address;    // in IMC memory (see IMC manual)
    int length;     // bits (see IMC manual)

    explicit IMCObject(int _address, int _length){
        this->address = _address;
        this->length = _length;
    }

    IMCObject(){};
};

enum class dataDirection {miso, mosi};

// If a new object is added to this enum, make sure to also add it to PDOmap::initAllObjects()!
enum class IMCObjectName {  StatusWord,
                            ActualPosition,
                            MotionErrorRegister,
                            DetailedErrorRegister,
                            DCLinkVoltage,
                            DriveTemperature,
                            ActualTorque,
                            CurrentLimit,
                            MotorPosition,
                            ControlWord,
                            TargetPosition};

class PDOmap
{
public:
    // Constructor
    PDOmap();

    void addObject(IMCObjectName objectname);
    std::map<IMCObjectName, int> map(int slaveIndex, dataDirection direction);


private:
    void initAllObjects();
    void sortPDOObjects();
    uint32_t combineAddressLength(uint16_t address, uint16_t length);
    std::map<IMCObjectName, IMCObject> PDOObjects;
    std::map<IMCObjectName, IMCObject> allObjects;
    std::vector<std::pair<IMCObjectName, IMCObject>> sortedPDOObjects;
    std::map<IMCObjectName, int> byteOffsets;

    const int bitsPerReg = 64;
    const int nrofRegs = 4;
    const int objectSizes[3] = {8, 16, 32};

};
}

#endif