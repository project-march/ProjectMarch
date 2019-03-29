// Copyright 2019 Project March.
#ifndef MARCH4CPP__PDOMAP_H
#define MARCH4CPP__PDOMAP_H

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>

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

class PDOmap
{
public:
    // Constructor
    PDOmap();

    void addObject(std::string objectname);
    void mapMISO(int slaveIndex);
    void mapMOSI(int slaveIndex);

private:
    void initAllObjects();
    void sortPDOObjects();
    uint32_t combineAddressLength(uint16_t address, uint16_t length);
    std::map<std::string, IMCObject> PDOObjects;
    std::map<std::string, IMCObject> allObjects;
    std::vector<std::pair<std::string, IMCObject>> sortedPDOObjects;

    const int bitsPerReg = 64;
    const int nrofRegs = 4;

};
}

#endif