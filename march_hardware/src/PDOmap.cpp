#include <march_hardware/PDOmap.h>

namespace march4cpp
{

PDOmap::PDOmap(){
    this->initAllObjects();
}

void PDOmap::addObject(std::string objectname){
    if (this->allObjects.count(objectname) != 1) {
        ROS_WARN("IMC object %s does not exist (yet), or multiple exist", objectname.c_str());
    }
    else {
        this->PDOObjects[objectname] = this->allObjects[objectname];
    }
}

void PDOmap::mapMISO(int slaveIndex){
    // Sort PDOObjects map
    this->sortPDOObjects();
    // Place into PDO registers of IMC
    int reg = 0x1A00;
    int sizeleft = 64;
    int counter = 0;
    // sdo_write(slaveIndex, reg, 0, 0);
    ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex 0, value 0x0", slaveIndex, reg);
    while(this->sortedPDOObjects.size() > 0){
        // Get next instruction (from end, because sorted from small to large)
        std::pair<std::string, IMCObject> instruction = this->sortedPDOObjects.back();
        this->sortedPDOObjects.pop_back();
        if (sizeleft <= 0){
            // Go to the next register
            // sdo_write(slaveIndex, reg, 0, counter);
            ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex 0, value 0x%X", slaveIndex, reg, counter);
            reg++;
            counter = 0;
            sizeleft = 64;
            // sdo_write(slaveIndex, reg, 0, 0);
            ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex 0, value 0x0", slaveIndex, reg);
        }
        counter++;
        // sdo_write(slaveIndex, reg, counter, this->combineAddressLength(instruction.second.address, instruction.second.length));
        ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slaveIndex, reg, counter, this->combineAddressLength(instruction.second.address, instruction.second.length));
        sizeleft -= instruction.second.length;
    }

}

void PDOmap::sortPDOObjects(){
    // Sort from small to large

    //@TODO(Martijn) extract and make const globally.
    int sizes[] = {8, 16, 32};
    int totalbits = 0;
    for(int i = 0; i < (sizeof(sizes)/sizeof(sizes[0])); i++){
        std::map<std::string, IMCObject>::iterator j;
        for (j = this->PDOObjects.begin(); j != this->PDOObjects.end(); j++){
            if (j->second.length == sizes[i]){
                std::pair<std::string, IMCObject> nextObject;
                nextObject.first = j->first;
                nextObject.second = j->second;
                this->sortedPDOObjects.push_back(nextObject);
                totalbits += sizes[i];
            }
        }
    }
    if(totalbits > this->nrofRegs*this->bitsPerReg){
        ROS_ERROR("Too many objects in PDO Map (total bits %d, only %d allowed",
                totalbits, this->nrofRegs*this->bitsPerReg);
    }
}

uint32_t PDOmap::combineAddressLength(uint16_t address, uint16_t length){
    uint32_t MSword = ((address & 0xFFFF) << 16);
    uint32_t LSword = (length & 0xFFFF);
    return (MSword | LSword);
}

void PDOmap::initAllObjects(){
    // Object(address, length);
    this->allObjects["StatusWord"] = IMCObject(0x6041, 16);
    this->allObjects["ActualPosition"] = IMCObject(0x6064, 32);
    this->allObjects["MotionErrorRegister"] = IMCObject(0x2000, 16);
    this->allObjects["DetailedErrorRegister"] = IMCObject(0x2002, 16);
    this->allObjects["DCLinkVoltage"] = IMCObject(0x2055, 16);
    // etc...
}

}