#include <march_hardware/PDOmap.h>

namespace march4cpp
{

PDOmap::PDOmap(){
    this->initAllObjects();
}

void PDOmap::addObject(std::string objectname){
    if (this->allObjects.count(objectname) != 1) {
        // ROS_WARN("IMC object %s does not exist (yet), or multiple exist", objectname);
    } else {
        this->PDOObjects[objectname] = this->allObjects[objectname];
    }
}

void PDOmap::mapMISO(int slaveIndex){
    // Sort PDOObjects map
    this->sortPDOObjects();
    // Place into PDO registers of IMC
    int reg = 0x1A00;
    int sizeleft = 64;
    int counter = 1;
    while(this->sortedPDOObjects.size() > 0){
        // Get next instruction (from end, because sorted from small to large)
        std::pair<std::string, IMCObject> instruction = this->sortedPDOObjects.pop_back();
        if (sizeleft <= 0){
            // Go to the next register
            // sdo_write(slaveIndex, reg, 0, counter);
            // ROS_DEBUG("sdo write: slaveIndex %i, reg %i, counter 0, value %i", slaveIndex, reg, counter);
            reg++;
            counter = 1;
            sizeleft = 64;
            // sdo_write(slaveIndex, reg, 0, 0);
            // ROS_DEBUG("sdo write: slaveIndex %i, reg %i, counter 0, value 0", slaveIndex, reg);
        }
        // sdo_write(slaveIndex, reg, counter, this->combineAddressLength(instruction.second.address, instruction.second.length));
        // ROS_DEBUG("sdo write: slaveIndex %i, reg %i, counter %i, value %i", slaveIndex, reg, counter, this->combineAddressLength(instruction.second.address, instruction.second.length));
        sizeleft -= instruction.second.length;
        counter++;
    }

}

void PDOmap::sortPDOObjects(){
    // Sort from small to large
    int sizes[] = {8, 16, 32};
    int totalbits = 0;
    for(int i = 0; i < (sizeof(sizes)/sizeof(sizes[0])); i++){
        std::map<std::string, IMCObject>::iterator j;
        for (j = this->PDOObjects.begin(); j != this->PDOObjects.end(); j++){
            if (j->second.length == sizes[i]){
                this->sortedPDOObjects.push_back(j->first, j->second);
                totalbits += sizes[i];
            }
        }
    }
    if(totalbits > 4*64){
        // ROS_ERROR("Too many objects in PDO Map (total bits %d, only %d allowed", totalbits, 4*64);
    }
}

int PDOmap::combineAddressLength(int address, int length){
    return (((address << 16) & 0xFFFF) | (length & 0xFFFF));
}

void PDOmap::initAllObjects(){
    // Object(address, length);
    this->allObjects["StatusWord"] = IMCObject(0x6041, 16);
    this->allObjects["ActualPosition"] = IMCObject(0x6064, 32);
    this->allObjects["MotionErrorRegister"] = IMCObject(0x2000, 16);
    // etc...
}

}