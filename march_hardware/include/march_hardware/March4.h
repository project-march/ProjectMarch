#ifndef MARCH4CPP__MARCH4_H
#define MARCH4CPP__MARCH4_H

#include <sstream>
#include <vector>
#include "march_hardware/Joint.h"

class MARCH4 {
private:
public:
    MARCH4();
    ~MARCH4();
    std::vector<Joint> jointList;
    Joint getJoint(std::string jointName);

//    TODO(Isha) check if all slave indices are valid.
    bool isValid();
};

#endif