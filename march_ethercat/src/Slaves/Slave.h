//
// Base class header for all slaves
//

#ifndef PROJECT_SLAVE_H
#define PROJECT_SLAVE_H

#include <string>
#include <ros/ros.h>
#include "../ethercat_io.h"

extern "C" {
#include "osal.h"
}

class Slave {

protected:
    // Name, number and type of slave
    std::string name;
    uint16 number;
    std::string type;
public:
    // Constructor and Destructor
    Slave(std::string name, uint16 number);
    ~Slave(){};

    // Getters
    std::string getName();
    int getNumber();
    std::string getType();
};


#endif //PROJECT_SLAVE_H
