//
// Created by Martijn on 5-2-19.
//

#ifndef PROJECT_SLAVE_H
#define PROJECT_SLAVE_H

#include <string>
#include "ethercat_io.h"

extern "C" {
#include "osal.h"
}

class Slave {

protected:
    std::string name;
    uint16 number;
public:
    Slave(std::string name, uint16 number);
    ~Slave() {};

    std::string getName();
    int getNumber();
    std::string getType();
    std::string type;
};


#endif //PROJECT_SLAVE_H
