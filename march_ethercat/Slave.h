//
// Created by Martijn on 5-2-19.
//

#ifndef PROJECT_SLAVE_H
#define PROJECT_SLAVE_H

#include <string>

class Slave {

protected:
    std::string name;
    int number;
public:
    std::string getName();
    int getNumber();
};


#endif //PROJECT_SLAVE_H
