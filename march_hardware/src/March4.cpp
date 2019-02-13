#include "ros/ros.h"
#include "march_hardware/TemperatureSensor.h"
#include <stdexcept>
#include <march_hardware/March4.h>
#include <march_hardware/Joint.h>
#include <march_hardware/EtherCAT/EthercatMaster.h>

MARCH4::MARCH4() {


    TemperatureSensor *tempSens = new TemperatureSensor(1, 2);
    IMotionCube *iMotionCube= new IMotionCube(3);

    Joint temp_imc = Joint("TEMP IMC", tempSens, iMotionCube);
    Joint temp = Joint("TEMP", tempSens);
    Joint imc = Joint("IMC", NULL, iMotionCube);
    Joint none = Joint("NONE", NULL, NULL);

    jointList.push_back(temp_imc);
    jointList.push_back(temp);
    jointList.push_back(imc);
    jointList.push_back(none);

//    TODO(Isha, Martijn) Initialize ethercat (create seperate thread).

    EthercatMaster ethercatMaster = EthercatMaster(jointList);

}

MARCH4::~MARCH4() {

}

Joint MARCH4::getJoint(std::string jointName) {

    for (int i = 0; i < jointList.size(); i++) {
        if (jointList.at(i).getName() == jointName) {
            return jointList.at(i);
        }
    }

    throw std::runtime_error("Could not find joint with name " + jointName);
}


