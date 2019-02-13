//
// Created by projectmarch on 13-2-19.
//

#ifndef PROJECT_IMOTIONCUBE_H
#define PROJECT_IMOTIONCUBE_H


class IMotionCube {
private:
    int slaveIndex;

//    TODO(Martijn) add PDO/SDO settings here.

public:
    IMotionCube(int slaveIndex);
    void initialize();
    float getAngle();
    bool PDOmapping();
    bool StartupSDO(int ecatCycleTime);

};

#endif //PROJECT_IMOTIONCUBE_H
