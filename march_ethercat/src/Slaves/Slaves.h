//
// Holds list of slaves and can filter slaves
//

#ifndef PROJECT_SLAVES_H
#define PROJECT_SLAVES_H

#include "Slave.h"
#include "IMC.h"
#include "GES.h"
#include "PDB.h"

class Slaves {
private: std::vector<Slave*> slaves;
public:
    // Constructor
    Slaves(std::vector<Slave*> slaveList);
    std::vector<Slave*> getSlaves(std::string type);
    std::vector<IMC*> getIMCs();
    std::vector<GES*> getGESs();
    std::vector<PDB*> getPDBs();


};


#endif //PROJECT_SLAVES_H
