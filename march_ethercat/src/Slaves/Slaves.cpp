//
// Holds list of slaves and can filter slaves
//

#include "Slaves.h"

Slaves::Slaves(std::vector<Slave*> slaveList)
{
  slaves = slaveList;
}

// Returns a list of all IMCs
std::vector<IMC*> Slaves::getIMCs()
{
    std::vector<Slave*> slaveList = getSlaves("IMC");
    std::vector<IMC*> IMCList(slaveList.begin(), slaveList.end());
    return IMCList;
}

// Returns a list of all GESs
std::vector<GES*> Slaves::getGESs()
{
    std::vector<Slave*> slaveList = getSlaves("GES");
    std::vector<GES*> GESList(slaveList.begin(), slaveList.end());
    return GESList;
}

// Returns a list of all PDBs
std::vector<PDB*> Slaves::getPDBs()
{
    std::vector<Slave*> slaveList = getSlaves("PDB");
    std::vector<PDB*> PDBList(slaveList.begin(), slaveList.end());
    return PDBList;
}

// Returns a list of all slaves (base class) of a specific type
std::vector<Slave*> Slaves::getSlaves(std::string type)
{
  std::vector<Slave*> returnList;
  for (int i = 0; i < slaves.size(); i++)
  {
    if (slaves[i]->getType() == type)
    {
      returnList.push_back(slaves[i]);
    }
  }
  return returnList;
}