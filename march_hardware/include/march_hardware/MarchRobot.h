// Copyright 2019 Project March.

#ifndef MARCH4CPP__MARCH4_H
#define MARCH4CPP__MARCH4_H

#include <string>
#include <vector>
#include <stdint.h>

#include <march_hardware/Joint.h>

#include <march_hardware/EtherCAT/EthercatMaster.h>
#include <march_hardware/PowerDistributionBoard.h>

namespace march4cpp
{
class MarchRobot
{
private:
  std::unique_ptr<EthercatMaster> ethercatMaster;

public:
  ::std::vector<Joint> jointList;
  PowerDistributionBoard powerDistributionBoard;

  MarchRobot(::std::vector<Joint> jointList, ::std::string ifName, int ecatCycleTime);

  MarchRobot(::std::vector<Joint> jointList, PowerDistributionBoard powerDistributionBoard, ::std::string ifName,
             int ecatCycleTime);

  //TODO(TIM) This is needed for the destructor, but why??
  MarchRobot(MarchRobot&&) = default;

  ~MarchRobot();

  void startEtherCAT();

  void stopEtherCAT();

  int getMaxSlaveIndex();

  bool hasValidSlaves();

  bool isEthercatOperational();

  Joint getJoint(::std::string jointName);

  PowerDistributionBoard getPowerDistributionBoard();

  /** @brief Override comparison operator */
  friend bool operator==(const MarchRobot& lhs, const MarchRobot& rhs)
  {
    if (lhs.jointList.size() != rhs.jointList.size())
    {
      return false;
    }
    for (unsigned int i = 0; i < lhs.jointList.size(); i++)
    {
      const march4cpp::Joint lhsJoint = lhs.jointList.at(i);
      const march4cpp::Joint rhsJoint = rhs.jointList.at(i);
      if (lhsJoint != rhsJoint)
      {
        return false;
      }
    }
    return true;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const MarchRobot& marchRobot)
  {
    for (unsigned int i = 0; i < marchRobot.jointList.size(); i++)
    {
      os << marchRobot.jointList.at(i) << "\n";
    }
    return os;
  }
};
}  // namespace march4cpp
#endif
