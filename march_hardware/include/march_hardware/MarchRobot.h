// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MARCHROBOT_H
#define MARCH_HARDWARE_MARCHROBOT_H

#include <string>
#include <vector>
#include <stdint.h>

#include <march_hardware/Joint.h>

#include <march_hardware/EtherCAT/EthercatMaster.h>
#include <march_hardware/PowerDistributionBoard.h>

namespace march
{
class MarchRobot
{
private:
  EthercatMaster ethercatMaster;
  PowerDistributionBoard powerDistributionBoard;
  ::std::vector<Joint> jointList;

public:
  MarchRobot(::std::vector<Joint> jointList, ::std::string ifName, int ecatCycleTime);

  MarchRobot(::std::vector<Joint> jointList, PowerDistributionBoard powerDistributionBoard, ::std::string ifName,
             int ecatCycleTime);

  ~MarchRobot();

  MarchRobot(const MarchRobot&) = delete;
  MarchRobot& operator=(const MarchRobot&) = delete;

  MarchRobot(MarchRobot&&) = default;

  void startEtherCAT();

  void stopEtherCAT();

  int getMaxSlaveIndex();

  bool hasValidSlaves();

  bool isEthercatOperational();

  Joint getJoint(::std::string jointName);

  PowerDistributionBoard& getPowerDistributionBoard();
  const PowerDistributionBoard& getPowerDistributionBoard() const;

  /** @brief Override comparison operator */
  friend bool operator==(const MarchRobot& lhs, const MarchRobot& rhs)
  {
    if (lhs.jointList.size() != rhs.jointList.size())
    {
      return false;
    }
    for (unsigned int i = 0; i < lhs.jointList.size(); i++)
    {
      const march::Joint lhsJoint = lhs.jointList.at(i);
      const march::Joint rhsJoint = rhs.jointList.at(i);
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
}  // namespace march
#endif  // MARCH_HARDWARE_MARCHROBOT_H
