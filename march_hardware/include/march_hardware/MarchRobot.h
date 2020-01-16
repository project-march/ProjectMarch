// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MARCHROBOT_H
#define MARCH_HARDWARE_MARCHROBOT_H

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
  std::unique_ptr<PowerDistributionBoard> powerDistributionBoard;
  ::std::vector<Joint> jointList;

public:
  MarchRobot(::std::vector<Joint> jointList, ::std::string ifName, int ecatCycleTime);

  MarchRobot(::std::vector<Joint> jointList, PowerDistributionBoard powerDistributionBoard, ::std::string ifName,
             int ecatCycleTime);

  // TODO(TIM) This is needed for the destructor, but why??
  MarchRobot(MarchRobot&&) = default;

  ~MarchRobot();

  void startEtherCAT();

  void stopEtherCAT();

  int getMaxSlaveIndex();

  bool hasValidSlaves();

  bool isEthercatOperational();

  Joint getJoint(::std::string jointName);

  const std::unique_ptr<PowerDistributionBoard>& getPowerDistributionBoard() const;

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
#endif  // MARCH_HARDWARE_MARCHROBOT_H
