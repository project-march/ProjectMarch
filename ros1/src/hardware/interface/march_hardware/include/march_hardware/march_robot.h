// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MARCH_ROBOT_H
#define MARCH_HARDWARE_MARCH_ROBOT_H
#include "march_hardware/ethercat/ethercat_master.h"
#include "march_hardware/joint.h"
#include "march_hardware/power/power_distribution_board.h"
#include "march_hardware/pressure_sole/pressure_sole.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

namespace march
{
class MarchRobot
{
private:
  ::std::vector<Joint> jointList;
  urdf::Model urdf_;
  std::unique_ptr<EthercatMaster> ethercatMaster;
  std::unique_ptr<PowerDistributionBoard> pdb_;
  std::vector<PressureSole> pressureSoles;

public:
  using iterator = std::vector<Joint>::iterator;

  MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, std::unique_ptr<EthercatMaster> ethercatMaster);

  MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
             std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
             std::unique_ptr<EthercatMaster> ethercatMaster);

  MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
             std::unique_ptr<PowerDistributionBoard> powerDistributionBoard);

  MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
             std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
             std::vector<PressureSole> pressureSoles);

  ~MarchRobot();

  /* Delete copy constructor/assignment since the ethercat master can not be copied */
  MarchRobot(const MarchRobot&) = delete;
  MarchRobot& operator=(const MarchRobot&) = delete;

  /* Delete move constructor/assignment since atomic bool cannot be moved */
  MarchRobot(MarchRobot&&) = delete;
  MarchRobot& operator=(MarchRobot&&) = delete;

  void resetMotorControllers();

  void startCommunication(bool reset_motor_controllers);

  void stopCommunication();

  bool hasValidSlaves();

  bool isEthercatOperational();

  bool isCommunicationOperational();

  std::exception_ptr getLastCommunicationException() const noexcept;

  void waitForUpdate();

  int getCycleTime() const;

  Joint& getJoint(::std::string jointName);

  Joint& getJoint(size_t index);

  size_t size() const;

  iterator begin();
  iterator end();

  bool hasPowerDistributionboard() const;
  PowerDistributionBoard* getPowerDistributionBoard() const;

  bool hasPressureSoles() const;
  std::vector<PressureSole> getPressureSoles() const;

  const urdf::Model& getUrdf() const;

  /** @brief Override comparison operator */
  friend bool operator==(const MarchRobot& lhs, const MarchRobot& rhs)
  {
    if (lhs.jointList.size() != rhs.jointList.size())
    {
      return false;
    }
    for (unsigned int i = 0; i < lhs.jointList.size(); i++)
    {
      const march::Joint& lhsJoint = lhs.jointList.at(i);
      const march::Joint& rhsJoint = rhs.jointList.at(i);
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
#endif  // MARCH_HARDWARE_MARCH_ROBOT_H
