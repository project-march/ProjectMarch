// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MARCH_ROBOT_H
#define MARCH_HARDWARE_MARCH_ROBOT_H
#include "march_hardware/ethercat/ethercat_master.h"
#include "march_hardware/joint.h"
#include "march_hardware/power_distribution_board/power_distribution_board.h"
#include "march_logger_cpp/base_logger.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <optional>

namespace march {
class MarchRobot {
private:
    ::std::vector<Joint> joint_list_;
    EthercatMaster ethercat_master_;
    std::optional<PowerDistributionBoard> power_distribution_board_;
    std::shared_ptr<march_logger::BaseLogger> logger_;

public:
    using iterator = std::vector<Joint>::iterator;

    MarchRobot(::std::vector<Joint> jointList, std::shared_ptr<march_logger::BaseLogger> logger, ::std::string network_interface_name,
        int ecatCycleTime, int ecatSlaveTimeout);

    MarchRobot(::std::vector<Joint> jointList, std::shared_ptr<march_logger::BaseLogger> logger,
        ::std::string network_interface_name, int ecatCycleTime, int ecatSlaveTimeout,
        std::optional<PowerDistributionBoard>);

    ~MarchRobot();

    /* Delete copy constructor/assignment since the ethercat master can not be
     * copied */
    MarchRobot(const MarchRobot&) = delete;
    MarchRobot& operator=(const MarchRobot&) = delete;

    /* Delete move constructor/assignment since atomic bool cannot be moved */
    MarchRobot(MarchRobot&&) = delete;
    MarchRobot& operator=(MarchRobot&&) = delete;

    void resetMotorControllers();

    void startEtherCAT(bool reset_motor_controllers);

    void stopEtherCAT();

    int getMaxSlaveIndex();

    bool hasValidSlaves();

    bool isEthercatOperational();

    std::exception_ptr getLastEthercatException() const noexcept;

    void waitForPdo();

    int getEthercatCycleTime() const;

    Joint& getJoint(const ::std::string& jointName);

    Joint& getJoint(size_t index);

    std::vector<Joint*> getJoints();

    size_t size() const;

    iterator begin();
    iterator end();

    bool hasPowerDistributionBoard() const;
    std::optional<PowerDistributionBoard> getPowerDistributionBoard() const;

    /**
     * Are the joints of the robot operational
     * Calls the isOperational method of the MotorControler of the joint
     * @return List of not operational Joints.
     */
    std::vector<Joint*> getNotOperationalJoints();

    /** @brief Override comparison operator */
    friend bool operator==(const MarchRobot& lhs, const MarchRobot& rhs)
    {
        if (lhs.joint_list_.size() != rhs.joint_list_.size()) {
            return false;
        }
        for (unsigned int i = 0; i < lhs.joint_list_.size(); i++) {
            const march::Joint& lhsJoint = lhs.joint_list_.at(i);
            const march::Joint& rhsJoint = rhs.joint_list_.at(i);
            if (lhsJoint != rhsJoint) {
                return false;
            }
        }
        return true;
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(std::ostream& os, const MarchRobot& marchRobot)
    {
        for (unsigned int i = 0; i < marchRobot.joint_list_.size(); i++) {
            os << marchRobot.joint_list_.at(i) << "\n";
        }
        return os;
    }
};
} // namespace march
#endif // MARCH_HARDWARE_MARCH_ROBOT_H
