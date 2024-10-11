// Copyright 2018 Project March.
#include "march_hardware/march_robot.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/joint.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace march {
MarchRobot::MarchRobot(
    ::std::vector<Joint> jointList,
    std::shared_ptr<march_logger::BaseLogger> logger,
    ::std::string network_interface_name,
    int ecatCycleTime,
    int ecatSlaveTimeout)
    : joint_list_(std::move(jointList)),
    ethercat_master_(std::move(network_interface_name), this->getMaxSlaveIndex(), ecatCycleTime, ecatSlaveTimeout, logger),
    logger_(std::move(logger))

{
}

MarchRobot::MarchRobot(
    ::std::vector<Joint> jointList,
    std::shared_ptr<march_logger::BaseLogger> logger,
    ::std::string network_interface_name,
    int ecatCycleTime,
    int ecatSlaveTimeout,
    std::optional<PowerDistributionBoard> power_distribution_board)
    : joint_list_(std::move(jointList)),
    ethercat_master_(std::move(network_interface_name), this->getMaxSlaveIndex(), ecatCycleTime, ecatSlaveTimeout, logger),
    power_distribution_board_(std::move(power_distribution_board)),
    logger_(std::move(logger))

{
    for (const auto& joint : joint_list_) {
        logger_->info(logger_->fstring("Robot with joint: %s", joint.getName().c_str()));
    }
}

void MarchRobot::startEtherCAT(bool reset_motor_controllers)
{
    if (!hasValidSlaves()) {
        throw error::HardwareException(error::ErrorType::INVALID_SLAVE_CONFIGURATION);
    }
    logger_->info("Slave configuration is non-conflicting");

    if (ethercat_master_.isOperational()) {
        logger_->warn("Trying to start EtherCAT while it is already active.");
        return;
    }

    bool sw_reset = ethercat_master_.start(this->joint_list_);
    //    bool sw_reset = false;

    if (reset_motor_controllers || sw_reset) {
        logger_->debug(
            logger_->fstring("Resetting all ODrives due to either: reset arg: %d or downloading of .sw file: %d",
                reset_motor_controllers, sw_reset));
        resetMotorControllers();

        logger_->info("Restarting the EtherCAT Master");
        ethercat_master_.stop();
        ethercat_master_.start(this->joint_list_);
    }
}

void MarchRobot::stopEtherCAT()
{
    if (!ethercat_master_.isOperational()) {
        logger_->warn("Trying to stop EtherCAT while it is not active.");
        return;
    }

    ethercat_master_.stop();
}

void MarchRobot::resetMotorControllers()
{
    for (auto& joint : joint_list_) {
        joint.getMotorController()->Slave::resetSlave();
    }
}

int MarchRobot::getMaxSlaveIndex()
{
    int maxSlaveIndex = -1;

    for (Joint& joint : joint_list_) {
        maxSlaveIndex = std::max((int)joint.getMotorController()->getSlaveIndex(), maxSlaveIndex);
    }
    return maxSlaveIndex;
}

bool MarchRobot::hasValidSlaves()
{
    ::std::vector<int> motorControllerIndices;
    ::std::vector<int> pdbSlaveIndices;

    for (auto& joint : joint_list_) {
        int motorControllerSlaveIndex = joint.getMotorController()->getSlaveIndex();
        motorControllerIndices.push_back(motorControllerSlaveIndex);
    }

    if (hasPowerDistributionBoard()) {
        int index = getPowerDistributionBoard().value().getSlaveIndex();
        pdbSlaveIndices.push_back(index);
    }

    // Merge the slave indices
    ::std::vector<int> slaveIndices;

    slaveIndices.reserve(motorControllerIndices.size());
    slaveIndices.insert(slaveIndices.end(), motorControllerIndices.begin(), motorControllerIndices.end());
    slaveIndices.insert(slaveIndices.end(), pdbSlaveIndices.begin(), pdbSlaveIndices.end());

    logger_->info(logger_->fstring("Found configuration for %lu slaves.", slaveIndices.size()));

    if (slaveIndices.size() == 1) {
        return true;
    }

    // Sort the indices
    ::std::sort(slaveIndices.begin(), slaveIndices.end());
    if (joint_list_[0].getMotorController()->requiresUniqueSlaves()) {
        // Check for duplicates depending on the motor controller
        auto it = ::std::unique(slaveIndices.begin(), slaveIndices.end());
        bool isUnique = (it == slaveIndices.end());
        return isUnique;
    } else {
        return true;
    }
}

bool MarchRobot::isEthercatOperational()
{
    return ethercat_master_.isOperational();
}

std::exception_ptr MarchRobot::getLastEthercatException() const noexcept
{
    return this->ethercat_master_.getLastException();
}

void MarchRobot::waitForPdo()
{
    this->ethercat_master_.waitForPdo();
}

int MarchRobot::getEthercatCycleTime() const
{
    return this->ethercat_master_.getCycleTime();
}

Joint& MarchRobot::getJoint(const ::std::string& jointName)
{
    logger_->info(logger_->fstring("Getting joint %s.", jointName.c_str()));
    if (!ethercat_master_.isOperational()) {
        logger_->warn("Trying to access joints while ethercat is not operational. "
                      "This may lead to incorrect sensor data.");
    }
    for (auto& joint : joint_list_) {
        if (joint.getName() == jointName) {
            return joint;
        }
    }

    throw std::out_of_range("Could not find joint with name " + jointName);
}

Joint& MarchRobot::getJoint(size_t index)
{
    logger_->info(logger_->fstring("Getting joint by index with name %s.", joint_list_.at(index).getName().c_str()));
    if (!ethercat_master_.isOperational()) {
        logger_->warn("Trying to access joints while ethercat is not operational. "
                      "This may lead to incorrect sensor data.");
    }
    return this->joint_list_.at(index);
}

std::vector<Joint*> MarchRobot::getJoints()
{
    if (!ethercat_master_.isOperational()) {
        logger_->warn("Trying to access joints while ethercat is not operational. "
                      "This may lead to incorrect sensor data.");
    }
    std::vector<Joint*> jointPtrs;
    jointPtrs.reserve(joint_list_.size());
    for (size_t i = 0; i < joint_list_.size(); ++i) {
        jointPtrs.push_back(&joint_list_[i]);
    }
    return jointPtrs;
}

size_t MarchRobot::size() const
{
    return this->joint_list_.size();
}

MarchRobot::iterator MarchRobot::begin()
{
    if (!ethercat_master_.isOperational()) {
        logger_->warn("Trying to access joints while ethercat is not operational. "
                      "This may lead to incorrect sensor data.");
    }
    return this->joint_list_.begin();
}

MarchRobot::iterator MarchRobot::end()
{
    return this->joint_list_.end();
}

bool MarchRobot::hasPowerDistributionBoard() const
{
    return power_distribution_board_.has_value();
}

std::optional<PowerDistributionBoard> MarchRobot::getPowerDistributionBoard() const
{
    if (hasPowerDistributionBoard()) {
        return power_distribution_board_;
    }
    return std::nullopt;
}

MarchRobot::~MarchRobot()
{
    stopEtherCAT();
}

std::vector<Joint*> MarchRobot::getNotOperationalJoints()
{
    std::vector<Joint*> not_operational_joints;
    for (size_t i = 0; i < joint_list_.size(); ++i) {
        if (!joint_list_[i].getMotorController()->getState()->isOperational()) {
            not_operational_joints.push_back(&joint_list_[i]);
        }
    }
    return not_operational_joints;
}

} // namespace march
