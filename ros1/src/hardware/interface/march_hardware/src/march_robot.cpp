// Copyright 2018 Project March.
#include "march_hardware/march_robot.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/joint.h"
#include "march_hardware/temperature/temperature_sensor.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace march {
MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
    ::std::string if_name, int ecatCycleTime, int ecatSlaveTimeout)
    : jointList(std::move(jointList))
    , urdf_(std::move(urdf))
    , ethercatMaster(std::move(if_name), this->getMaxSlaveIndex(),
          ecatCycleTime, ecatSlaveTimeout)
    , pdb_(nullptr)
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
    std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
    ::std::string if_name, int ecatCycleTime, int ecatSlaveTimeout)
    : jointList(std::move(jointList))
    , urdf_(std::move(urdf))
    , ethercatMaster(std::move(if_name), this->getMaxSlaveIndex(),
          ecatCycleTime, ecatSlaveTimeout)
    , pdb_(std::move(powerDistributionBoard))
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
    std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
    std::vector<PressureSole> pressureSoles, ::std::string if_name,
    int ecatCycleTime, int ecatSlaveTimeout)
    : jointList(std::move(jointList))
    , urdf_(std::move(urdf))
    , ethercatMaster(std::move(if_name), this->getMaxSlaveIndex(),
          ecatCycleTime, ecatSlaveTimeout)
    , pdb_(std::move(powerDistributionBoard))
    , pressureSoles(std::move(pressureSoles))
{
}

void MarchRobot::startEtherCAT(bool reset_imc)
{
    if (!hasValidSlaves()) {
        throw error::HardwareException(
            error::ErrorType::INVALID_SLAVE_CONFIGURATION);
    }

    ROS_INFO("Slave configuration is non-conflicting");

    if (ethercatMaster.isOperational()) {
        ROS_WARN("Trying to start EtherCAT while it is already active.");
        return;
    }

    bool sw_reset = ethercatMaster.start(this->jointList);

    if (reset_imc || sw_reset) {
        ROS_DEBUG("Resetting all IMotionCubes due to either: reset arg: %d or "
                  "downloading of .sw fie: %d",
            reset_imc, sw_reset);
        resetMotorControllers();

        ROS_INFO("Restarting the EtherCAT Master");
        ethercatMaster.stop();
        ethercatMaster.start(this->jointList);
    }
}

void MarchRobot::stopEtherCAT()
{
    if (!ethercatMaster.isOperational()) {
        ROS_WARN("Trying to stop EtherCAT while it is not active.");
        return;
    }

    ethercatMaster.stop();
}

void MarchRobot::resetMotorControllers()
{
    for (auto& joint : jointList) {
        joint.getMotorController()->Slave::reset();
    }
}

int MarchRobot::getMaxSlaveIndex()
{
    int maxSlaveIndex = -1;

    for (Joint& joint : jointList) {
        if (joint.hasTemperatureGES()) {
            maxSlaveIndex = std::max(
                (int)joint.getTemperatureGES()->getSlaveIndex(), maxSlaveIndex);
        }
        maxSlaveIndex = std::max(
            (int)joint.getMotorController()->getSlaveIndex(), maxSlaveIndex);
    }
    return maxSlaveIndex;
}

bool MarchRobot::hasValidSlaves()
{
    ::std::vector<int> motorControllerIndices;
    ::std::vector<int> temperatureSlaveIndices;

    for (auto& joint : jointList) {
        if (joint.hasTemperatureGES()) {
            int temperatureSlaveIndex
                = joint.getTemperatureGES()->getSlaveIndex();
            temperatureSlaveIndices.push_back(temperatureSlaveIndex);
        }

        int motorControllerSlaveIndex
            = joint.getMotorController()->getSlaveIndex();
        motorControllerIndices.push_back(motorControllerSlaveIndex);
    }
    // Multiple temperature sensors may be connected to the same slave.
    // Remove duplicate temperatureSlaveIndices so they don't trigger as
    // duplicates later.
    sort(temperatureSlaveIndices.begin(), temperatureSlaveIndices.end());
    temperatureSlaveIndices.erase(
        unique(temperatureSlaveIndices.begin(), temperatureSlaveIndices.end()),
        temperatureSlaveIndices.end());

    // Merge the slave indices
    ::std::vector<int> slaveIndices;

    slaveIndices.reserve(
        motorControllerIndices.size() + temperatureSlaveIndices.size());
    slaveIndices.insert(slaveIndices.end(), motorControllerIndices.begin(),
        motorControllerIndices.end());
    slaveIndices.insert(slaveIndices.end(), temperatureSlaveIndices.begin(),
        temperatureSlaveIndices.end());

    if (slaveIndices.size() == 1) {
        ROS_INFO("Found configuration for 1 slave.");
        return true;
    }

    ROS_INFO("Found configuration for %lu slaves.", slaveIndices.size());

    // Sort the indices and check for duplicates.
    // If there are no duplicates, the configuration is valid.
    ::std::sort(slaveIndices.begin(), slaveIndices.end());
    auto it = ::std::unique(slaveIndices.begin(), slaveIndices.end());
    bool isUnique = (it == slaveIndices.end());
    return isUnique;
}

bool MarchRobot::isEthercatOperational()
{
    return ethercatMaster.isOperational();
}

std::exception_ptr MarchRobot::getLastEthercatException() const noexcept
{
    return this->ethercatMaster.getLastException();
}

void MarchRobot::waitForPdo()
{
    this->ethercatMaster.waitForPdo();
}

int MarchRobot::getEthercatCycleTime() const
{
    return this->ethercatMaster.getCycleTime();
}

Joint& MarchRobot::getJoint(const ::std::string& jointName)
{
    if (!ethercatMaster.isOperational()) {
        ROS_WARN(
            "Trying to access joints while ethercat is not operational. This "
            "may lead to incorrect sensor data.");
    }
    for (auto& joint : jointList) {
        if (joint.getName() == jointName) {
            return joint;
        }
    }

    throw std::out_of_range("Could not find joint with name " + jointName);
}

Joint& MarchRobot::getJoint(size_t index)
{
    if (!ethercatMaster.isOperational()) {
        ROS_WARN(
            "Trying to access joints while ethercat is not operational. This "
            "may lead to incorrect sensor data.");
    }
    return this->jointList.at(index);
}

size_t MarchRobot::size() const
{
    return this->jointList.size();
}

MarchRobot::iterator MarchRobot::begin()
{
    if (!ethercatMaster.isOperational()) {
        ROS_WARN(
            "Trying to access joints while ethercat is not operational. This "
            "may lead to incorrect sensor data.");
    }
    return this->jointList.begin();
}

MarchRobot::iterator MarchRobot::end()
{
    return this->jointList.end();
}

bool MarchRobot::hasPowerDistributionboard() const
{
    return this->pdb_ != nullptr;
}

PowerDistributionBoard* MarchRobot::getPowerDistributionBoard() const
{
    return this->pdb_.get();
}

bool MarchRobot::hasPressureSoles() const
{
    return pressureSoles.size() > 0;
}

std::vector<PressureSole> MarchRobot::getPressureSoles() const
{
    return pressureSoles;
}

MarchRobot::~MarchRobot()
{
    stopEtherCAT();
}

const urdf::Model& MarchRobot::getUrdf() const
{
    return this->urdf_;
}

} // namespace march
