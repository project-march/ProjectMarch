// Copyright 2019 Project March.
#include <ros/ros.h>

#include <bitset>
#include <string>

#include <march_hardware/error/motion_error.h>
#include <march_hardware/Joint.h>

namespace march
{
void Joint::initialize(int ecatCycleTime)
{
  if (hasIMotionCube())
  {
    iMotionCube.writeInitialSDOs(ecatCycleTime);
  }
  if (hasTemperatureGES())
  {
    temperatureGES.writeInitialSDOs(ecatCycleTime);
  }
}

void Joint::prepareActuation()
{
  ROS_ASSERT_MSG(this->getActuationMode() != ActuationMode::unknown, "The mode of actuation for joint %s is: %s",
                 this->name.c_str(), this->getActuationMode().toString().c_str());
  if (this->allowActuation)
  {
    ROS_INFO("[%s] Preparing for actuation", this->name.c_str());
    this->iMotionCube.goToOperationEnabled();
    ROS_INFO("[%s] Successfully prepared for actuation", this->name.c_str());
  }
  else
  {
    ROS_ERROR("[%s] Trying to prepare for actuation while it is not "
              "allowed to actuate",
              this->name.c_str());
  }

  this->incremental_position_ = this->iMotionCube.getAngleRadIncremental();
  this->absolute_position_ = this->iMotionCube.getAngleRadAbsolute();
  this->position_ = this->absolute_position_;
  this->velocity_ = 0;
}

void Joint::actuateRad(double targetPositionRad)
{
  ROS_ASSERT_MSG(this->allowActuation,
                 "Joint %s is not allowed to actuate, "
                 "yet its actuate method has been called",
                 this->name.c_str());
  // TODO(BaCo) check that the position is allowed and does not exceed (torque)
  // limits.
  this->iMotionCube.actuateRad(targetPositionRad);
}

void Joint::readEncoders(const ros::Duration& elapsed_time)
{
  const double new_incremental_position = this->iMotionCube.getAngleRadIncremental();
  const double new_absolute_position = this->iMotionCube.getAngleRadAbsolute();

  // Get velocity from encoder position
  const double incremental_velocity = (new_incremental_position - this->incremental_position_) / elapsed_time.toSec();
  const double absolute_velocity = (new_absolute_position - this->absolute_position_) / elapsed_time.toSec();

  if (std::abs(incremental_velocity - absolute_velocity) * elapsed_time.toSec() >
      2 * (this->iMotionCube.getAbsoluteRadPerBit() + this->iMotionCube.getIncrementalRadPerBit()))
  {
    // Take the velocity that is closest to that of the previous timestep.
    if (std::abs(incremental_velocity - this->velocity_) < std::abs(absolute_velocity - this->velocity_))
    {
      this->velocity_ = incremental_velocity;
    }
    else
    {
      this->velocity_ = absolute_velocity;
    }
  }
  else
  {
    // Recalibrate joint position if it has drifted
    if (std::abs(this->absolute_position_ - this->position_) > 2 * this->iMotionCube.getAbsoluteRadPerBit())
    {
      this->position_ = this->absolute_position_;
      ROS_INFO("Recalibrated joint position %s", this->name.c_str());
    }

    // Take the velocity with the highest resolution.
    if (this->iMotionCube.getIncrementalRadPerBit() < this->iMotionCube.getAbsoluteRadPerBit())
    {
      this->velocity_ = incremental_velocity;
    }
    else
    {
      this->velocity_ = absolute_velocity;
    }
  }

  // Update position with the most accurate velocity
  this->position_ += this->velocity_ * elapsed_time.toSec();
  this->incremental_position_ = new_incremental_position;
  this->absolute_position_ = new_absolute_position;
}

double Joint::getPosition()
{
  return this->position_;
}

double Joint::getVelocity()
{
  return this->velocity_;
}

void Joint::actuateTorque(int16_t targetTorque)
{
  ROS_ASSERT_MSG(this->allowActuation,
                 "Joint %s is not allowed to actuate, "
                 "yet its actuate method has been called",
                 this->name.c_str());
  this->iMotionCube.actuateTorque(targetTorque);
}

int16_t Joint::getTorque()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getTorque();
}

int32_t Joint::getAngleIUAbsolute()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleIUAbsolute();
}

int32_t Joint::getAngleIUIncremental()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleIUIncremental();
}

float Joint::getTemperature()
{
  if (!hasTemperatureGES())
  {
    ROS_WARN("[%s] Has no temperature sensor", this->name.c_str());
    return -1;
  }
  return this->temperatureGES.getTemperature();
}

IMotionCubeState Joint::getIMotionCubeState()
{
  IMotionCubeState states;

  std::bitset<16> statusWordBits = this->iMotionCube.getStatusWord();
  states.statusWord = statusWordBits.to_string();
  std::bitset<16> detailedErrorBits = this->iMotionCube.getDetailedError();
  states.detailedError = detailedErrorBits.to_string();
  std::bitset<16> motionErrorBits = this->iMotionCube.getMotionError();
  states.motionError = motionErrorBits.to_string();

  states.state = IMCState(this->iMotionCube.getStatusWord());
  states.detailedErrorDescription = error::parseDetailedError(this->iMotionCube.getDetailedError());
  states.motionErrorDescription = error::parseMotionError(this->iMotionCube.getMotionError());

  states.motorCurrent = this->iMotionCube.getMotorCurrent();
  states.motorVoltage = this->iMotionCube.getMotorVoltage();

  states.incrementalEncoderValue = this->iMotionCube.getAngleIUIncremental();

  return states;
}

void Joint::setName(const std::string& name)
{
  Joint::name = name;
}
void Joint::setAllowActuation(bool allowActuation)
{
  Joint::allowActuation = allowActuation;
}

void Joint::setTemperatureGes(const TemperatureGES& temperatureGes)
{
  temperatureGES = temperatureGes;
}

int Joint::getTemperatureGESSlaveIndex()
{
  if (hasTemperatureGES())
  {
    return this->temperatureGES.getSlaveIndex();
  }
  return -1;
}

int Joint::getIMotionCubeSlaveIndex()
{
  if (hasIMotionCube())
  {
    return this->iMotionCube.getSlaveIndex();
  }
  return -1;
}
std::string Joint::getName()
{
  return this->name;
}

bool Joint::hasIMotionCube()
{
  return this->iMotionCube.getSlaveIndex() != -1;
}

bool Joint::hasTemperatureGES()
{
  return this->temperatureGES.getSlaveIndex() != -1;
}

bool Joint::canActuate()
{
  return this->allowActuation;
}

void Joint::setNetNumber(int netNumber)
{
  Joint::netNumber = netNumber;
}

ActuationMode Joint::getActuationMode() const
{
  return this->iMotionCube.getActuationMode();
}

}  // namespace march
