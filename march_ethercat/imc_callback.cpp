#include "imc_callback.h"

extern "C" {
#include "ethercat.h"
}

// Servodrive params IN
//--------------------------------------------------------------------------------------//
int16_t controlwordLocation = 0;
int16_t targetPositionLocation = 2;

// Servodrive params OUT
//--------------------------------------------------------------------------------------//
int16_t statuswordLocation = 0;
int16_t actualPositionLocation = 2;
int16_t motionErrorRegisterLocation = 6;
int16_t detailedErrorRegisterLocation = 8;
int16_t motorVoltageLocation = 10;
int16_t temperatureLocation = 12;
int16_t motorCurrentLocation = 14;
int16_t currentLimitLocation = 16;
int16_t motorPositionLocation = 18;

// Servodrive RPDO
//--------------------------------------------------------------------------------------//

void Imc_callback::set_target_position(const custom_msgs::IUPos& msgIn)
{
  union bit32 targetPosition;
  string slaveName;

  targetPosition.i = msgIn.position;
  slaveName = msgIn.slaveName;

  int8_t slaveNumber = LaunchParameters::get_slave_number(slaveName);

  set_output_bit32(slaveNumber, targetPositionLocation, targetPosition);
}

void Imc_callback::set_controlword(const custom_msgs::register16Msg& msgIn)
{
  union bit16 controlword;
  string slaveName;

  controlword.i = msgIn.data;
  slaveName = msgIn.slaveName;

  int8_t slaveNumber = LaunchParameters::get_slave_number(slaveName);
  ;

  set_output_bit16(slaveNumber, controlwordLocation, controlword);
}

// Servodrive TPDO
//--------------------------------------------------------------------------------------//

void Imc_callback::get_imc_data(string slaveName)
{
  custom_msgs::ECtoJH outputmsg;
  int slaveNumber = LaunchParameters::get_slave_number(slaveName);

  outputmsg.slaveName = slaveName;
  outputmsg.position = get_input_bit32(slaveNumber, actualPositionLocation).i;
  outputmsg.motionError = get_input_bit16(slaveNumber, motionErrorRegisterLocation).ui;
  outputmsg.statusword = get_input_bit16(slaveNumber, statuswordLocation).ui;
  outputmsg.detailedError = get_input_bit16(slaveNumber, detailedErrorRegisterLocation).ui;
  outputmsg.temperature = get_input_bit16(slaveNumber, temperatureLocation).ui;
  outputmsg.motorVoltage = get_input_bit16(slaveNumber, motorVoltageLocation).ui;
  outputmsg.motorCurrent = get_input_bit16(slaveNumber, motorCurrentLocation).i;
  outputmsg.currentLimit = get_input_bit16(slaveNumber, currentLimitLocation).ui;
  outputmsg.motorPosition = get_input_bit32(slaveNumber, motorPositionLocation).i;

  publish_to_joint_handler(slaveName, outputmsg);
}

// Servodrive SDO
//--------------------------------------------------------------------------------------//
void Imc_callback::set_SDO(const custom_msgs::SDOMsg& msgIn)
{
  string slaveName = msgIn.slaveName;
  uint32_t index = msgIn.index;
  uint8_t sub = msgIn.sub;
  uint32_t value = msgIn.value;
  int8_t valueType = msgIn.valueType;

  int8_t slaveNumber = LaunchParameters::get_slave_number(slaveName);

  ec_SDOwrite(slaveNumber, index, sub, FALSE, valueType, &value, EC_TIMEOUTRXM);
}
