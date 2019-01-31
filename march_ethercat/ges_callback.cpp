#include "ges_callback.h"

extern "C" {
#include "ethercat.h"
}

void Upper_ges_callback::get_ges_data(string slaveName)
{
  custom_msgs::ECtoUG gesMessage;
  gesMessage.slaveName = slaveName;

  int slaveNumber = LaunchParameters::get_slave_number(slaveName);

  gesMessage.hipTemperature = get_input_bit32(slaveNumber, temperatureSensorHip).f;
  gesMessage.kneeTemperature = get_input_bit32(slaveNumber, temperatureSensorKnee).f;
  gesMessage.manualInputState = get_input_bit8(slaveNumber, buttonLocation).i;
  gesMessage.errorCode = get_input_bit8(slaveNumber, errorCodeLocation).ui;

  publish_to_ges_handler(slaveName, gesMessage);
}

void Lower_ges_callback::get_ges_data(string slaveName)
{
  custom_msgs::ECtoLG gesMessage;
  gesMessage.slaveName = slaveName;

  int slaveNumber = LaunchParameters::get_slave_number(slaveName);

  gesMessage.ankleTemperature = get_input_bit32(slaveNumber, temperatureSensorAnkle).f;
  gesMessage.errorCode = get_input_bit8(slaveNumber, errorCodeLocation).ui;

  publish_to_ges_handler(slaveName, gesMessage);
}

// void Backpack_ges_callback::get_ges_data(string slaveName)
// {
// 	custom_msgs::ECtoBPG gesMessage;
// 	gesMessage.slaveName = slaveName;

// 	int slaveNumber = get_slave_number(slaveName);

// 	gesMessage.

// 	publish_to_ges_handler(slaveName, gesMessage);
// }

void Backpack_ges_callback::send_frequency(const custom_msgs::register16Msg& msgIn)
{
  union bit16 frequency;

  frequency.ui = msgIn.data;

  int8_t slaveNumber = LaunchParameters::get_slave_number(msgIn.slaveName);

  set_output_bit16(slaveNumber, speakerLocation, frequency);
}