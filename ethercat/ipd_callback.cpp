#include "ipd_callback.h"

extern "C" {
#include "ethercat.h"
}

void Ipd_callback::get_ipd_data(string slaveName)
{
  custom_msgs::ECtoIPD ipdMessage;
  ipdMessage.slaveName = slaveName;

  int slaveNumber = LaunchParameters::get_slave_number(slaveName);

  ipdMessage.IpdErrorRegister = get_input_bit8(slaveNumber, IpdErrorRegisterLocation).ui;
  ipdMessage.IpdDesiredState = get_input_bit8(slaveNumber, IpdDesiredStateLocation).i;

  publish_to_ipd_handler(ipdMessage);
}

void Ipd_callback::set_ipd_data(const custom_msgs::IPDtoEC msgIn)
{
  int8_t slaveNumber = LaunchParameters::get_slave_number("IPD");

  union bit8 masterState;
  union bit16 errorMessage;

  masterState.i = msgIn.masterState;
  errorMessage.ui = (uint16_t)msgIn.errorMessage;

  set_output_bit8(slaveNumber, IpdActualStateLocation, masterState);
  set_output_bit16(slaveNumber, IpdErrorMessageLocation, errorMessage);
}