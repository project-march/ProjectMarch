#include "pdb_callback.h"

//----------------------------------------------------------------------
// PDB Down
int16_t commandDownLocation = 0;

//----------------------------------------------------------------------
// PDB Up
int16_t commandUpLocation = 0;

int16_t LVNetCurrentLocation = 1;
int16_t PDBNetCurrentLocation = 3;

int16_t HVConnectionsStatesLocation = 5;
int16_t HVConnectionTriggerLocation = 7;
int16_t HVConnectionI2tLocation = 8;

int16_t HVConnectionsCurrent1Location = 9;
int16_t HVConnectionsCurrent2Location = 11;
int16_t HVConnectionsCurrent3Location = 13;
int16_t HVConnectionsCurrent4Location = 15;
int16_t HVConnectionsCurrent5Location = 17;
int16_t HVConnectionsCurrent6Location = 19;

void Pdb_callback::set_pdb_command(const std_msgs::UInt8& msgIn)
{
  union bit8 pdbCommand;

  pdbCommand.ui = msgIn.data;
  int8_t slaveNumber = LaunchParameters::get_slave_number("PDB");

  set_output_bit8(slaveNumber, commandDownLocation, pdbCommand);
}

void Pdb_callback::get_pdb_data(string slaveName)
{
  custom_msgs::ECtoPDB PDBdata;
  PDBdata.slaveName = slaveName;

  int slaveNumber = LaunchParameters::get_slave_number(slaveName);

  PDBdata.command = get_input_bit8(slaveNumber, commandUpLocation).i;
  PDBdata.HVConnectionsStates = get_input_bit16(slaveNumber, HVConnectionsStatesLocation).ui;
  PDBdata.HVConnectionsTrigger = get_input_bit8(slaveNumber, HVConnectionTriggerLocation).i;
  PDBdata.HVConnectionsI2t = get_input_bit8(slaveNumber, HVConnectionI2tLocation).ui;
  PDBdata.LVNetCurrent = get_input_bit16(slaveNumber, LVNetCurrentLocation).i;
  PDBdata.PDBNetCurrent = get_input_bit16(slaveNumber, PDBNetCurrentLocation).i;

  PDBdata.HVConnectionsCurrent1 = get_input_bit16(slaveNumber, HVConnectionsCurrent1Location).i;
  PDBdata.HVConnectionsCurrent2 = get_input_bit16(slaveNumber, HVConnectionsCurrent2Location).i;
  PDBdata.HVConnectionsCurrent3 = get_input_bit16(slaveNumber, HVConnectionsCurrent3Location).i;
  PDBdata.HVConnectionsCurrent4 = get_input_bit16(slaveNumber, HVConnectionsCurrent4Location).i;
  PDBdata.HVConnectionsCurrent5 = get_input_bit16(slaveNumber, HVConnectionsCurrent5Location).i;
  PDBdata.HVConnectionsCurrent6 = get_input_bit16(slaveNumber, HVConnectionsCurrent6Location).i;

  publish_to_pdb_handler(PDBdata);
}
