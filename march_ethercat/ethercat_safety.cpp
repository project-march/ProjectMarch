#include "ethercat_safety.h"

extern "C" {
#include "ethercat.h"
}

int ethercat_safety::prevLostSlaveIndex = 0;
int ethercat_safety::lostSlaveCounter = 0;

void ethercat_safety::monitor_slave_connection()
{
  int lostSlaveIndex = are_slaves_disconnected();

  if (lostSlaveIndex || prevLostSlaveIndex)
  {
    send_safety_message(lostSlaveIndex);
    prevLostSlaveIndex = lostSlaveIndex;
  }
}

int ethercat_safety::are_slaves_disconnected()
{
  for (int slave = 1; slave <= ec_slavecount; slave++)
  {
    if (ec_slave[slave].islost)
    {
      lostSlaveCounter++;

      // longest ecountered missing timeis 16 ticks
      if (lostSlaveCounter >= 40)
      {
        return slave;
      }

      return 0;
    }
  }

  lostSlaveCounter = 0;
  return 0;
}

unsigned int ethercat_safety::get_error_code(int lostSlaveIndex)
{
  if (!lostSlaveIndex)
    return 0;

  // Todo: lostSlaveIndex <= slavenumber of PDB
  if (lostSlaveIndex <= 1)
    return 0b010;

  if (lostSlaveIndex == ec_slavecount)
    return 0b001;

  return 0b100;
}

void ethercat_safety::send_safety_message(int lostSlaveIndex)
{
  custom_msgs::data8Msg safetyMassage;

  // Todo: Fix this
  // safetyMassage.slaveName = LaunchParameters::get_slave_name(lostSlaveIndex);
  // safetyMassage.data = get_error_code(lostSlaveIndex);
  // publish_to_safety(safetyMassage);
}