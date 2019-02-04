#include "update.h"

extern "C" {
#include "ethercat.h"
}

#define PI 3.14159265359

int tempCounter = 0;

// TODO: cleanup
uint16_t getSd_StatusWord(int sdSlave)
{
  // object 6041
  return get_input_bit16(sdSlave, 0).ui;
}

uint32_t getSd_PositionActualValue(int sdSlave)
{
  // object 6064
  return get_input_bit32(sdSlave, 2).i;
}

uint16_t getSd_MotionErrorRegister(int sdSlave)
{
  // object 2000
  return get_input_bit16(sdSlave, 6).ui;
}

int sdo_bit8(int slave, uint32_t index, uint8_t sub, uint8_t value)
{
  return ec_SDOwrite(slave, index, sub, FALSE, 1, &value, EC_TIMEOUTRXM);
}

int sdo_bit16(int slave, uint32_t index, uint8_t sub, uint16_t value)
{
  return ec_SDOwrite(slave, index, sub, FALSE, 2, &value, EC_TIMEOUTRXM);
}

int sdo_bit32(int slave, uint32_t index, uint8_t sub, uint32_t value)
{
  return ec_SDOwrite(slave, index, sub, FALSE, 4, &value, EC_TIMEOUTRXM);
}

int32_t translateToIU(int32_t motorPosition)
{
}

int pdo_mapping(int slave)
{
  printf("PDO mapping START!\n");

  // int wkc = 0;
  bool success = 1;

  //----------------------------------------

  // clear sm pdos
  success &= sdo_bit8(slave, 0x1C12, 0, 0);
  success &= sdo_bit8(slave, 0x1C13, 0, 0);

  //----------------------------------------

  // clear 1A00 pdo entries
  success &= sdo_bit32(slave, 0x1A00, 0, 0);
  // download 1A00 pdo entries
  success &= sdo_bit32(slave, 0x1A00, 1, 0x60410010);
  success &= sdo_bit32(slave, 0x1A00, 2, 0x60640020);
  success &= sdo_bit32(slave, 0x1A00, 3, 0x20000010);
  // download 1A00 pdo counslave, t: 3
  success &= sdo_bit32(slave, 0x1A00, 0, 3);
  //--------------------

  // clear 1A01 pdo entries
  success &= sdo_bit32(slave, 0x1A01, 0, 0);
  // download 1A01 pdo entries
  success &= sdo_bit32(slave, 0x1A01, 1, 0x20020010);
  success &= sdo_bit32(slave, 0x1A01, 2, 0x20550010);
  success &= sdo_bit32(slave, 0x1A01, 3, 0x20580010);
  // download 1A01 pdo counslave, t: 4
  success &= sdo_bit32(slave, 0x1A01, 0, 3);

  // clear 1A02 pdo entries
  success &= sdo_bit32(slave, 0x1A02, 0, 0);
  // download 1A02 pdo entries
  success &= sdo_bit32(slave, 0x1A02, 1, 0x60770010);
  success &= sdo_bit32(slave, 0x1A02, 2, 0x207f0010);
  success &= sdo_bit32(slave, 0x1A02, 3, 0x20880020);
  // download 1A02 pdo counslave, t: 1
  success &= sdo_bit32(slave, 0x1A02, 0, 3);
  // clear 1A03 pdo entries
  success &= sdo_bit32(slave, 0x1A03, 0, 0);

  //----------------------------------------

  // clear 1600 pdo entries
  success &= sdo_bit32(slave, 0x1600, 0, 0);
  // download 1600 pdo entries
  success &= sdo_bit32(slave, 0x1600, 1, 0x60400010);
  success &= sdo_bit32(slave, 0x1600, 2, 0x607A0020);
  // download 1600 pdo count: 2
  success &= sdo_bit32(slave, 0x1600, 0, 2);

  //--------------------

  // clear 1601 pdo entries
  success &= sdo_bit32(slave, 0x1601, 0, 0);
  // clear 1602 pdo entries
  success &= sdo_bit32(slave, 0x1602, 0, 0);
  // clear 1603 pdo entries
  success &= sdo_bit32(slave, 0x1603, 0, 0);

  //----------------------------------------

  // download 1C12:01 index
  success &= sdo_bit16(slave, 0x1C12, 1, 0x1600);
  // download 1C12 counter
  success &= sdo_bit8(slave, 0x1C12, 0, 1);

  //--------------------

  // download 1C13:01 index
  success &= sdo_bit16(slave, 0x1C13, 1, 0x1A00);
  success &= sdo_bit16(slave, 0x1C13, 2, 0x1A01);
  success &= sdo_bit16(slave, 0x1c13, 3, 0x1A02);
  // download 1C13 counter
  success &= sdo_bit8(slave, 0x1C13, 0, 3);

  //----------------------------------------

  printf(success ? "true\n" : "false\n");

  // printf("wkc pdo_mapping = (%i\n / 21)", wkc);

  return success;
}

int per_sec_to_per_sample(int speed)
{
  return speed / 1000 * *LaunchParameters::get_ethercat_cycle_time();
}

int startup_sdo(int slave)
{
  //------------------------------------------------------------------------------------------------//
  // Configure varibales to write to sdo
  //------------------------------------------------------------------------------------------------//

  bool success = 1;
  printf("Startup SDO\n");

  // string name = get_slave_name(slave);

  // jointParameters* jp = get_joint_parameters(name);

  // int softStopMax = rad_to_iu(jp->softStopMax, name);
  // int softStopMin = rad_to_iu(jp->softStopMin, name);

  // TODO: remove pow
  // int maxSpeed = per_sec_to_per_sample(rad_to_iu(jp->maxSpeed, name)) * pow(2, 16);

  //------------------------------------------------------------------------------------------------//
  // Start writing to sdo
  //------------------------------------------------------------------------------------------------//

  // mode of operation
  success &= sdo_bit8(slave, 0x6060, 0, 8);

  // limit speed -- 1 = limit active, 0 = limit inactive
  // success &= sdo_bit16(slave, 0x2086, 0, 1);

  // profile velocity -- maximum speed 16:16 notation???
  // 0x00320000 = 100 IU
  // success &= sdo_bit32(slave, 0x6081, 0, 0x000B0000);

  // position dimension index
  success &= sdo_bit8(slave, 0x608A, 0, 1);

  // position factor -- scaling factor numerator
  success &= sdo_bit32(slave, 0x6093, 1, 1);
  // postion factor -- scaling factor denominator
  success &= sdo_bit32(slave, 0x6093, 2, 1);

  // set the ethercat rate of encoder in form x*10^y
  success &= sdo_bit8(slave, 0x60C2, 1, *LaunchParameters::get_ethercat_cycle_time());
  success &= sdo_bit8(slave, 0x60C2, 2, -3);

  // time units -- numerator
  // wkc += sdo_bit32(1, 0x2071, 1, 1);
  // time units -- diviser
  // wkc += sdo_bit32(1, 0x2071, 2, 1000);

  printf(success ? "true\n" : "false\n");

  // printf("wkc startup_sdo = (%i\n / 8)", wkc);

  return success;
}

void update()
{
  ec_send_processdata();

  int wkc = ec_receive_processdata(EC_TIMEOUTRET);

  for (vector<string>::iterator i = LaunchParameters::get_list_of_GES()->begin();
       i != LaunchParameters::get_list_of_GES()->end(); ++i)
  {
    if (*i == "LUG" || *i == "RUG")
    {
      Upper_ges_callback::get_ges_data(*i);
    }
    else if (*i == "LLG" || *i == "RLG")
    {
      Lower_ges_callback::get_ges_data(*i);
    }
    else if (*i == "BPG")
    {
      // Backpack_ges_callback::get_ges_data(*i);
    }
    else if (*i == "TEMPL_GES")
    {
      // Do stuff with TEMPL_GES
//      printf("Template GES callback called\n");
      Template_ges_callback::get_ges_data(*i);

      // For now, set led command here. Todo: make this a callback for ros message
//      Template_ges_callback::set_ges_data(*i, 2);
    }
    else if (*i == "CAP_GES")
    {
      // Do stuff with GES one
//      printf("Capacity test GES callback called\n");
    }
    else
    {
      printf("error when getting ges data from EC! unknown name\n");
    }
  }

  for (int i = 0; i < *LaunchParameters::get_number_of_joints(); i++)
  {
    string slaveName = (*LaunchParameters::get_list_of_joints())[i];

    Imc_callback::get_imc_data(slaveName);
  }

  if (*LaunchParameters::get_number_of_Pdb())
  {
    Pdb_callback::get_pdb_data("PDB");
  }

  if (*LaunchParameters::get_number_of_ipd())
  {
    Ipd_callback::get_ipd_data("IPD");
  }

  ethercat_safety::monitor_slave_connection();
}
