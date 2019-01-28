#ifndef PDB_CALLBACK_
#define PDB_CALLBACK_

#include "ros_init.h"
#include "ethercat_io.h"

#include <std_msgs/UInt8.h>
#include <custom_msgs/ECtoPDB.h>

#include "launch_parameters.h"

class Pdb_callback
{
public:
  static void set_pdb_command(const std_msgs::UInt8& msgIn);
  static void get_pdb_data(string slaveName);
};

#endif  // PDB_CALLBACK_