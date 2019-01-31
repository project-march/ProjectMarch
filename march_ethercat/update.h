#ifndef UPDATE_
#define UPDATE_

#include "ges_callback.h"
#include "imc_callback.h"
#include "pdb_callback.h"
#include "ipd_callback.h"
#include "ethercat_safety.h"
#include "ethercat_io.h"
#include <math.h>

#include "launch_parameters.h"

int pdo_mapping(int slave);
int startup_sdo(int slave);
void update();

#endif