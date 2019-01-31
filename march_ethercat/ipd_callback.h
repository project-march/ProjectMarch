#ifndef IPD_CALLBACK_
#define IPD_CALLBACK_

#include "ros_init.h"
#include "ethercat_io.h"

#include <custom_msgs/msg/register16Msg.h>
#include <custom_msgs/msg/ECtoIPD.h>
#include <custom_msgs/msg/IPDtoEC.h>

#include "launch_parameters.h"

class Ipd_callback
{
public:
  static void get_ipd_data(string slaveName);
  static void set_ipd_data(const custom_msgs::IPDtoEC);

private:
  // upstream
  const static int16_t IpdDesiredStateLocation = 0;
  const static int16_t IpdErrorRegisterLocation = 1;

  // downstream
  const static int16_t IpdActualStateLocation = 2;
  const static int16_t IpdErrorMessageLocation = 0;
};

#endif  // IPD_CALLBACK_