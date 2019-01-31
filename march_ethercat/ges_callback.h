#ifndef GES_CALLBACK_
#define GES_CALLBACK_

#include "ros_init.h"
#include "ethercat_io.h"

#include <custom_msgs/register16Msg.h>
#include <custom_msgs/ECtoUG.h>
#include <custom_msgs/ECtoLG.h>

#include "launch_parameters.h"

class Ges_callback
{
public:
  // not yet implemented
  // static void set_sound(const custom_msgs::soundMsg &msgIn);

  static void get_ges_data(string slaveName){};

private:
};

class Upper_ges_callback : public Ges_callback
{
public:
  static void get_ges_data(string slaveName);

private:
  // upstream
  const static int16_t temperatureSensorHip = 0;
  const static int16_t temperatureSensorKnee = 4;
  const static int16_t buttonLocation = 8;
  const static int16_t errorCodeLocation = 9;
  // downstream
};

class Lower_ges_callback : public Ges_callback
{
public:
  static void get_ges_data(string slaveName);

private:
  // upstream
  const static int16_t temperatureSensorAnkle = 0;
  const static int16_t errorCodeLocation = 4;
  // downstream
};

class Backpack_ges_callback : public Ges_callback
{
public:
  static void get_ges_data(string slaveName);
  static void send_frequency(const custom_msgs::register16Msg& msgIn);

private:
  // upstream
  // downstream
  const static int16_t speakerLocation = 0;
};

#endif  // GES_CALLBACK_