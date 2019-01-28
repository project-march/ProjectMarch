#ifndef IMC_CALLBACK_
#define IMC_CALLBACK_

#include "ros_init.h"
#include "ethercat_io.h"
#include <string>

#include <custom_msgs/IUPos.h>
#include <custom_msgs/SDOMsg.h>
#include <custom_msgs/register16Msg.h>
#include <custom_msgs/ECtoJH.h>

#include "launch_parameters.h"


class Imc_callback
{
public: 

	// static void set_imc_data(const custom_msgs::JHtoEC &msgIn);
	static void get_imc_data(string slavename);

	static void set_target_position(const custom_msgs::IUPos &msgIn);
	static void set_controlword(const custom_msgs::register16Msg &msgIn);

	static void set_SDO(const custom_msgs::SDOMsg &msgIn);


};

#endif //IMC_CALLBACK_