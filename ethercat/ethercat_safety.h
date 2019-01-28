#ifndef ETHERCAT_SAFETY_
#define ETHERCAT_SAFETY_ 

#include "ros_init.h"
#include <custom_msgs/data8Msg.h>

#include "launch_parameters.h"

class ethercat_safety
{
public:
	static void monitor_slave_connection();
private:
	static int are_slaves_disconnected();
	static unsigned int get_error_code(int lostSlaveIndex);
	static void send_safety_message(int lostSlaveIndex);
	static int prevLostSlaveIndex;
	static int lostSlaveCounter;
};


#endif //ETHERCAT_SAFETY_