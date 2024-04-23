# ExoMode.msg
std_msgs/Header header
int8 mode
std_msgs/String joint

# Constant types of modes
int8 SIT = 0
int8 STAND = 1
int8 WALK = 2
int8 STEPCLOSE = 3
int8 BOOTUP = 4
int8 ERROR = 5