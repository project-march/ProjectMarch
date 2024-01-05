# ExoState.msg
std_msgs/Header header
int8 state
std_msgs/String joint

# Constant types of states
int8 SIT = 0
int8 STAND = 1
int8 WALK = 2
int8 STEPCLOSE = 3
int8 BOOTUP = 4
int8 ERROR = 5